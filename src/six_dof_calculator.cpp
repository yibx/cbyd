#include "six_dof_calculator.h"
#include <chrono>
#include <cstdlib>
#include <iostream>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/registration/gicp.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/ndt.h>
#include <cmath>
#include <thread>
#include <mutex>
#include <queue>
#include <condition_variable>
#include <vector>
#include <functional>
#include <pthread.h>

#include <string>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include "Logger.h"

#include "system_state_manager.h"
using SM = SystemStateManager;

using namespace std;

using PointT = pcl::PointXYZ;
using PointCloudT = pcl::PointCloud<PointT>;

// 飞腾D2000 最优固定参数（速度+精度平衡）
#define PREPROCESS_LEAF_SIZE 0.15f
#define NDT_RESOLUTION 0.6f
#define GICP_LEAF_SIZE 0.05f
#define GICP_MAX_CORR 0.3f


const double SamplePeriod = 0.1;
const int StableFrameNum = 30;


// 快速裁剪
template<typename T>
inline T clamp_val(T val, T min_val, T max_val)
{
    return (val < min_val) ? min_val : (val > max_val) ? max_val : val;
}

// 快速角度包装
inline float wrapAngle(float angle) {
    angle = fmod(angle + M_PI, 2 * M_PI);
    if (angle < 0) angle += 2 * M_PI;
    return angle - M_PI;
}

using FPFHSignature = pcl::FPFHSignature33;

// 输入旋转矩阵 R，输出 欧拉角 (rad)
void getEulerAngles(const Eigen::Matrix3f& R, float& roll, float& pitch, float& yaw)
{
    roll = atan2(R(2, 1), R(2, 2));    // 横摇
    pitch = asin(-R(2, 0));           // 纵摇
    yaw = atan2(R(0, 1), R(0, 0));     // 偏航（最重要）
}

// 构造函数：初始化线程池
SixDofCalculator::SixDofCalculator(LockFreeRingQueue<FusedPointCloud>* rq_fuse, LockFreeRingQueue<SixDofResult>* rq_sixdof)
    : rq_fuse_(rq_fuse), rq_sixdof_(rq_sixdof), pool_running_(true)
{
#if 0
    // ==============================================
    // 自动构建船舶坐标系
    // ==============================================
    cout << "===== 三点构建船舶坐标系 =====" << endl;
     雷达坐标系下的 3 个船体特征点
    Point3d p1_radar(10.0, 5.0, 0.0);   // 船艏
    Point3d p2_radar(20.0, 5.0, 0.0);   // 船舯
    Point3d p3_radar(10.0, 10.0, 0.0);  // 船左舷
    bool build_ok = frame_builder_.buildShipFrameFrom3Points(p1_radar, p2_radar, p3_radar);
    if (!build_ok) {
        cout << "建系失败！三点共线或重合" << endl;
    }
    lr_sr_ = frame_builder_.getShipRotationMatrix();

    // 把【虚拟质心】从雷达坐标 → 船舶坐标
    Point3d virtual_centroid_radar(15.0, 7.0, 0.0);  // 雷达下质心
    Point3d centroid_ship = frame_builder_.radarToShip(virtual_centroid_radar);
    cout << "雷达坐标系质心: " << virtual_centroid_radar.x << ", " << virtual_centroid_radar.y << endl;
    cout << "船舶坐标系质心: " << centroid_ship.x << ", " << centroid_ship.y << endl << endl;

    cout << "===== 2. 拟合码头直线 =====" << endl;

    // 雷达坐标系下码头边缘若干点
    vector<Point3d> wharf_points = {
        Point3d(0, 0, 0),
        Point3d(5, 0, 0),
        Point3d(10, 0, 0),
        Point3d(15, 0, 0),
        Point3d(20, 0, 0)
    };

    dock_checker_.fitWharfLine(wharf_points);
    double wharf_yaw = dock_checker_.calcWharfYawDegree();
    cout << "码头朝向角（固定）: " << wharf_yaw << " 度" << endl << endl;
#endif
    if (!loadRegParam("../reg_config.yaml", regCfg_)){
        return;
    }

    b_set_base_ = false;
    // 飞腾D2000 8核推荐：6个工作线程
    int thread_num = 6;
    for (int i = 0; i < thread_num; ++i) {
        workers_.emplace_back(&SixDofCalculator::workerThread, this, i);
    }
}

// 线程池工作线程（绑定CPU核心）
void SixDofCalculator::workerThread(int core_id)
{
    // 绑定飞腾物理核心
    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    CPU_SET(core_id % 8, &cpuset);
    pthread_setaffinity_np(pthread_self(), sizeof(cpu_set_t), &cpuset);

    while (true) {
        function<void()> task;
        {
            unique_lock<mutex> lock(task_mtx_);
            cv_.wait(lock, [this]() { return !pool_running_ || !tasks_.empty(); });

            if (!pool_running_ && tasks_.empty()) return;
            task = move(tasks_.front());
            tasks_.pop();
        }
	auto start = std::chrono::high_resolution_clock::now();
        task();
	auto end = std::chrono::high_resolution_clock::now();
        double elapsed_ms = std::chrono::duration<double, std::milli>(end - start).count();
	std::cout << "single: " << elapsed_ms << std::endl;
    }
}

void SixDofCalculator::start() {
    if (is_running_) return;
    is_running_ = true;
    thread_ = thread(&SixDofCalculator::calcLoop, this);
}

void SixDofCalculator::stop() {
    is_running_ = false;

    // 关闭线程池
    {
        lock_guard<mutex> lock(task_mtx_);
        pool_running_ = false;
    }
    cv_.notify_all();
    for (auto& t : workers_) {
        if (t.joinable()) t.join();
    }

    if (thread_.joinable()) thread_.join();
}

// 主线程：仅分发任务
void SixDofCalculator::calcLoop() {
    while (is_running_) {
        FusedPointCloud c;
        if (rq_fuse_->dequeue(c)) 
        {
	        {
                lock_guard<mutex> lock(task_mtx_);
	        	if(tasks_.size() >= 30) {
	        		tasks_.pop();
	        	}
                tasks_.emplace([this, c = move(c)]() {
                    auto res = calculateSixDof(c);
		    	    int retry = 0;
		    	    const int max_retry = 10;
                    while (!rq_sixdof_->enqueue(res) && retry < max_retry) {
                    	this_thread::sleep_for(chrono::milliseconds(3 * (1 << retry)));
		    		    retry++;
                    }
		    	    if (retry >= max_retry) {
		    	    	cerr << "六自由度结果入队失败（重试" << max_retry << "次)，丢弃结果" << std::endl;
		    	    }
                });
                cv_.notify_one();
	        }
	        this_thread::sleep_for(chrono::milliseconds(10));
        } else {
            this_thread::sleep_for(chrono::milliseconds(3));
        }
    }
}

// 极速预处理（仅体素滤波，无统计滤波）
PointCloudT::Ptr preprocess(const PointCloudT::Ptr& cloud)
{
    if (cloud->empty()) return cloud;
    PointCloudT::Ptr out(new PointCloudT);

    pcl::VoxelGrid<PointT> voxel;
    voxel.setLeafSize(PREPROCESS_LEAF_SIZE, PREPROCESS_LEAF_SIZE, PREPROCESS_LEAF_SIZE);
    voxel.setInputCloud(cloud);
    voxel.filter(*out);

    return out;
}
/*
// NDT 粗配准（固定参数，极速）
Eigen::Matrix4f coarseRegistration(PointCloudT::Ptr& src, PointCloudT::Ptr& dst)
{
    if (src->empty() || dst->empty())
        return Eigen::Matrix4f::Identity();

    pcl::NormalDistributionsTransform<PointT, PointT> ndt;
    ndt.setInputSource(src);
    ndt.setInputTarget(dst);

    ndt.setResolution(NDT_RESOLUTION);
    ndt.setStepSize(0.1f);
    ndt.setMaximumIterations(25);
    ndt.setTransformationEpsilon(1e-6);

    Eigen::Matrix4f init_guess = Eigen::Matrix4f::Identity();
    PointCloudT::Ptr output(new PointCloudT);
    ndt.align(*output, init_guess);

    if (!ndt.hasConverged() || ndt.getFitnessScore() > 2.0f) {
        Eigen::Vector4f c1, c2;
        pcl::compute3DCentroid(*src, c1);
        pcl::compute3DCentroid(*dst, c2);
        Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
        T(0,3) = c2[0]-c1[0];
        T(1,3) = c2[1]-c1[1];
        T(2,3) = c2[2]-c1[2];
        return T;
    }

    return ndt.getFinalTransformation();
}

// GICP 精配准（极速稳定版）
Eigen::Matrix4f fineRegistrationGICP(PointCloudT::Ptr& src, PointCloudT::Ptr& dst, Eigen::Matrix4f init_trans)
{
    if (src->empty() || dst->empty())
        return init_trans;

    PointCloudT::Ptr src_filtered(new PointCloudT);
    PointCloudT::Ptr dst_filtered(new PointCloudT);
    pcl::VoxelGrid<PointT> vg;
    vg.setLeafSize(GICP_LEAF_SIZE, GICP_LEAF_SIZE, GICP_LEAF_SIZE);
    vg.setInputCloud(src); vg.filter(*src_filtered);
    vg.setInputCloud(dst); vg.filter(*dst_filtered);

    if (src_filtered->empty() || dst_filtered->empty())
        return init_trans;

    pcl::GeneralizedIterativeClosestPoint<PointT, PointT> gicp;
    gicp.setInputSource(src_filtered);
    gicp.setInputTarget(dst_filtered);

    gicp.setUseReciprocalCorrespondences(true);
    gicp.setMaxCorrespondenceDistance(GICP_MAX_CORR);
    gicp.setMaximumIterations(80);
    gicp.setTransformationEpsilon(1e-6);
    gicp.setRotationEpsilon(1e-3);

    PointCloudT::Ptr aligned(new PointCloudT);
    gicp.align(*aligned, init_trans);

    if (!gicp.hasConverged() || gicp.getFitnessScore(GICP_MAX_CORR) > 0.8f) {
        SM::instance().reportError(ModuleType::SIX_DOF_CALC, ErrorLevel::STATUS_ERROR, "位姿解算失败");
        return init_trans;
    }

    return gicp.getFinalTransformation();
}
*/

Eigen::Matrix4f coarseRegistration(PointCloudT::Ptr& src, PointCloudT::Ptr& dst, const RegParam& cfg)
{
    if (src->empty() || dst->empty())
        return Eigen::Matrix4f::Identity();

    const NdtParam& ndt_param = cfg.ndt;

    // 降采样（使用配置参数）
    PointCloudT::Ptr src_down(new PointCloudT);
    PointCloudT::Ptr dst_down(new PointCloudT);
    pcl::VoxelGrid<PointT> vg;
    vg.setLeafSize(ndt_param.voxel_x, ndt_param.voxel_y, ndt_param.voxel_z);
    vg.setInputCloud(src);
    vg.filter(*src_down);
    vg.setInputCloud(dst);
    vg.filter(*dst_down);

    // NDT 配准
    pcl::NormalDistributionsTransform<PointT, PointT> ndt;
    ndt.setInputSource(src_down);
    ndt.setInputTarget(dst_down);

    ndt.setMaximumIterations(ndt_param.max_iter);
    ndt.setTransformationEpsilon(ndt_param.trans_eps);
    ndt.setStepSize(ndt_param.step_size);
    ndt.setResolution(ndt_param.resolution);

    Eigen::Matrix4f init_guess = Eigen::Matrix4f::Identity();
    PointCloudT::Ptr output(new PointCloudT);
    ndt.align(*output, init_guess);

    float score = ndt.getFitnessScore();
    std::cout << "NDT 分数: " << score << std::endl;

    // 配准失败，使用质心平移兜底
    if (!ndt.hasConverged() || score > ndt_param.fit_thresh)
    {
        Eigen::Vector4f c1, c2;
        pcl::compute3DCentroid(*src_down, c1);
        pcl::compute3DCentroid(*dst_down, c2);
        Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
        T(0, 3) = c2[0] - c1[0];
        T(1, 3) = c2[1] - c1[1];
        T(2, 3) = c2[2] - c1[2];
        return T;
    }

    return ndt.getFinalTransformation();
}

Eigen::Matrix4f fineRegistrationGICP(PointCloudT::Ptr& src, PointCloudT::Ptr& dst,
                                      Eigen::Matrix4f init_trans, const RegParam& cfg)
{
    if (src->empty() || dst->empty())
        return init_trans;

    const GicpParam& gicp_param = cfg.gicp;

    // 轻量级降采样
    PointCloudT::Ptr src_filtered(new PointCloudT);
    PointCloudT::Ptr dst_filtered(new PointCloudT);
    pcl::VoxelGrid<PointT> vg;
    vg.setLeafSize(gicp_param.voxel_x, gicp_param.voxel_y, gicp_param.voxel_z);
    vg.setInputCloud(src);
    vg.filter(*src_filtered);
    vg.setInputCloud(dst);
    vg.filter(*dst_filtered);

    // GICP 初始化
    pcl::GeneralizedIterativeClosestPoint<PointT, PointT> gicp;
    gicp.setInputSource(src_filtered);
    gicp.setInputTarget(dst_filtered);

    gicp.setUseReciprocalCorrespondences(gicp_param.use_reciprocal);
    gicp.setMaxCorrespondenceDistance(gicp_param.max_corr_dist);
    gicp.setMaximumIterations(gicp_param.max_iter);
    gicp.setTransformationEpsilon(gicp_param.trans_eps);
    gicp.setEuclideanFitnessEpsilon(gicp_param.euclid_eps);
    gicp.setRotationEpsilon(gicp_param.rot_eps);
    gicp.setCorrespondenceRandomness(gicp_param.rand_num);

    PointCloudT::Ptr aligned(new PointCloudT);
    gicp.align(*aligned, init_trans);

    float fitness = gicp.getFitnessScore(gicp_param.score_radius);
    bool converged = gicp.hasConverged();

    std::cout << "GICP 收敛: " << converged << " 分数: " << fitness << std::endl;

    // 配准不佳，保留上一帧姿态
    if (!converged || fitness > gicp_param.fit_thresh)
    {
        std::cout << "→ 配准效果不佳，保持上一帧姿态\n";
        return init_trans;
    }

    return gicp.getFinalTransformation();
}

// 六自由度计算主函数
SixDofResult SixDofCalculator::calculateSixDof(const FusedPointCloud& c) {
    SixDofResult r;
    r.timestamp = c.timestamp;
    r.tx = r.ty = r.tz = r.rx = r.ry = r.rz = 0.00f;
    r.confidence = 0.00f;

    if (!fuse_pc_base_.cloud || fuse_pc_base_.cloud->empty()) {
        fuse_pc_base_ = c;
        r.confidence = 1.00f;
        return r;
    }

    //PointCloudT::Ptr src = preprocess(c.cloud);
    //PointCloudT::Ptr dst = preprocess(fuse_pc_base_.cloud);

    PointCloudT::Ptr src = c.cloud;
    PointCloudT::Ptr dst = fuse_pc_base_.cloud;

    std::cout << "src size:" << src->size()  << std::endl;


    if (src->empty() || dst->empty()) {
        return r;
    }

    Eigen::Matrix4f coarse_T = coarseRegistration(src, dst, regCfg_).cast<float>();
    Eigen::Matrix4f fine_T = fineRegistrationGICP(src, dst, coarse_T, regCfg_).cast<float>();

    // 旋转矩阵 → 欧拉角 XYZ 顺序
    Eigen::Matrix3f R_r = fine_T.block<3, 3>(0, 0).cast<float>();
    Eigen::Vector3f t_r = fine_T.block<3, 1>(0, 3).cast<float>();
#if 0
    Eigen::Matrix3f l_s = lr_sr_.block<3,3>(0,0).cast<float>();
    Eigen::Matrix3f ship_rot = l_s * R_r;

    float roll, pitch, yaw;
    getEulerAngles(ship_rot, roll, pitch, yaw);

    static Eigen::Vector3f t_r_last = t_r;
    Eigen::Vector3f delta_pos = t_r - t_r_last;
    t_r_last = t_r;

    Eigen::Vector3f ship_pos = l_s * delta_pos;
    r.rx = roll;
    r.ry = pitch;
    r.rz = yaw;

    r.tx = ship_pos[0] * 100;
    r.ty = ship_pos[1] * 100;
    r.tz = ship_pos[2] * 100;

    r.confidence = 0.95f;
#endif

    float roll, pitch, yaw;
    getEulerAngles(R_r, roll, pitch, yaw);
    r.rx = roll;
    r.ry = pitch;
    r.rz = yaw;
	
    r.tx = t_r.x();
    r.ty = t_r.y();
    r.tz = t_r.z();

    std::cout << "r.rx:" << r.rx << ",r.ry:" << r.ry << ",r.rz:" << r.rz << ",r.tx:" << r.tx << ",r.ty:" << r.ty << ",r.tz:" << r.tz  <<  std::endl;
    if (b_set_base_) {
    	return r;
    }
#if 0
    // ==============================================
    // 自动筛选停稳基准帧
    // ==============================================
    cout << "===== 开始检测停稳基准帧 =====" << endl;
    ShipPose pose;
    pose.tx = r.tx;
    pose.ty = r.ty;
    pose.rz = r.rz;
    pose.timestamp = r.timestamp;
    // 送入检测器
    dock_checker_.addPoseFrame(pose);

    // 尝试获取基准帧
    ShipPose base_frame;
    bool found = dock_checker_.getStableDockBaseFrame(base_frame);

    if (found) {
        cout << "成功找到停稳基准帧！" << endl;
        cout << "基准帧时间戳: " << base_frame.timestamp << endl;
        cout << "基准帧位置: (" << base_frame.tx << ", " << base_frame.ty << ")" << endl;
        cout << "基准帧艏向: " << base_frame.rz << " 度" << endl;
	b_set_base_ = true;
    } else {
        cout << "未达到稳定停靠" << endl;
    }

    LOG_INFO("print six result");
#endif

    return r;
}
