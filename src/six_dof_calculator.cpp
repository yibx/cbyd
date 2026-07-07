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

#define MAX_RETRY_QUEUE 5
#define MAX_QUEUE_SIZE 30

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

    if (!loadLidarConfigs("../dev_config.yaml", lidarCfg_)){
        std::string err_msg = "雷达配置加载失败，请检查dev_config.yaml文件";
        LOG_ERROR("SIX_DOF_CALCULATOR", err_msg);
        return;
    }

    if (!loadRegParam("../reg_config.yaml", regCfg_)){
        std::string err_msg = "算法配置加载失败，请检查reg_config.yaml文件";
        LOG_ERROR("SIX_DOF_CALCULATOR", err_msg);
        return;
    }
    if (!loadMonitorConfig("../monitor_config.yaml", monitor_cfg_)) {
        std::string err_msg = "监测配置加载失败，请检查monitor_config.yaml文件";
        LOG_ERROR("SIX_DOF_CALCULATOR", err_msg);
        return;
    }
    if (lidarCfg_.debug_save) {
        LOG_DEBUG("SIX_DOF_CALCULATOR", "调试保存功能已启用");
        initFusionCsv();
    }

    // 获取雷达A到dock的变换矩阵
    T_A2dock_ = getLidarATrans(monitor_cfg_.lidarA);
    // 获取雷达B到dock的变换矩阵
    T_B2dock_ = getLidarBTrans(monitor_cfg_.lidarB);

    last_base_update_tp_ = std::chrono::steady_clock::now();
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
	    //auto start = std::chrono::high_resolution_clock::now();
        task();
	    //auto end = std::chrono::high_resolution_clock::now();
        //double elapsed_ms = std::chrono::duration<double, std::milli>(end - start).count();
	    //std::cout << "single: " << elapsed_ms << std::endl;
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
	        	if(tasks_.size() >= MAX_QUEUE_SIZE) {
	        		tasks_.pop();
	        	}
                tasks_.emplace([this, c = move(c)]() {
                    auto res = calculateSixDof(c);
		    	    int retry = 0;
                    while (!rq_sixdof_->enqueue(res) && retry < MAX_RETRY_QUEUE) {
                    	this_thread::sleep_for(chrono::milliseconds(3 * (1 << retry)));
		    		    retry++;
                    }
		    	    if (retry >= MAX_RETRY_QUEUE) {
                        std::string err_msg = "六自由度结果入队失败，可能导致监测数据丢失" + to_string(MAX_RETRY_QUEUE) + "次";
                        LOG_ERROR("SIX_DOF_CALCULATOR", err_msg);
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

// 极速预处理
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
    //std::cout << "NDT 分数: " << score << std::endl;

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

    //std::cout << "GICP 收敛: " << converged << " 分数: " << fitness << std::endl;

    // 配准不佳，保留上一帧姿态
    if (!converged || fitness > gicp_param.fit_thresh) {
        std::string err_msg = "位姿解算失败，保持上一帧结果";
        LOG_DEBUG("SIX_DOF_CALCULATOR", err_msg);
        return init_trans;
    }

    return gicp.getFinalTransformation();
}


void SixDofCalculator::initFusionCsv() {
    // 按天命名，和日志保持一致
    std::time_t ts = std::time(nullptr);
    std::tm tm_buf{};
    localtime_r(&ts, &tm_buf);
    std::ostringstream oss;
    oss << "six_data_" << std::put_time(&tm_buf, "%Y-%m-%d") << ".csv";
    std::string csv_path = oss.str();

    fusion_csv_.open(csv_path, std::ios::out | std::ios::app);
    if (!fusion_csv_.is_open())
    {
        std::string err_msg = "无法打开船舶运动数据CSV文件：" + csv_path;
        LOG_ERROR("SIX_DOF_CALCULATOR", err_msg);
        return;
    }

    // 文件为空时写入表头
    if (fusion_csv_.tellp() == 0)
    {
        fusion_csv_ << "local_time,timestamp,lidar_ip,lidar_id,rx(rad),ry(rad),rz(rad),tx(cm),ty(cm),tz(cm),confidence,ship_length\n";
    }
    fusion_csv_.flush();
}

// CSV字段包裹双引号，防止内部逗号干扰
std::string SixDofCalculator::csvWrap(const std::string& val) {
    std::string res = val;
    // 双引号替换成两个双引号csv标准转义
    size_t pos = 0;
    while ((pos = res.find('"', pos)) != std::string::npos)
    {
        res.insert(pos, "\"");
        pos += 2;
    }
    return "\"" + res + "\"";
}


// 六自由度计算主函数
SixDofResult SixDofCalculator::calculateSixDof(const FusedPointCloud& c) {

    SixDofResult r;
    r.timestamp = c.timestamp;
    r.tx = r.ty = r.tz = r.rx = r.ry = r.rz = 0.00f;
    r.confidence = 0.00f;

    // 计算间隔阈值 分钟转毫秒
    const double interval_min = monitor_cfg_.base_update_interval_min;
    const auto interval_dur = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
        std::chrono::minutes((long long)interval_min)
    );
    auto now_tp = std::chrono::steady_clock::now();

    std::lock_guard<std::mutex> lock(base_mtx_);
    bool need_update_base = false;

    // 无基准帧，强制更新
    if (!fuse_pc_base_.cloud || fuse_pc_base_.cloud->empty())
    {
        need_update_base = true;
    }
    else
    {
        // 判断是否超过设定分钟
        auto delta = now_tp - last_base_update_tp_;
        if (delta >= interval_dur)
        {
            need_update_base = true;
        }
    }

    // 需要更新基准帧
    if (need_update_base)
    {
        fuse_pc_base_ = c;
        last_base_update_tp_ = now_tp;
        r.confidence = 1.00f;
        std::string log_msg = "更新基准帧，间隔=" + to_string(interval_min) + "秒钟";
        LOG_INFO("SIX_DOF_CALCULATOR", log_msg);

        if(!base_extremum_calc_) {
            PointCloudT::Ptr base_cloud = fuse_pc_base_.cloud;
            if (base_cloud && !base_cloud->empty())
            {
                // 基准帧位姿：此时船舶相对基准无运动，T=单位矩阵
                Eigen::Matrix3f R_base = Eigen::Matrix3f::Identity();
                Eigen::Vector3f t_base(0,0,0);
                Eigen::Matrix3f R_T = R_base.transpose();

                float minXb = 1e9f;
                float maxXb = -1e9f;
                for (const auto& pt : *base_cloud)
                {
                    Eigen::Vector3f pg(pt.x, pt.y, pt.z);
                    // 全局点转到船体局部坐标系
                    Eigen::Vector3f pb = R_T * (pg - t_base);
                    if (pb.x() < minXb) minXb = pb.x();
                    if (pb.x() > maxXb) maxXb = pb.x();
                }
                // 缓存极值
                base_min_x_body_ = minXb;
                base_max_x_body_ = maxXb;
                base_extremum_calc_ = true;
                log_msg = "基准船体X范围 min:" + std::to_string(minXb) + ", max:" + std::to_string(maxXb);
                LOG_INFO("SIX_DOF_CALCULATOR", log_msg);
            }
        }
    }

    //PointCloudT::Ptr src = preprocess(c.cloud);
    //PointCloudT::Ptr dst = preprocess(fuse_pc_base_.cloud);

    // 不做任何预处理，原始点云
    PointCloudT::Ptr src = c.cloud;
    PointCloudT::Ptr dst = fuse_pc_base_.cloud;

    //std::cout << "src size:" << src->size()  << std::endl;

    if (src->empty() || dst->empty()) {
        return r;
    }

    // coarse + fine GICP配准，输出【当前雷达局部坐标系】变换矩阵
    Eigen::Matrix4f coarse_T = coarseRegistration(src, dst, regCfg_).cast<float>();
    Eigen::Matrix4f fine_T_local = fineRegistrationGICP(src, dst, coarse_T, regCfg_).cast<float>();

    // 局部变换仿射容器：当前雷达下船舶相对运动变换
    Eigen::Affine3f T_local;
    T_local.matrix() = fine_T_local;

    Eigen::Affine3f T_ship_dock;
    Eigen::Affine3f T_A2dock_inv = T_A2dock_.inverse();
    Eigen::Affine3f T_B2dock_inv = T_B2dock_.inverse();

    if (c.lidar_id == "B") {
        // B雷达：直接相似变换，无需中转A
        T_ship_dock = T_B2dock_ * T_local * T_B2dock_inv;
    } else if (c.lidar_id == "A") {
        // A雷达标准相似变换
        T_ship_dock = T_A2dock_ * T_local * T_A2dock_inv;
    } else {
        std::string err_msg = "未知雷达ID: " + c.lidar_id;
        LOG_ERROR("SIX_DOF_CALCULATOR", err_msg);
        return r;
    }

    // 提取码头坐标系下旋转、平移
    Eigen::Matrix3f R_ship = T_ship_dock.rotation().cast<float>();
    Eigen::Vector3f t_ship = T_ship_dock.translation().cast<float>();

    // 分解XYZ欧拉角（码头全局坐标系船舶姿态）
    float roll, pitch, yaw;
    getEulerAngles(R_ship, roll, pitch, yaw);

    // 填充结果：tx/ty/tz 码头坐标系厘米，rx/ry/rz 欧拉角（rad），confidence 简单置信度
    r.lidar_ip = c.lidar_ip;
    r.lidar_id = c.lidar_id;
    r.rx = roll;
    r.ry = pitch;
    r.rz = yaw;
    r.tx = t_ship.x() * 100; // m 转 cm
    r.ty = t_ship.y() * 100;
    r.tz = t_ship.z() * 100;
    r.confidence = 0.95f;
    r.ship_length = base_max_x_body_ - base_min_x_body_;

    if (lidarCfg_.debug_save) {
        std::lock_guard<std::mutex> lock(fusion_csv_mtx_);
        if (fusion_csv_.is_open())
        {
            // 获取本地系统时间
            auto now = std::chrono::system_clock::now();
            std::time_t t_now = std::chrono::system_clock::to_time_t(now);
            std::tm tm_now{};
            localtime_r(&t_now, &tm_now);
            std::ostringstream ts_ss;
            ts_ss << std::put_time(&tm_now, "%Y-%m-%d %H:%M:%S");
            std::string local_time = ts_ss.str();

            // 逐字段输出一行csv
            fusion_csv_
                << csvWrap(local_time) << ","
                << csvWrap(local_time) << ","
                << csvWrap(r.lidar_ip) << ","
                << csvWrap(r.lidar_id) << ","
                << r.rx << ","
                << r.ry << ","
                << r.rz << ","
                << r.tx << ","
                << r.ty << ","
                << r.tz << ","
                << r.confidence << ","
                << r.ship_length << "\n";

            fusion_csv_.flush(); // 强制落盘，防止丢失
        }
    }

              
    return r;
}
