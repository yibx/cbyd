#include "point_cloud_acquirer.h"
#include <chrono>
#include <iostream>
#include <string>
#include <IncludeFile.h>
#include <pcl/io/pcd_io.h>
#include "system_state_manager.h"
#include "Logger.h" 
#include "file_clean.h"

using SM = SystemStateManager;

using namespace std;
using namespace std::chrono;

#include "lidar_bg_diff.h"
#include <Eigen/Dense>
#include <pcl/common/transforms.h>

#define MAX_RETRY_LIDAR 100 // 雷达数据获取失败最大重试次数
#define SLEEP_NO_LIDAR_DATA 100 // 获取雷达数据失败时的休眠时间（毫秒）
#define SLEEP_FRAME_ERROR 5 // 点云帧异常时的休眠时间（毫秒）
#define ENTER_LIDAR_TIMEOUT 2 // 雷达数据进入队列的超时时间（毫秒）
#define DELAY_GET_LIDAR 300 // 雷达数据获取间隔时间（毫秒）

// 初始化背景差分处理器，加载背景点云
bool PointCloudAcquirer::initLidarBgProcessor(LidarBgDiff& proc, int lidar_sel) {
    proc.setDistanceThreshold(monitor_cfg_.dist_threshold);
    proc.setStableFrameCount(monitor_cfg_.stable_frame_count);
    proc.setStableDistThreshold(monitor_cfg_.stable_dist_threshold);
    proc.setMinValidShipPoints(monitor_cfg_.min_valid_ship_points);

    if (lidar_sel == 0) {
        if (!proc.loadBackground(monitor_cfg_.lidarA_bg_pcd_path))
        {
            return false;
        }
    } else {
        if (!proc.loadBackground(monitor_cfg_.lidarB_bg_pcd_path))
        {
            return false;
        }
    }
    return true;
}
// 处理单帧点云，返回船舶监测结果
ShipMonitorResult PointCloudAcquirer::processOneLidarFrame(LidarBgDiff& proc,
                                       PointCloudT::Ptr raw_cloud,
                                       PointCloudT::Ptr ship_out) {
    ShipMonitorResult res = proc.monitorShip(
        raw_cloud,
        ship_out,
        monitor_cfg_.cluster_eps,
        monitor_cfg_.cluster_min_points
    );

    switch(res.status)
    {
    case ShipMonitorStatus::SHIP_LEAVE:
        break;
    case ShipMonitorStatus::SHIP_MOVING:
        break;
    case ShipMonitorStatus::SHIP_STABLE:
        break;
    }
    return res;
}

PointCloudAcquirer::PointCloudAcquirer(
    LockFreeRingQueue<RawPointCloud>* queueA,
    LockFreeRingQueue<RawPointCloud>* queueB
)
    : queueA_(queueA), queueB_(queueB)
{}

void PointCloudAcquirer::start() {
    
    if (!loadLidarConfigs("../dev_config.yaml", lidarCfg_)){
        std::string err_msg = "雷达配置加载失败，请检查dev_config.yaml文件";
        Logger::instance().info(err_msg);
        return;
    }
    if (!loadMonitorConfig("../monitor_config.yaml", monitor_cfg_)) {
        std::string err_msg = "监测配置加载失败，请检查monitor_config.yaml文件";
        Logger::instance().info(err_msg);
        return;
    }

    if (is_running_) return;
    is_running_ = true;

    thread_radar1_ = std::thread(&PointCloudAcquirer::acquireRadar1Loop, this);
    thread_radar2_ = std::thread(&PointCloudAcquirer::acquireRadar2Loop, this);

    thread_camera1_ = std::thread(&PointCloudAcquirer::acquireCamera1Loop, this);
    thread_camera2_ = std::thread(&PointCloudAcquirer::acquireCamera2Loop, this);
}

void PointCloudAcquirer::stop() {
    is_running_ = false;

    if (thread_radar1_.joinable()) thread_radar1_.join();
    if (thread_radar2_.joinable()) thread_radar2_.join();

    if (thread_camera1_.joinable()) thread_camera1_.join();
    if (thread_camera2_.joinable()) thread_camera2_.join();
}

// 读取PCD文件，成功返回点云智能指针，失败返回空指针
PointCloudT::Ptr loadPcdFile(const std::string& pcd_path) {
    PointCloudT::Ptr cloud(new PointCloudT);

    // 调用PCL读取接口
    int ret = pcl::io::loadPCDFile<PointT>(pcd_path, *cloud);
    if (ret == -1)
    {
        std::cerr << "读取PCD失败，文件路径：" << pcd_path << std::endl;
        return nullptr;
    }

    std::cout << "PCD读取成功，点云数量：" << cloud->size() << std::endl;
    return cloud;
}

// ------------------------------
// 雷达 A 独立线程 → 写队列 A
// ------------------------------
void PointCloudAcquirer::acquireRadar1Loop() {
    std::string dev_info = "name:" + lidarCfg_.lidarA.name + ",ip:" + lidarCfg_.lidarA.lidar_ip + ",dev_port:" + std::to_string(lidarCfg_.lidarA.dev_port) + ",data_port:" + std::to_string(lidarCfg_.lidarA.data_port) + ",local_ip:" + lidarCfg_.lidarA.local_ip;
    Logger::instance().info(dev_info);
    GetLidarData* LidarDataA = new GetLidarData_LS();
    LidarDataA->setPortAndIP(lidarCfg_.lidarA.dev_port, lidarCfg_.lidarA.data_port, lidarCfg_.lidarA.local_ip);
    LidarDataA->LidarStart();

    int try_get_cloud = 0;

    // 线程内部
    if (lidarCfg_.ship_monitor) {
        if (!initLidarBgProcessor(bg_a_proc_, 0)) {
            Logger::instance().error("雷达A背景差分初始化失败，船舶监测关闭");
            lidarCfg_.ship_monitor = 0; 
        } 
    }
    PointCloudT::Ptr cloud = loadPcdFile("/home/hz/CBYD/bg_berth/L_A_Lidar_bg.pcd");

    while (is_running_) {
#if 0
        if (!LidarDataA->isFrameOK) {
            this_thread::sleep_for(milliseconds(SLEEP_NO_LIDAR_DATA));
	        if (try_get_cloud > MAX_RETRY_LIDAR) {
                std::string err_msg = lidarCfg_.lidarA.name + ",ip:" + lidarCfg_.lidarA.lidar_ip + "点云接收超时";
            	SM::instance().reportError(
                	ModuleType::ACQUIRER,
                	ErrorLevel::STATUS_ERROR,
               		err_msg
            		);
		        try_get_cloud = 0;
	        }
	        try_get_cloud++;
            continue;
        }
	    try_get_cloud = 0;

        shared_ptr<vector<MuchLidarData>> temp;
        string info;
        if (!LidarDataA->getLidarPerFrameDate(temp, info) || !temp || temp->empty()) {
            this_thread::sleep_for(milliseconds(SLEEP_FRAME_ERROR));
            continue;
        }

        uint64_t ts = temp->back().Mtimestamp_nsce;
        PointCloudT::Ptr cloud(new PointCloudT);
        cloud->reserve(100000);

        for (auto& p : *temp) {
            cloud->push_back(PointT(p.X, p.Y, p.Z));
        }


        PointCloudT::Ptr ship_out; 
        if (lidarCfg_.ship_monitor) {
            ship_out.reset(new PointCloudT); // 开启船舶监测才分配内存
            ship_out->reserve(cloud->size());
            // lidar_sel=0 代表雷达A，内部自动转换A的码头坐标
            ShipMonitorResult ship_status = processOneLidarFrame(bg_a_proc_, cloud, ship_out);
            // ship_status 可传递给上层业务/队列，这里仅打印
            if (ship_status.status == ShipMonitorStatus::SHIP_LEAVE) {
                std::string log_msg = "[A] Ship left, points: " + std::to_string(ship_status.ship_point_count);
                Logger::instance().info(log_msg);
                continue;
            } else if (ship_status.status == ShipMonitorStatus::SHIP_MOVING) {
                std::string log_msg = "[A] Ship moving, center_x: " + std::to_string(ship_status.pose.center_x) +
                                  ", center_y: " + std::to_string(ship_status.pose.center_y);
                Logger::instance().info(log_msg);
                continue;
            } else if (ship_status.status == ShipMonitorStatus::SHIP_STABLE) {
                std::string log_msg = "[A] Ship stable, center_x: " + std::to_string(ship_status.pose.center_x) +
                                  ", center_y: " + std::to_string(ship_status.pose.center_y);
                Logger::instance().info(log_msg);
            }
        }

        if (lidarCfg_.debug_save) {
            const std::string pcd_dir = "./pcd_debug";
            MakeDirLinux(pcd_dir);
            CleanOldPcdLinux(pcd_dir, lidarCfg_.save_min);

            std::string pcd_name = pcd_dir + "/lidarA_single_frame_" + std::to_string(ts) + ".pcd";
            pcl::io::savePCDFileBinary(pcd_name, *cloud);
            Logger::instance().info("[INFO] Save one frame to: " + pcd_name);

            if (lidarCfg_.ship_monitor && ship_out) {
                std::string pcd_name = pcd_dir + "/lidarA_split_frame_" + std::to_string(ts) + ".pcd";
                pcl::io::savePCDFileBinary(pcd_name, *ship_out);
                Logger::instance().info("[INFO] Save split frame to: " + pcd_name);
            }
        }

        if (lidarCfg_.ship_monitor && ship_out) {
            while (!queueA_->enqueue({ lidarCfg_.lidarA.lidar_ip, "A", ts, ship_out }) && is_running_) {
                this_thread::sleep_for(milliseconds(ENTER_LIDAR_TIMEOUT));
            }
        } else {
            while (!queueA_->enqueue({ lidarCfg_.lidarA.lidar_ip, "A", ts, cloud }) && is_running_) {
                this_thread::sleep_for(milliseconds(ENTER_LIDAR_TIMEOUT));
            }
        }
#endif
        
        while (!queueA_->enqueue({ lidarCfg_.lidarA.lidar_ip, "A", 19999999999, cloud }) && is_running_) {
            this_thread::sleep_for(milliseconds(ENTER_LIDAR_TIMEOUT));
        }

        this_thread::sleep_for(milliseconds(DELAY_GET_LIDAR));
    }

    Logger::instance().info(lidarCfg_.lidarA.name + " exit, ip:" + lidarCfg_.lidarA.lidar_ip);

    if (LidarDataA) {
    	delete LidarDataA;
    }
}

// ------------------------------
// 雷达 B 独立线程 → 写队列 B
// ------------------------------
void PointCloudAcquirer::acquireRadar2Loop() {
    std::stringstream ss;
    ss << "name:" << lidarCfg_.lidarB.name
       << ",ip:" << lidarCfg_.lidarB.lidar_ip
       << ",dev_port:" << lidarCfg_.lidarB.dev_port
       << ",data_port:" << lidarCfg_.lidarB.data_port
       << ",local_ip:" << lidarCfg_.lidarB.local_ip
       << ",monitor:" << lidarCfg_.ship_monitor
       << ",save:" << lidarCfg_.debug_save;
    std::string dev_info = ss.str();

    Logger::instance().info(dev_info);
    GetLidarData* LidarDataB = new GetLidarData_LS();
    LidarDataB->setPortAndIP(lidarCfg_.lidarB.dev_port, lidarCfg_.lidarB.data_port, lidarCfg_.lidarB.local_ip);
    LidarDataB->LidarStart();

    if (lidarCfg_.ship_monitor) {
        if (!initLidarBgProcessor(bg_b_proc_, 1)) {
            Logger::instance().error("雷达B背景差分初始化失败，船舶监测关闭");
            lidarCfg_.ship_monitor = 0; 
        } 
    }
    PointCloudT::Ptr cloud = loadPcdFile("/home/hz/CBYD/bg_berth/L_B_Lidar_bg.pcd");

    int try_get_cloud = 0;
    while (is_running_) {
#if 0
        if (!LidarDataB->isFrameOK) {
            this_thread::sleep_for(milliseconds(SLEEP_NO_LIDAR_DATA));
            if (try_get_cloud > 100) {
                std::string err_msg = lidarCfg_.lidarB.name + ",ip:" + lidarCfg_.lidarB.lidar_ip + "点云接收超时";
                SM::instance().reportError(
                        ModuleType::ACQUIRER,
                        ErrorLevel::STATUS_ERROR,
                        err_msg
                        );
                try_get_cloud = 0;
            }
            try_get_cloud++;
            continue;
        }
        try_get_cloud = 0;

        shared_ptr<vector<MuchLidarData>> temp;
        string info;
        if (!LidarDataB->getLidarPerFrameDate(temp, info) || !temp || temp->empty()) {
            this_thread::sleep_for(milliseconds(SLEEP_FRAME_ERROR));
            continue;
        }

        uint64_t ts = temp->back().Mtimestamp_nsce;
        PointCloudT::Ptr cloud(new PointCloudT);
        cloud->reserve(100000);

        for (auto& p : *temp) {
             cloud->push_back(PointT(p.X, p.Y, p.Z));
        }

        PointCloudT::Ptr ship_out; // 先空智能指针
        if (lidarCfg_.ship_monitor) {
            ship_out.reset(new PointCloudT); // 仅开启船舶监测才分配内存
            ship_out->reserve(cloud->size());
            ShipMonitorResult ship_status = processOneLidarFrame(bg_b_proc_, cloud, ship_out);
            if (ship_status.status == ShipMonitorStatus::SHIP_LEAVE) {
                Logger::instance().info("[B] Ship left, points: " + std::to_string(ship_status.ship_point_count));
                continue;
            } else if (ship_status.status == ShipMonitorStatus::SHIP_MOVING) {
                Logger::instance().info("[B] Ship moving, center_x: " + std::to_string(ship_status.pose.center_x) +
                                        ", center_y: " + std::to_string(ship_status.pose.center_y));
                continue;
            } else if (ship_status.status == ShipMonitorStatus::SHIP_STABLE) {
                Logger::instance().info("[B] Ship stable, center_x: " + std::to_string(ship_status.pose.center_x) +
                                        ", center_y: " + std::to_string(ship_status.pose.center_y));
            }
        }

        if (lidarCfg_.debug_save) {
            const std::string pcd_dir = "./pcd_debug";
            MakeDirLinux(pcd_dir);
            CleanOldPcdLinux(pcd_dir, lidarCfg_.save_min);

            std::string pcd_name = pcd_dir + "/lidarB_single_frame_" + std::to_string(ts) + ".pcd";
            pcl::io::savePCDFileBinary(pcd_name, *cloud);
            Logger::instance().info("[INFO] Save one frame to: " + pcd_name);

            if (lidarCfg_.ship_monitor && ship_out) {
                pcd_name = pcd_dir + "/lidarB_split_frame_" + std::to_string(ts) + ".pcd";
                pcl::io::savePCDFileBinary(pcd_name, *ship_out);
                Logger::instance().info("[INFO] Save split frame to: " + pcd_name);
            }
        }

        if (lidarCfg_.ship_monitor && ship_out) {
            while (!queueB_->enqueue({ lidarCfg_.lidarB.lidar_ip, "B", ts, ship_out }) && is_running_) {
                this_thread::sleep_for(milliseconds(ENTER_LIDAR_TIMEOUT));
            }
        } else {
            while (!queueB_->enqueue({ lidarCfg_.lidarB.lidar_ip, "B", ts, cloud }) && is_running_) {
                this_thread::sleep_for(milliseconds(ENTER_LIDAR_TIMEOUT));
            }
        }
#endif
        while (!queueB_->enqueue({ lidarCfg_.lidarB.lidar_ip, "B", 19999999999, cloud }) && is_running_) {
            this_thread::sleep_for(milliseconds(ENTER_LIDAR_TIMEOUT));
        }
        this_thread::sleep_for(milliseconds(DELAY_GET_LIDAR));
    }

    Logger::instance().info(lidarCfg_.lidarB.name + " exit, ip:" + lidarCfg_.lidarB.lidar_ip);

    if (LidarDataB) {
    	delete LidarDataB;
    }
}

void PointCloudAcquirer::acquireCamera1Loop() {

    while (is_running_) {
        // 接收左相机
        std::cout << "accept left camera info" << std::endl;
        this_thread::sleep_for(seconds(30));
    }
}

void PointCloudAcquirer::acquireCamera2Loop() {

    while (is_running_) {
        // 接受右相机
        std::cout << "accept right camera info" << std::endl;
        this_thread::sleep_for(seconds(30));
    }
}