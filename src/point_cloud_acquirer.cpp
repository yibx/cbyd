#include "point_cloud_acquirer.h"
#include <chrono>
#include <iostream>
#include <string>
#include <IncludeFile.h>
#include <pcl/io/pcd_io.h>
#include "system_state_manager.h"

using SM = SystemStateManager;

using namespace std;
using namespace std::chrono;

//#define PCD_SAVE_DEBUG 1
#define SHIP_MONITOR 2

#include "lidar_bg_diff.h"
#include <Eigen/Dense>
#include <pcl/common/transforms.h>

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

// 单帧完整处理函数，配置通过参数传入
ShipMonitorResult PointCloudAcquirer::processOneLidarFrame(LidarBgDiff& proc,
                                       PointCloudT::Ptr raw_cloud,
                                       PointCloudT::Ptr ship_out,
                                       int lidar_sel)
{
    // 1. 原始雷达点云转换至码头坐标系
    PointCloudT::Ptr dock_cloud(new PointCloudT);
#if 0
    Eigen::Affine3f trans;
    if (lidar_sel == 0)
        trans = getLidarATrans(monitor_cfg_.lidarA);
    else
        trans = getLidarBTrans(monitor_cfg_.lidarB);
    pcl::transformPointCloud(*raw_cloud, *dock_cloud, trans);
#endif
    // 2. 执行背景差分、聚类、船舶状态判定，参数取自传入的cfg
    ShipMonitorResult res = proc.monitorShip(
        dock_cloud,
        ship_out,
        monitor_cfg_.cluster_eps,
        monitor_cfg_.cluster_min_points
    );

    // 调试打印状态
    switch(res.status)
    {
    case ShipMonitorStatus::SHIP_LEAVE:
        std::cout << "[SHIP] 船舶已离开，点数:" << res.ship_point_count << std::endl;
        break;
    case ShipMonitorStatus::SHIP_MOVING:
        std::cout << "[SHIP] 船舶移动中，中心X:" << res.pose.center_x
                  << " Y:" << res.pose.center_y << std::endl;
        break;
    case ShipMonitorStatus::SHIP_STABLE:
        std::cout << "[SHIP] 船舶已停稳！中心X:" << res.pose.center_x
                  << " Y:" << res.pose.center_y << std::endl;
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
        return;
    }
    if (!loadMonitorConfig("../monitor_config.yaml", monitor_cfg_)) {
        std::cerr << "配置文件加载失败，程序退出" << std::endl;
        return;
    }
    if (is_running_) return;
    is_running_ = true;

    thread_radar1_ = std::thread(&PointCloudAcquirer::acquireRadar1Loop, this);
    thread_radar2_ = std::thread(&PointCloudAcquirer::acquireRadar2Loop, this);
}

void PointCloudAcquirer::stop() {
    is_running_ = false;

    if (thread_radar1_.joinable()) thread_radar1_.join();
    if (thread_radar2_.joinable()) thread_radar2_.join();
}

// ------------------------------
// 雷达 A 独立线程 → 写队列 A
// ------------------------------
void PointCloudAcquirer::acquireRadar1Loop() {
    
    std::cout << "name:" << lidarCfg_.lidarA.name << ",ip:" << lidarCfg_.lidarA.lidar_ip << ",dev_port:" << lidarCfg_.lidarA.dev_port << ",lidarCfg_.data_port:" << lidarCfg_.lidarA.data_port << ",lidarCfg_.server_ip:" << lidarCfg_.lidarA.local_ip  << std::endl;
    GetLidarData* LidarDataA = new GetLidarData_LS();
    LidarDataA->setPortAndIP(lidarCfg_.lidarA.dev_port, lidarCfg_.lidarA.data_port, lidarCfg_.lidarA.local_ip);
    LidarDataA->LidarStart();

    int try_get_cloud = 0;
#ifdef SHIP_MONITOR
    LidarBgDiff bg_proc;
    if (!initLidarBgProcessor(bg_proc, 0)) {
        std::cerr << "[FATAL] Initialize background processor failed." << std::endl;
        return;
    }
#endif
    while (is_running_) {
        if (!LidarDataA->isFrameOK) {
            this_thread::sleep_for(milliseconds(100));
	        if (try_get_cloud > 10) {
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
            this_thread::sleep_for(milliseconds(5));
            continue;
        }

        uint64_t ts = temp->back().Mtimestamp_nsce;
        PointCloudT::Ptr cloud(new PointCloudT);
        cloud->reserve(100000);

        for (auto& p : *temp) {
            cloud->push_back(PointT(p.X, p.Y, p.Z));
        }

	    std::cout << "A ts:" << ts  << std::endl;

#ifdef PCD_SAVE_DEBUG
        std::string pcd_name = "lidarA_single_frame_" + std::to_string(ts) + ".pcd";
        pcl::io::savePCDFileBinary(pcd_name, *cloud);
        std::cout << "[INFO] Save one frame to: " << pcd_name << std::endl;
#endif

#ifdef SHIP_MONITOR
        PointCloudT::Ptr ship_out(new PointCloudT);
        // lidar_sel=0 代表雷达A，内部自动转换A的码头坐标
        ShipMonitorResult ship_status = processOneLidarFrame(bg_proc, cloud, ship_out, 0);
        // ship_status 可传递给上层业务/队列，这里仅打印
        if (ship_status.status == ShipMonitorStatus::SHIP_LEAVE) {
            std::cout << "[A] 船舶已离开，点数:" << ship_status.ship_point_count << std::endl;
            continue;
        } else if (ship_status.status == ShipMonitorStatus::SHIP_MOVING) {
            std::cout << "[A] 船舶移动中，中心X:" << ship_status.pose.center_x
                      << " Y:" << ship_status.pose.center_y << std::endl;
            continue;
        } else if (ship_status.status == ShipMonitorStatus::SHIP_STABLE) {
            std::cout << "[A] 船舶已停稳！中心X:" << ship_status.pose.center_x
                      << " Y:" << ship_status.pose.center_y << std::endl;
        }
#endif

#ifdef PCD_SAVE_DEBUG
        pcd_name = "lidarA_split_frame_" + std::to_string(ts) + ".pcd";
        pcl::io::savePCDFileBinary(pcd_name, *ship_out);
        std::cout << "[INFO] Save split frame to: " << pcd_name << std::endl;
#endif

        // 入队 A（无竞争）
        while (!queueA_->enqueue({ lidarCfg_.lidarA.lidar_ip, ts, cloud }) && is_running_) {
            this_thread::sleep_for(milliseconds(2));
        }
	    //std::cout << "ns:" << ts << ", size:" << cloud->size() << std::endl;
    }

    std::cout << lidarCfg_.lidarA.lidar_ip << " exit"  << std::endl;

    if (LidarDataA) {
    	delete LidarDataA;
    }
}

// ------------------------------
// 雷达 B 独立线程 → 写队列 B
// ------------------------------
void PointCloudAcquirer::acquireRadar2Loop() {

    GetLidarData* LidarDataB = new GetLidarData_LS();
    LidarDataB->setPortAndIP(lidarCfg_.lidarB.dev_port, lidarCfg_.lidarB.data_port, lidarCfg_.lidarB.local_ip);
    LidarDataB->LidarStart();

#ifdef SHIP_MONITOR
    LidarBgDiff bg_proc;
    if (!initLidarBgProcessor(bg_proc, 1)) {
        std::cerr << "[FATAL] Initialize background processor failed." << std::endl;
        return;
    }
#endif

    int try_get_cloud = 0;
    while (is_running_) {
        if (!LidarDataB->isFrameOK) {
            this_thread::sleep_for(milliseconds(100));
            if (try_get_cloud > 10) {
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
            this_thread::sleep_for(milliseconds(5));
            continue;
        }

        uint64_t ts = temp->back().Mtimestamp_nsce;
        PointCloudT::Ptr cloud(new PointCloudT);
        cloud->reserve(100000);

        for (auto& p : *temp) {
             cloud->push_back(PointT(p.X, p.Y, p.Z));
        }

#ifdef SHIP_MONITOR
        PointCloudT::Ptr ship_out(new PointCloudT);
        ShipMonitorResult ship_status = processOneLidarFrame(bg_proc, cloud, ship_out, 1);
        if (ship_status.status == ShipMonitorStatus::SHIP_LEAVE) {
            std::cout << "[B] 船舶已离开，点数:" << ship_status.ship_point_count << std::endl;
            continue;
        } else if (ship_status.status == ShipMonitorStatus::SHIP_MOVING) {
            std::cout << "[B] 船舶移动中，中心X:" << ship_status.pose.center_x
                      << " Y:" << ship_status.pose.center_y << std::endl;
            continue;
        } else if (ship_status.status == ShipMonitorStatus::SHIP_STABLE) {
            std::cout << "[B] 船舶已停稳！中心X:" << ship_status.pose.center_x
                      << " Y:" << ship_status.pose.center_y << std::endl;
        }
#endif

#ifdef PCD_SAVE_DEBUG
        std::string pcd_name = "lidarB_single_frame_" + std::to_string(ts) + ".pcd";
        pcl::io::savePCDFileBinary(pcd_name, *cloud);
        std::cout << "[INFO] Save one frame to: " << pcd_name << std::endl;

        pcd_name = "lidarB_split_frame_" + std::to_string(ts) + ".pcd";
        pcl::io::savePCDFileBinary(pcd_name, *ship_out);
        std::cout << "[INFO] Save split frame to: " << pcd_name << std::endl;
#endif

        // 入队 B（无竞争）
        while (!queueB_->enqueue({ lidarCfg_.lidarB.lidar_ip, ts, cloud }) && is_running_) {
            this_thread::sleep_for(milliseconds(2));
        }
    }

    std::cout << lidarCfg_.lidarB.lidar_ip << " exit"  << std::endl;

    if (LidarDataB) {
    	delete LidarDataB;
    }
}
