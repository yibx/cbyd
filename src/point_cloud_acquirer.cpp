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
    int save_frame_count = 5; // 仅保存第一帧点云用于调试
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

	    if (save_frame_count > 0)
        {
            std::string pcd_name = "lidarA_single_frame_" + std::to_string(ts) + ".pcd";
            pcl::io::savePCDFileBinary(pcd_name, *cloud);
            std::cout << "[INFO] Save one frame to: " << pcd_name << std::endl;
            save_frame_count--;
        }

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
