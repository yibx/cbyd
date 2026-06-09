#include "point_cloud_acquirer.h"
#include <chrono>
#include <iostream>
#include <string>
#include <IncludeFile.h>

#include "system_state_manager.h"
using SM = SystemStateManager;

using namespace std;
using namespace std::chrono;

PointCloudAcquirer::PointCloudAcquirer(
    LockFreeRingQueue<RawPointCloud>* queueA,
    LockFreeRingQueue<RawPointCloud>* queueB,
    int port_dev,
    int port_data
)
    : queueA_(queueA), queueB_(queueB), port_dev_(port_dev), port_data_(port_data)
{}

void PointCloudAcquirer::start() {
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
    GetLidarData* LidarDataA = new GetLidarData_LS();
    LidarDataA->setPortAndIP(2368, 2369, "192.168.1.102");
    LidarDataA->LidarStart();

    int try_get_cloud = 0;
    while (is_running_) {
        if (!LidarDataA->isFrameOK) {
            this_thread::sleep_for(milliseconds(100));
	    if (try_get_cloud > 10) {
            	SM::instance().reportError(
                	ModuleType::ACQUIRER,
                	ErrorLevel::STATUS_ERROR,
               		"雷达A点云接收超时"
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

        // 入队 A（无竞争）
        while (!queueA_->enqueue({ "A", ts, cloud }) && is_running_) {
            this_thread::sleep_for(milliseconds(2));
        }
	//std::cout << "ns:" << ts << ", size:" << cloud->size() << std::endl;
    }

    std::cout << "A exit" << std::endl;

    if (LidarDataA) {
    	delete LidarDataA;
    }
}

// ------------------------------
// 雷达 B 独立线程 → 写队列 B
// ------------------------------
void PointCloudAcquirer::acquireRadar2Loop() {
    GetLidarData* LidarDataB = new GetLidarData_LS();
    LidarDataB->setPortAndIP(2370, 2371, "192.168.1.102");
    LidarDataB->LidarStart();

    int try_get_cloud = 0;
    while (is_running_) {
        if (!LidarDataB->isFrameOK) {
            this_thread::sleep_for(milliseconds(100));
            if (try_get_cloud > 10) {
                SM::instance().reportError(
                        ModuleType::ACQUIRER,
                        ErrorLevel::STATUS_ERROR,
                        "雷达B点云接收超时"
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
        while (!queueB_->enqueue({ "B", ts, cloud }) && is_running_) {
            this_thread::sleep_for(milliseconds(2));
        }
    }

    std::cout << "B exit" << std::endl;

    if (LidarDataB) {
    	delete LidarDataB;
    }
}
