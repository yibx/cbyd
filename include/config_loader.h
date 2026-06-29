#ifndef CONFIG_LOADER_H
#define CONFIG_LOADER_H

#include <Eigen/Dense>
#include <pcl/common/transforms.h>
#include <yaml-cpp/yaml.h>
#include <iostream>
#include <cmath>
#include <string>
#include <yaml-cpp/yaml.h>

// ====================== 雷达设备配置 ======================
struct LidarConfig
{
    std::string name;
    std::string lidar_ip;
    std::string local_ip;
    int dev_port;
    int data_port;
};

struct AllLidarConfigs
{
    LidarConfig lidarA;
    LidarConfig lidarB;
    // 顶层开关
    int debug_save = 0;
    int ship_monitor = 0;
    int save_min = 0;
};

/**
 * @brief 加载雷达设备 YAML 配置
 * @param configPath 配置文件路径
 * @param outConfigs 输出配置结构体
 * @return 成功返回 true，失败返回 false
 */
bool loadLidarConfigs(const std::string& configPath, AllLidarConfigs& outConfigs);

// ====================== 点云配准参数配置 ======================
// NDT 粗配准参数
struct NdtParam
{
    double voxel_x    = 0.1;
    double voxel_y    = 0.1;
    double voxel_z    = 0.1;
    int    max_iter   = 30;
    double trans_eps  = 1e-6;
    double step_size  = 0.1;
    double resolution = 0.5;
    double fit_thresh = 1.5;
};

// GICP 精配准参数
struct GicpParam
{
    double voxel_x      = 0.05;
    double voxel_y      = 0.05;
    double voxel_z      = 0.05;
    bool   use_reciprocal = true;
    double max_corr_dist  = 0.3;
    int    max_iter       = 150;
    double trans_eps      = 1e-6;
    double euclid_eps     = 1e-6;
    double rot_eps        = 1e-3;
    int    rand_num       = 15;
    double fit_thresh     = 1.0;
    double score_radius   = 0.5;
};

// 整合所有配准参数
struct RegParam
{
    NdtParam  ndt;
    GicpParam gicp;
};

/**
 * @brief 加载 NDT/GICP 点云配准参数
 * @param cfgPath 配置文件路径
 * @param outParam 输出配准参数结构体
 * @return 成功返回 true，失败返回 false
 */
bool loadRegParam(const std::string& cfgPath, RegParam& outParam);


// MQTT客户端配置
struct MqttConfigParam
{
    std::string berth_id;
    std::string host;
    int port;
    std::string username;
    std::string password;
    std::string default_topic;
    std::string dev_status_topic;
    int qos;
    int reconnect_interval;
};

// 单台雷达6自由度外参
struct RadarExtrinsic
{
    Eigen::Vector3f trans;
    float rotate_z_deg;
    float rotate_y_deg = 0.f;
    float rotate_x_deg = 0.f;
};

// 全局算法+路径配置
struct RadarGlobalConfig
{
    // 文件路径
    std::string lidarA_bg_pcd_path;
    std::string lidarB_bg_pcd_path;

    // 差分、聚类参数
    double dist_threshold;
    double cluster_eps;
    int cluster_min_points;

    // 船舶稳定判定参数
    int stable_frame_count;
    double stable_dist_threshold;
    int min_valid_ship_points;

    // 两台雷达外参
    RadarExtrinsic lidarA;
    RadarExtrinsic lidarB;

    // 基准帧自动更新间隔，单位：秒，默认300秒=5分钟
    double base_update_interval_min = 300.0;

    MqttConfigParam mqtt_cfg;
};

bool loadMonitorConfig(const std::string& yaml_path, RadarGlobalConfig& outConfig);


Eigen::Affine3f buildLidarTransform(const RadarExtrinsic& ext);

Eigen::Affine3f getLidarATrans(RadarExtrinsic lidarA);

Eigen::Affine3f getLidarBTrans(RadarExtrinsic lidarB);

#endif // CONFIG_LOADER_H