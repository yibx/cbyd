#ifndef CONFIG_LOADER_H
#define CONFIG_LOADER_H

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

#endif // CONFIG_LOADER_H