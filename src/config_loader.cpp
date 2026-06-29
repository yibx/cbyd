#include "config_loader.h"
#include <iostream>

bool loadLidarConfigs(const std::string& configPath, AllLidarConfigs& outConfigs)
{
    try
    {
        YAML::Node config = YAML::LoadFile(configPath);
        if (!config.IsDefined())
        {
            std::cerr << "YAML file is empty or invalid: " << configPath << std::endl;
            return false;
        }

        // 读取顶层开关 debug_save / ship_monitor
        outConfigs.debug_save  = config["debug_save"].as<int>(0);
        outConfigs.ship_monitor= config["ship_monitor"].as<int>(0);
        outConfigs.save_min= config["save_min"].as<int>(0);

        // 解析 lidarA
        if (config["lidarA"].IsDefined())
        {
            outConfigs.lidarA.name     = config["lidarA"]["name"].as<std::string>("Unknown");
            outConfigs.lidarA.lidar_ip   = config["lidarA"]["dev_ip"].as<std::string>("0.0.0.0");
            outConfigs.lidarA.local_ip = config["lidarA"]["local_ip"].as<std::string>("0.0.0.0");
            outConfigs.lidarA.dev_port = config["lidarA"]["dev_port"].as<int>(0);
            outConfigs.lidarA.data_port= config["lidarA"]["data_port"].as<int>(0);
        }
        else
        {
            std::cerr << "Warning: lidarA config not found in YAML!" << std::endl;
        }

        // 解析 lidarB
        if (config["lidarB"].IsDefined())
        {
            outConfigs.lidarB.name     = config["lidarB"]["name"].as<std::string>("Unknown");
            outConfigs.lidarB.lidar_ip   = config["lidarB"]["dev_ip"].as<std::string>("0.0.0.0");
            outConfigs.lidarB.local_ip = config["lidarB"]["local_ip"].as<std::string>("0.0.0.0");
            outConfigs.lidarB.dev_port = config["lidarB"]["dev_port"].as<int>(0);
            outConfigs.lidarB.data_port= config["lidarB"]["data_port"].as<int>(0);
        }
        else
        {
            std::cerr << "Warning: lidarB config not found in YAML!" << std::endl;
        }

        return true;
    }
    catch (const std::exception& e)
    {
        std::cerr << "Failed to load lidar config: " << e.what() << std::endl;
        return false;
    }
}

bool loadRegParam(const std::string& cfgPath, RegParam& outParam)
{
    try
    {
        YAML::Node root = YAML::LoadFile(cfgPath);
        if (!root.IsDefined())
        {
            std::cerr << "Config file empty or invalid: " << cfgPath << std::endl;
            return false;
        }

        // 加载 NDT 参数
        if (root["ndt"].IsDefined())
        {
            YAML::Node ndtNode = root["ndt"];
            if (ndtNode["voxel"].IsDefined())
            {
                outParam.ndt.voxel_x = ndtNode["voxel"]["x"].as<double>(outParam.ndt.voxel_x);
                outParam.ndt.voxel_y = ndtNode["voxel"]["y"].as<double>(outParam.ndt.voxel_y);
                outParam.ndt.voxel_z = ndtNode["voxel"]["z"].as<double>(outParam.ndt.voxel_z);
            }
            outParam.ndt.max_iter   = ndtNode["max_iter"].as<int>(outParam.ndt.max_iter);
            outParam.ndt.trans_eps  = ndtNode["trans_eps"].as<double>(outParam.ndt.trans_eps);
            outParam.ndt.step_size  = ndtNode["step_size"].as<double>(outParam.ndt.step_size);
            outParam.ndt.resolution = ndtNode["resolution"].as<double>(outParam.ndt.resolution);
            outParam.ndt.fit_thresh = ndtNode["fit_thresh"].as<double>(outParam.ndt.fit_thresh);
        }

        // 加载 GICP 参数
        if (root["gicp"].IsDefined())
        {
            YAML::Node gicpNode = root["gicp"];
            if (gicpNode["voxel"].IsDefined())
            {
                outParam.gicp.voxel_x = gicpNode["voxel"]["x"].as<double>(outParam.gicp.voxel_x);
                outParam.gicp.voxel_y = gicpNode["voxel"]["y"].as<double>(outParam.gicp.voxel_y);
                outParam.gicp.voxel_z = gicpNode["voxel"]["z"].as<double>(outParam.gicp.voxel_z);
            }
            outParam.gicp.use_reciprocal = gicpNode["use_reciprocal"].as<bool>(outParam.gicp.use_reciprocal);
            outParam.gicp.max_corr_dist  = gicpNode["max_corr_dist"].as<double>(outParam.gicp.max_corr_dist);
            outParam.gicp.max_iter       = gicpNode["max_iter"].as<int>(outParam.gicp.max_iter);
            outParam.gicp.trans_eps      = gicpNode["trans_eps"].as<double>(outParam.gicp.trans_eps);
            outParam.gicp.euclid_eps     = gicpNode["euclid_eps"].as<double>(outParam.gicp.euclid_eps);
            outParam.gicp.rot_eps        = gicpNode["rot_eps"].as<double>(outParam.gicp.rot_eps);
            outParam.gicp.rand_num       = gicpNode["rand_num"].as<int>(outParam.gicp.rand_num);
            outParam.gicp.fit_thresh     = gicpNode["fit_thresh"].as<double>(outParam.gicp.fit_thresh);
            outParam.gicp.score_radius   = gicpNode["score_radius"].as<double>(outParam.gicp.score_radius);
        }

        return true;
    }
    catch (const YAML::Exception& e)
    {
        std::cerr << "YAML parse error: " << e.what() << std::endl;
        return false;
    }
    catch (const std::exception& e)
    {
        std::cerr << "Load reg param exception: " << e.what() << std::endl;
        return false;
    }
}

bool loadMonitorConfig(const std::string& yaml_path, RadarGlobalConfig& outConfig)
{
    try
    {
        YAML::Node root = YAML::LoadFile(yaml_path);

        // 路径
        outConfig.lidarA_bg_pcd_path  = root["Abg_pcd_path"].as<std::string>();
        outConfig.lidarB_bg_pcd_path  = root["Bbg_pcd_path"].as<std::string>();

        // 基础算法阈值
        outConfig.dist_threshold = root["dist_threshold"].as<double>();
        outConfig.cluster_eps    = root["cluster_eps"].as<double>();
        outConfig.cluster_min_points = root["cluster_min_points"].as<int>();

        // 船舶停稳判定参数
        outConfig.stable_frame_count     = root["stable_frame_count"].as<int>();
        outConfig.stable_dist_threshold  = root["stable_dist_threshold"].as<double>();
        outConfig.min_valid_ship_points  = root["min_valid_ship_points"].as<int>();

        // 雷达A外参
        YAML::Node nodeA = root["lidar_a"];
        auto arrA = nodeA["trans"].as<std::vector<float>>();
        outConfig.lidarA.trans = Eigen::Vector3f(arrA[0], arrA[1], arrA[2]);
        outConfig.lidarA.rotate_z_deg = nodeA["rotate_z_deg"].as<float>();
        if(nodeA["rotate_y_deg"]) outConfig.lidarA.rotate_y_deg = nodeA["rotate_y_deg"].as<float>();
        if(nodeA["rotate_x_deg"]) outConfig.lidarA.rotate_x_deg = nodeA["rotate_x_deg"].as<float>();

        // 雷达B外参
        YAML::Node nodeB = root["lidar_b"];
        auto arrB = nodeB["trans"].as<std::vector<float>>();
        outConfig.lidarB.trans = Eigen::Vector3f(arrB[0], arrB[1], arrB[2]);
        outConfig.lidarB.rotate_z_deg = nodeB["rotate_z_deg"].as<float>();
        if(nodeB["rotate_y_deg"]) outConfig.lidarB.rotate_y_deg = nodeB["rotate_y_deg"].as<float>();
        if(nodeB["rotate_x_deg"]) outConfig.lidarB.rotate_x_deg = nodeB["rotate_x_deg"].as<float>();

        if (root["base_update_interval_min"].IsDefined())
        {
            outConfig.base_update_interval_min = root["base_update_interval_min"].as<double>();
        }

        if (root["mqtt"].IsDefined())
        {
            YAML::Node mqtt_node = root["mqtt"];
            outConfig.mqtt_cfg.host = mqtt_node["host"].as<std::string>();
            outConfig.mqtt_cfg.port = mqtt_node["port"].as<int>();
            outConfig.mqtt_cfg.username = mqtt_node["username"].as<std::string>();
            outConfig.mqtt_cfg.password = mqtt_node["password"].as<std::string>();
            outConfig.mqtt_cfg.default_topic = mqtt_node["default_topic"].as<std::string>();
            outConfig.mqtt_cfg.dev_status_topic = mqtt_node["dev_status_topic"].as<std::string>();
            outConfig.mqtt_cfg.qos = mqtt_node["qos"].as<int>();
            outConfig.mqtt_cfg.reconnect_interval = mqtt_node["reconnect_interval"].as<int>();
            outConfig.mqtt_cfg.berth_id = mqtt_node["berth_id"].as<std::string>();
        }
    }
    catch (std::exception& e)
    {
        std::cerr << "[ERROR] Load yaml config failed: " << e.what() << std::endl;
        return false;
    }
    return true;
}

// 根据雷达外参生成 lidar->dock 变换矩阵
Eigen::Affine3f buildLidarTransform(const RadarExtrinsic& ext) {
    Eigen::Affine3f trans = Eigen::Affine3f::Identity();
    trans.translation() = ext.trans;

    float rad_z = ext.rotate_z_deg * M_PI / 180.f;
    float rad_y = ext.rotate_y_deg * M_PI / 180.f;
    float rad_x = ext.rotate_x_deg * M_PI / 180.f;

    trans.rotate(Eigen::AngleAxisf(rad_z, Eigen::Vector3f::UnitZ()));
    trans.rotate(Eigen::AngleAxisf(rad_y, Eigen::Vector3f::UnitY()));
    trans.rotate(Eigen::AngleAxisf(rad_x, Eigen::Vector3f::UnitX()));
    return trans;
}

// 获取雷达A到dock的变换矩阵
Eigen::Affine3f getLidarATrans(RadarExtrinsic lidarA) {
    return buildLidarTransform(lidarA);
}

// 获取雷达B到dock的变换矩阵
Eigen::Affine3f getLidarBTrans(RadarExtrinsic lidarB) {
    return buildLidarTransform(lidarB);
}