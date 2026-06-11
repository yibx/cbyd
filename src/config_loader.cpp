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