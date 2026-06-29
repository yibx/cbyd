#include "file_clean.h"

#include <string>
#include <cstdio>
#include <cstring>
#include <dirent.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <errno.h>
#include "Logger.h"

void CleanOldPcdLinux(const std::string& dir, int keep_sec)
{
    DIR* dp = opendir(dir.c_str());
    if (nullptr == dp)
    {
        Logger::instance().warn("[CLEAN] open dir failed: " + dir + ", err:" + std::string(strerror(errno)));
        return;
    }

    // 获取当前系统时间
    struct timeval now_tv;
    gettimeofday(&now_tv, nullptr);
    time_t now_sec = now_tv.tv_sec;

    struct dirent* entry;
    while ((entry = readdir(dp)) != nullptr)
    {
        // 跳过 . 和 ..
        if (strcmp(entry->d_name, ".") == 0 || strcmp(entry->d_name, "..") == 0)
            continue;

        std::string full_path = dir + "/" + entry->d_name;

        // 只处理 .pcd 后缀文件
        std::string fname(entry->d_name);
        if (fname.size() < 4 || fname.substr(fname.size() - 4) != ".pcd")
            continue;

        // 获取文件状态
        struct stat st;
        if (stat(full_path.c_str(), &st) != 0)
        {
            Logger::instance().warn("[CLEAN] stat file fail: " + full_path);
            continue;
        }
        // 跳过文件夹，只处理普通文件
        if (!S_ISREG(st.st_mode))
            continue;

        // 文件最后修改时间秒数
        time_t file_mtime = st.st_mtime;
        // 判断是否超时
        if (now_sec - file_mtime > keep_sec)
        {
            if (remove(full_path.c_str()) == 0)
            {
                Logger::instance().info("[CLEAN] remove expired pcd: " + full_path);
            }
            else
            {
                Logger::instance().warn("[CLEAN] delete fail: " + full_path + ", err:" + std::string(strerror(errno)));
            }
        }
    }

    closedir(dp);
}

bool MakeDirLinux(const std::string& path)
{
    if (mkdir(path.c_str(), 0755) == 0)
        return true;
    // 目录已存在不算错误
    if (errno == EEXIST)
        return true;
    Logger::instance().warn("[DIR] create fail: " + path);
    return false;
}