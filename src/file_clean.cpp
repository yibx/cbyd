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
        LOG_WARN("CLEAN", "open dir failed: {}, err: {}", dir, strerror(errno));
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
            LOG_WARN("CLEAN", "[CLEAN] stat file fail: {}, err: {}", full_path, strerror(errno));
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
                LOG_INFO("CLEAN", "[CLEAN] remove expired pcd: {}", full_path);
            }
            else
            {
                LOG_WARN("CLEAN", "[CLEAN] delete fail: {}, err: {}", dir, strerror(errno));
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
    LOG_WARN("DIR", "create fail: {}, err: {}", path, strerror(errno));
    return false;
}