#ifndef FILE_CLEAN_H
#define FILE_CLEAN_H

#include <string>

// 清理目录下超时pcd文件，Linux C++11专用
// dir: 存放pcd的目录
// keep_sec: 保留时长，单位秒，例如5分钟=300秒
void CleanOldPcdLinux(const std::string& dir, int keep_sec);

// 创建目录，不存在则创建
bool MakeDirLinux(const std::string& path);

#endif // FILE_CLEAN_H