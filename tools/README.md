1、先确保脚本可执行

2、sudo crontab -e

3、在 crontab 文件的末尾添加以下行：
@reboot sleep 60 && /home/yourusername/reboot_in_10_minutes.sh

这行配置的含义是：

@reboot: 表示在系统重启时执行
sleep 60: 等待 60 秒（这给系统一些时间来完全启动）
&&: 表示前一个命令成功执行后，才执行后面的命令
/home/yourusername/reboot_in_10_minutes.sh: 你的脚本的完整路径

4、如果你想查看当前的 crontab 设置，可以使用：
sudo crontab -l

5、删除这个任务
sudo crontab -e


