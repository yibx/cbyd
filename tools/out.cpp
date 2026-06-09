#include <signal.h>
#include <iostream>
#include <termios.h>
#include <unistd.h>

// 设置终端为非阻塞模式
void setNonBlocking() {
    struct termios ttystate;
    tcgetattr(STDIN_FILENO, &ttystate);
    ttystate.c_lflag &= ~(ICANON | ECHO);
    ttystate.c_cc[VMIN] = 0;
    tcsetattr(STDIN_FILENO, TCSANOW, &ttystate);
}

// 检测键盘输入
int kbhit() {
    char ch;
    int nread = read(STDIN_FILENO, &ch, 1);
    if (nread == 1) {
        return ch;
    }
    return 0;
}

void signalHandler(int signum) {
    exit(signum);
}

int main() {
    signal(SIGINT, signalHandler);
    setNonBlocking();  // 设置非阻塞模式
    
    while (true) {
        int ch = kbhit();
        if (ch == 'q') {
            system("killall smads");
            exit(0);
        }
        usleep(10000);  // 短暂休眠，减少CPU使用
    }
    return 0;
}
