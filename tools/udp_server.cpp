
#include <iostream>
#include <cstring>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <thread>
#include <vector>

class UDPServer {
private:
    int sockfd;
    int port;
    
public:
    UDPServer(int port) : port(port), sockfd(-1) {}
    
    bool initialize() {
        // 创建UDP套接字
        sockfd = socket(AF_INET, SOCK_DGRAM, 0);
        if (sockfd < 0) {
            std::cerr << "创建套接字失败 (端口 " << port << ")" << std::endl;
            return false;
        }
        
        // 设置套接字选项，允许地址重用
        int opt = 1;
        if (setsockopt(sockfd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt)) < 0) {
            std::cerr << "设置套接字选项失败" << std::endl;
            close(sockfd);
            return false;
        }
        
        // 绑定地址信息
        struct sockaddr_in server_addr{};
        server_addr.sin_family = AF_INET;
        server_addr.sin_port = htons(port);
        server_addr.sin_addr.s_addr = INADDR_ANY; // 监听所有网卡
        
        if (bind(sockfd, (struct sockaddr*)&server_addr, sizeof(server_addr)) < 0) {
            std::cerr << "绑定端口 " << port << " 失败" << std::endl;
            close(sockfd);
            return false;
        }
        
        std::cout << "UDP服务端已启动，监听端口: " << port << std::endl;
        return true;
    }
    
    void startReceiving() {
        char buffer[1024];
        struct sockaddr_in client_addr{};
        socklen_t addr_len = sizeof(client_addr);
        
        while (true) {
            // 接收数据
            ssize_t bytes_received = recvfrom(sockfd, buffer, sizeof(buffer) - 1, 0,
                        (struct sockaddr*)&client_addr, &addr_len);
            
            if (bytes_received < 0) {
                std::cerr << "接收数据失败" << std::endl;
                continue;
            }
            
            // 添加字符串终止符
            buffer[bytes_received] = '\0';
            
            // 获取客户端IP地址
            char client_ip[INET_ADDRSTRLEN];
            inet_ntop(AF_INET, &client_addr.sin_addr, client_ip, sizeof(client_ip));
            
            std::cout << "端口 " << port << " 收到来自 " << client_ip << ":" << ntohs(client_addr.sin_port)
                      << " 的数据: " << buffer << " (长度: " << bytes_received << " 字节)" << std::endl;
            
            // 可选：发送响应
            std::string response = "已收到 " + std::to_string(bytes_received) + " 字节数据";
            sendto(sockfd, response.c_str(), response.length(), 0,
                   (struct sockaddr*)&client_addr, addr_len);
        }
    }
    
    ~UDPServer() {
        if (sockfd >= 0) {
            close(sockfd);
        }
    }
};

// 处理指定端口的接收线程函数
void handlePort(int port) {
    UDPServer server(port);
    if (server.initialize()) {
        server.startReceiving();
    }
}

int main() {
    std::cout << "启动UDP双端口数据接收服务..." << std::endl;
    
    // 创建线程处理2368端口
    std::thread port2368_thread(handlePort, 2368);
    
    // 创建线程处理2369端口  
    std::thread port2369_thread(handlePort, 2369);
    
    // 等待线程完成
    port2368_thread.join();
    port2369_thread.join();
    
    return 0;
}
// 验证接收雷达数据是否正常
// g++ -std=c++11 -pthread udp_server.cpp -o udp_server
