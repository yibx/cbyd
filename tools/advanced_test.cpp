/**
 * g++ -o advanced_test advanced_test.cpp -pthread -O3 -march=native
 */
#include <iostream>
#include <thread>
#include <vector>
#include <chrono>
#include <cmath>
#include <pthread.h>

void set_cpu_affinity(int cpu_id) {
    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    CPU_SET(cpu_id, &cpuset);
    pthread_setaffinity_np(pthread_self(), sizeof(cpu_set_t), &cpuset);
}

void benchmark_task(int thread_id, long iterations) {
    set_cpu_affinity(thread_id);
    
    auto start = std::chrono::high_resolution_clock::now();
    volatile double result = 0.0;
    
    // Mix of CPU operations
    for (long i = 0; i < iterations; i++) {
        result += std::sin(i) * std::cos(i) + std::sqrt(i);
        if (i % 1000 == 0) {
            result *= 1.0000001;  // Prevent optimization
        }
    }
    
    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
    
    std::cout << "Thread " << thread_id << " on CPU " << sched_getcpu() 
              << " completed in " << duration.count() / 1000.0 << "ms\n";
}

int main() {
    const int num_threads = 8;
    const long iterations = 50000000;
    std::vector<std::thread> threads;
    
    std::cout << "Starting performance test on Jetson Orin NX...\n";
    std::cout << "Number of hardware threads: " << std::thread::hardware_concurrency() << "\n";
    
    auto total_start = std::chrono::high_resolution_clock::now();
    
    // Launch threads with different workloads
    for (int i = 0; i < num_threads; i++) {
        threads.emplace_back(benchmark_task, i, iterations * (i + 1));
    }
    
    // Monitor and join threads
    for (auto& t : threads) {
        t.join();
    }
    
    auto total_end = std::chrono::high_resolution_clock::now();
    auto total_duration = std::chrono::duration_cast<std::chrono::milliseconds>(total_end - total_start);
    
    std::cout << "\nTotal test duration: " << total_duration.count() << "ms\n";
    
    return 0;
}
