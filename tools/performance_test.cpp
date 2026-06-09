/**
 * g++ -o perf_test performance_test.cpp -pthread -O3
 */

#include <iostream>
#include <thread>
#include <vector>
#include <chrono>
#include <cmath>

void cpu_intensive_task(int thread_id, long iterations) {
    auto start = std::chrono::high_resolution_clock::now();
    double result = 0.0;
    
    for (long i = 0; i < iterations; i++) {
        result += std::sin(i) * std::cos(i);
    }
    
    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    
    std::cout << "Thread " << thread_id << " completed in " << duration.count() << "ms\n";
}

int main() {
    const int num_threads = 8;  // Orin NX has 8 cores
    const long iterations = 100000000;  // Adjust based on your needs
    std::vector<std::thread> threads;
    
    auto total_start = std::chrono::high_resolution_clock::now();
    
    // Launch threads
    for (int i = 0; i < num_threads; i++) {
        threads.emplace_back(cpu_intensive_task, i, iterations);
    }
    
    // Wait for all threads
    for (auto& t : threads) {
        t.join();
    }
    
    auto total_end = std::chrono::high_resolution_clock::now();
    auto total_duration = std::chrono::duration_cast<std::chrono::milliseconds>(total_end - total_start);
    
    std::cout << "Total execution time: " << total_duration.count() << "ms\n";
    
    return 0;
}
