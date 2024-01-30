#include <chrono>
#include <iostream>

class Timer
{
public:
    void start(){
        t1 = std::chrono::high_resolution_clock::now();
        use_t1 = true;
    };
    void end(){
        if (use_t1) {
            t2 = std::chrono::high_resolution_clock::now();
            duration = std::chrono::duration_cast<std::chrono::microseconds>(t2-t1).count();
        }
        else {
            t1 = std::chrono::high_resolution_clock::now();
            duration = std::chrono::duration_cast<std::chrono::microseconds>(t1-t2).count();
        }
        std::cout << "Time:" << duration / 1000000.0 << "s";
        std::cout << ",\tFPS:" << 1000000.0 / duration << std::endl;
    };
    void restart(){
        if (use_t1) {
            t2 = std::chrono::high_resolution_clock::now();
            duration = std::chrono::duration_cast<std::chrono::microseconds>(t2-t1).count();
        }
        else {
            t1 = std::chrono::high_resolution_clock::now();
            duration = std::chrono::duration_cast<std::chrono::microseconds>(t1-t2).count();
        }
        use_t1 = !use_t1;
        std::cout << "Time:" << duration;
        std::cout << ",\tFPS:" << 1000000.0 / duration << std::endl;
    }
    float get_duration(){
        return duration/1000000.0;
    }
private:
    std::chrono::high_resolution_clock::time_point t1, t2;
    bool use_t1;
    long duration;
};