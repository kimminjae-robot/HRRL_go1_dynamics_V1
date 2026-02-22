#pragma once
#include <chrono>
#include <cstdio>

inline void LogT(const char* tag = "")
{
    using clk = std::chrono::steady_clock;   // 시간 역행 없음 (프로파일용)
    using ns  = std::chrono::nanoseconds;

    static const auto t0 = clk::now();       // 프로그램 시작 기준
    static thread_local auto t_prev = t0;    // 스레드별 직전 호출 시각

    const auto t = clk::now();

    const auto since0_ns = std::chrono::duration_cast<ns>(t - t0).count();
    const auto delta_ns  = std::chrono::duration_cast<ns>(t - t_prev).count();
    t_prev = t;

    // [t=123.456789 ms | dt=0.123456 ms] TAG
    std::printf("[t=%9.3f ms | dt=%9.3f ms] %s\n",
                since0_ns / 1e6, delta_ns / 1e6, tag);
}
