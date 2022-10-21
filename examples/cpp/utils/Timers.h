// @file      Timers.h
// @author    Ignacio Vizzo     [ivizzo@uni-bonn.de]
//
// Copyright (c) 2021 Ignacio Vizzo, all rights reserved
//
// Partially inspired in Open3D/cpp/utility/Timer.h
#pragma once

#include <numeric>
#ifndef FMT_HEADER_ONLY
#define FMT_HEADER_ONLY
#endif
#include <fmt/core.h>

#include <chrono>
#include <ratio>
#include <string>

namespace type_traits {

template <typename>
struct get_string_type {
    static const char* value();
};

#define REGISTER_PERIOD_TYPE(T)                             \
    template <>                                             \
    struct get_string_type<T> {                             \
        static const char* value() { return #T "seconds"; } \
    };

using namespace std;
REGISTER_PERIOD_TYPE(atto);
REGISTER_PERIOD_TYPE(femto);
REGISTER_PERIOD_TYPE(pico);
REGISTER_PERIOD_TYPE(nano);
REGISTER_PERIOD_TYPE(micro);
REGISTER_PERIOD_TYPE(milli);
REGISTER_PERIOD_TYPE(centi);
REGISTER_PERIOD_TYPE(deci);
REGISTER_PERIOD_TYPE(deca);
REGISTER_PERIOD_TYPE(hecto);
REGISTER_PERIOD_TYPE(kilo);
REGISTER_PERIOD_TYPE(mega);
REGISTER_PERIOD_TYPE(giga);
REGISTER_PERIOD_TYPE(tera);
REGISTER_PERIOD_TYPE(peta);
REGISTER_PERIOD_TYPE(exa);

}  // namespace type_traits

namespace timers {

template <typename Period = std::milli, typename FloatT = double>
class ScopeTimer {
public:
    using TimeT = std::chrono::duration<FloatT, Period>;
    ScopeTimer(const std::string& timer_info) : timer_info_(timer_info) { start_time_ = tic(); }
    ~ScopeTimer() {
        fmt::print("{} took {:.2f} {}.\n", timer_info_, GetDuration(), GetPeriodType());
    }

private:
    inline FloatT GetDuration() const { return tic() - start_time_; }
    inline std::string GetPeriodType() const {
        return type_traits::get_string_type<Period>::value();
    }
    inline FloatT tic() const {
        TimeT current_time = std::chrono::high_resolution_clock::now().time_since_epoch();
        return current_time.count();
    }

public:
    // Avoid strange creation of the ScopteTimer object
    ScopeTimer() = delete;
    ScopeTimer(const ScopeTimer&) = delete;
    void operator=(const ScopeTimer&) = delete;

private:
    FloatT start_time_ = 0.0;
    std::string timer_info_;
};

template <int N = 10, typename Period = std::ratio<1, 1>, typename FloatT = double>
class FPSTimer {
public:
    using TimeT = std::chrono::duration<FloatT, Period>;
    void tic() { t0_ = now(); }
    void toc() {
        times_[frame_idx_++] = now() - t0_;
        if (frame_idx_ == N) {
            frame_idx_ = 0;
            auto FPS = N / std::accumulate(times_.cbegin(), times_.cend(), 0.0);
            fmt::print("Integrating scans at {:.2f} FPS\n", FPS);
        }
    }
    [[nodiscard]] static FloatT now() {
        TimeT current_time = std::chrono::high_resolution_clock::now().time_since_epoch();
        return current_time.count();
    }

    int frame_idx_ = 0;
    FloatT t0_ = 0.0;
    std::array<FloatT, N> times_{0.0};
};
}  // namespace timers
