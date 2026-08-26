#pragma once

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <utility>
#include <vector>

namespace inference::robot_detail {

class TargetInterpolator {
public:
    using Clock = std::chrono::steady_clock;
    using TimePoint = Clock::time_point;

    explicit TargetInterpolator(double duration_s = 0.0)
    {
        set_duration(duration_s);
    }

    void set_duration(double duration_s) noexcept
    {
        duration_s_ = std::isfinite(duration_s) && duration_s > 0.0
                          ? duration_s
                          : 0.0;
    }

    void reset(std::vector<double> target)
    {
        start_target_ = target;
        goal_target_ = target;
        current_target_ = std::move(target);
        active_ = false;
    }

    void set_target(const std::vector<double>& target, TimePoint now)
    {
        if (current_target_.empty() || current_target_.size() != target.size()) {
            reset(target);
            return;
        }

        if (duration_s_ <= 0.0) {
            reset(target);
            return;
        }

        sample(now);
        start_target_ = current_target_;
        goal_target_ = target;
        start_time_ = now;
        active_ = start_target_ != goal_target_;
    }

    const std::vector<double>& sample(TimePoint now)
    {
        if (!active_) {
            return current_target_;
        }

        const double elapsed_s =
            std::chrono::duration<double>(now - start_time_).count();
        const double ratio = std::clamp(elapsed_s / duration_s_, 0.0, 1.0);

        current_target_.resize(goal_target_.size());
        for (std::size_t i = 0; i < goal_target_.size(); ++i) {
            current_target_[i] =
                start_target_[i] +
                (goal_target_[i] - start_target_[i]) * ratio;
        }

        if (ratio >= 1.0) {
            current_target_ = goal_target_;
            start_target_ = goal_target_;
            active_ = false;
        }

        return current_target_;
    }

private:
    double duration_s_{0.0};
    TimePoint start_time_{Clock::now()};
    std::vector<double> start_target_;
    std::vector<double> goal_target_;
    std::vector<double> current_target_;
    bool active_{false};
};

template <std::size_t N>
class FixedTargetInterpolator {
public:
    using Clock = std::chrono::steady_clock;
    using TimePoint = Clock::time_point;
    using Target = std::array<double, N>;

    explicit FixedTargetInterpolator(double duration_s = 0.0)
    {
        set_duration(duration_s);
    }

    void set_duration(double duration_s) noexcept
    {
        duration_s_ = std::isfinite(duration_s) && duration_s > 0.0
                          ? duration_s
                          : 0.0;
    }

    void reset(const Target& target)
    {
        start_target_ = target;
        goal_target_ = target;
        current_target_ = target;
        active_ = false;
    }

    void set_target(const Target& target, TimePoint now)
    {
        if (duration_s_ <= 0.0) {
            reset(target);
            return;
        }

        sample(now);
        start_target_ = current_target_;
        goal_target_ = target;
        start_time_ = now;
        active_ = start_target_ != goal_target_;
    }

    const Target& sample(TimePoint now)
    {
        if (!active_) {
            return current_target_;
        }

        const double elapsed_s =
            std::chrono::duration<double>(now - start_time_).count();
        const double ratio = std::clamp(elapsed_s / duration_s_, 0.0, 1.0);

        for (std::size_t i = 0; i < N; ++i) {
            current_target_[i] =
                start_target_[i] +
                (goal_target_[i] - start_target_[i]) * ratio;
        }

        if (ratio >= 1.0) {
            current_target_ = goal_target_;
            start_target_ = goal_target_;
            active_ = false;
        }

        return current_target_;
    }

private:
    double duration_s_{0.0};
    TimePoint start_time_{Clock::now()};
    Target start_target_{};
    Target goal_target_{};
    Target current_target_{};
    bool active_{false};
};

}  // namespace inference::robot_detail
