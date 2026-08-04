#pragma once

namespace myactua {

constexpr double kPi = 3.14159265358979323846;
constexpr double kPosPulsePerRev = 131072.0;
constexpr double kRawPosToRad = (2.0 * kPi) / kPosPulsePerRev;
constexpr double kRawVelToRpm = 60.0 / kPosPulsePerRev;
constexpr double kRpmToRadPerSec = (2.0 * kPi) / 60.0;
constexpr double kRawVelToRadPerSec = kRawVelToRpm * kRpmToRadPerSec;
constexpr double kRadToDeg = 180.0 / kPi;

inline double raw_pos_to_rad(double raw_pos)
{
    return raw_pos * kRawPosToRad;
}

inline double rad_to_deg(double rad)
{
    return rad * kRadToDeg;
}

inline double raw_vel_to_rad_s(double raw_vel)
{
    return raw_vel * kRawVelToRadPerSec;
}

} // namespace myactua
