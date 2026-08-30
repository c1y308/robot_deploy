#pragma once

namespace myactua {

constexpr double kPi = 3.14159265358979323846;
constexpr double kRadToDeg = 180.0 / kPi;

constexpr double kPosPulsePerRev = 131072.0;
constexpr double kRadToRawPos = kPosPulsePerRev / (2.0 * kPi);
constexpr double kRawPosToRad = (2.0 * kPi) / kPosPulsePerRev;

constexpr double kRawVelToRadPerSec = kRawPosToRad;
constexpr double kRadPerSecToRawVel = kRadToRawPos;

constexpr double kRawTorqueToPercent = 0.1;

constexpr double kAnklePosPulsePerRev = 2.0 * kPosPulsePerRev;
constexpr double kAnkleRadToRawPos = kAnklePosPulsePerRev / (2.0 * kPi);
constexpr double kAnkleRawPosToRad = (2.0 * kPi) / kAnklePosPulsePerRev;

constexpr double kAnkleRawVelToRadPerSec = kAnkleRawPosToRad;
constexpr double kAnkleRadPerSecToRawVel = kAnkleRadToRawPos;

} // namespace myactua
