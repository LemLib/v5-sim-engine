#pragma once
#include "Vector2D.hpp"
#include "util.h"
#include "emu_devices.h"
#include "vdml/vdml.h"

namespace sim {
    constexpr Current iStall = 2.5_amp;
    constexpr Torque tStall = 2.1_nm;
    constexpr Power pMax = 12.75_watt;
    constexpr Voltage vMax = 12_volt;
}