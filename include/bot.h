#pragma once

#include "Vector2D.hpp"
#include "util.h"
#include "emu_devices.h"
#include "vdml/vdml.h"
#include "matrix.h"
#include "../include/math.h"
#include "pros/rtos.hpp"
#include <set>

namespace sim {
    constexpr Current STALL_CURRENT = 2.5_amp;
    constexpr Current FREE_CURRENT = 0.13_amp;
    constexpr Torque STALL_TORQUE = 2.1_nm;
    constexpr Power POWER_MAX = 12.75_watt;
    constexpr Voltage VOLTAGE_MAX = 12_volt;

    class Bot {
    public:
    private:
        pros::Mutex mutex;
        V2Position pos = {0_in, 0_in};
        V2Velocity vel;
        Angle theta;

        uint32_t timestamp;

        Length track_radius;
        Length wheel_radius;
        AngularVelocity cartridge;
        Mass mass;
        Inertia inertia;

        std::set<uint8_t> left, right;
        Quantity<std::ratio<-1>, std::ratio<-2>, std::ratio<2>, std::ratio<1>, std::ratio<1>> angular_vel_constant_l, angular_vel_constant_r;
        Quantity<std::ratio<1>, std::ratio<0>, std::ratio<-1>, std::ratio<-1>, std::ratio<0>> C1_l, C1_r;
        Quantity<std::ratio<0>, std::ratio<-1>, std::ratio<1>, std::ratio<0>, std::ratio<1>> C2_l, C2_r;
        Quantity<std::ratio<-1>, std::ratio<0>, std::ratio<0>, std::ratio<0>, std::ratio<0>> D1, D2;
        algebra::Matrix2d A_l, A_r, B_l, B_r;
        algebra::Matrix<int, 2, 2> C, D;
        algebra::Vector2d X_l, X_r;
    public:
        Bot(std::initializer_list<uint8_t> left, std::initializer_list<uint8_t> right, V2Position start, Angle start_theta, Length wheel_radius, Length track_radius, AngularVelocity cartridge, double gear_ratio, Mass mass, Inertia inertia = 0.5_kgm2);
        void update(bool lock);
        V2Position getPos();
        std::pair<V2Position, V2Position> getWheelPos();
        V2Velocity getVel();
        Angle getTheta();
    };

}