//
// Created by aang on 12/20/23.
//


#include "bot.h"


namespace sim {


    Bot::Bot(std::initializer_list<uint8_t> left, std::initializer_list<uint8_t> right, double gear_ratio) : left(left), right(right), gear_ratio(gear_ratio) {
        mutex.lock();
        int lc = 0, rc = 0;

        // figure out the motor count and voltage setting for each side

        for (uint8_t port: left) {
            if (!claim_port_try(port, pros::c::E_DEVICE_MOTOR)) continue;
            V5_DeviceT dev = registry_get_device(port)->device_info;
            if (dev->exists && dev->type == kDeviceTypeMotorSensor) {
                lV += dev->motor.voltage * (volt / 1000);
                lc++;
            }
            port_mutex_give(dev->port);
        }
        for (uint8_t port: right) {
            if (!claim_port_try(port, pros::c::E_DEVICE_MOTOR)) continue;
            V5_DeviceT dev = registry_get_device(port)->device_info;
            if (dev->exists && dev->type == kDeviceTypeMotorSensor) {
                rV += dev->motor.voltage * (volt / 1000);
                rc++;
            }
            port_mutex_give(dev->port);
        }
        lV /= lc;
        rV /= rc;

        // determine the actual torques, currents, etc for each drivetrain side based on motor count
        Torque stall_torque_l = lc * STALL_TORQUE;
        Torque stall_torque_r = rc * STALL_TORQUE;
        Current stall_current_l = lc * STALL_CURRENT;
        Current stall_current_r = rc * STALL_CURRENT;
        Current free_current_l = lc * FREE_CURRENT;
        Current free_current_r = rc * FREE_CURRENT;

        Quantity<std::ratio<1>, std::ratio<2>, std::ratio<-2>, std::ratio<0>, std::ratio<-1>> torque_const_l = stall_torque_l / stall_current_l;
        Quantity<std::ratio<1>, std::ratio<2>, std::ratio<-2>, std::ratio<0>, std::ratio<-1>> torque_const_r = stall_torque_r / stall_current_r;

        Resistance resistance_l = VOLTAGE_MAX / stall_current_l;
        Resistance resistance_r = VOLTAGE_MAX / stall_current_r;

        // calculate angular velocity constants
        angular_vel_constant_l = cartridge / (VOLTAGE_MAX - resistance_l * free_current_l);
        angular_vel_constant_r = cartridge / (VOLTAGE_MAX - resistance_r * free_current_l);

        // calculate variables C1 & C2
        C1_l = (-(std::pow(gear_ratio, 2) * torque_const_l) /
                (angular_vel_constant_l * resistance_l * units::square(wheel_radius)));
        C1_r = -(std::pow(gear_ratio, 2) * torque_const_r) /
               (angular_vel_constant_r * resistance_r * units::square(wheel_radius));
        C2_l = (gear_ratio * torque_const_l) / (resistance_l * wheel_radius);
        C2_r = (gear_ratio * torque_const_r) / (resistance_r * wheel_radius);

        D1 = (1 / mass + units::square(track_radius) / inertia);
        D2 = (1 / mass - units::square(track_radius) / inertia);

        // create the matrices
        A_l = algebra::Matrix<double, 2, 2>({(D1 * C1_l).raw(), (D2 * C1_l).raw(),
                                             (D2 * C1_l).raw(), (D1 * C1_l).raw()});
        A_r = algebra::Matrix<double, 2, 2>({(D1 * C1_r).raw(), (D2 * C1_r).raw(),
                                             (D2 * C1_l).raw(), (D1 * C1_r).raw()});
        B_l = algebra::Matrix<double, 2, 2>({(D1 * C2_l).raw(), (D2 * C2_l).raw(),
                                             (D2 * C2_l).raw(), (D1 * C2_l).raw()});
        B_r = algebra::Matrix<double, 2, 2>({(D1 * C2_r).raw(), (D2 * C2_r).raw(),
                                             (D2 * C2_r).raw(), (D1 * C2_r).raw()});
        C = algebra::Matrix<int, 2, 2>({1, 0, 0, 1});
        D = algebra::Matrix<int, 2, 2>({0, 0, 0, 0});
        X_l = algebra::Vector2d({0, 0});
        X_r = algebra::Vector2d({0, 0});

        mutex.unlock();

    }

    void Bot::update() {
        mutex.lock();
        algebra::Matrix<double, 2, 1> e({lV.convert(volt), rV.convert(volt)});
        auto y_l = (A_l * X_l) + (B_l * e);
        auto y_r = (A_r * X_r) + (B_r * e);

        Time dt = 4_ms;
        LinearVelocity leftSpeed = y_l(0, 0) * mps;
        LinearVelocity rightSpeed = y_r(1, 0) * mps;
        LinearVelocity linVel = (leftSpeed + rightSpeed) / 2;
        AngularVelocity omega = ((rightSpeed - leftSpeed) * rad) / (track_radius * 2);
        LinearVelocity vx = linVel * units::cos(theta);
        LinearVelocity vy = linVel * units::sin(theta);

        V2Position delta(vx * dt, vy * dt);

        vel = V2Velocity(vx, vy);

        pos += delta;
        theta += omega * dt;
        mutex.unlock();
    }

    V2Position Bot::getPos() {
        mutex.lock();
        V2Position x = pos;
        mutex.unlock();
        return x;
    }

    V2Velocity Bot::getVel() {
        mutex.lock();
        V2Velocity x = vel;
        mutex.unlock();
        return x;
    }

    Angle Bot::getTheta() {
        mutex.lock();
        Angle x = theta;
        mutex.unlock();
        return x;
    }
}