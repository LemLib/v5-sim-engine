//
// Created by aang on 12/20/23.
//


#include "bot.h"
#include "pros/rtos.h"


namespace sim {

    Bot::Bot(std::initializer_list<uint8_t> left, std::initializer_list<uint8_t> right, V2Position start,
             Angle start_theta, Length wheel_radius, Length track_radius, AngularVelocity cartridge, double gear_ratio,
             Mass mass, Inertia inertia)
            : left(left), right(right), pos(start), theta(start_theta), wheel_radius(wheel_radius),
              track_radius(track_radius), cartridge(cartridge), mass(mass), inertia(inertia), gear_ratio(gear_ratio) {
        C = algebra::Matrix<int, 2, 2>({1, 0, 0, 1});
        D = algebra::Matrix<int, 2, 2>({0, 0, 0, 0});

        X_l = algebra::Vector2d({0, 0});
        X_r = algebra::Vector2d({0, 0});

        D1 = (1 / mass + units::square(track_radius) / inertia);
        D2 = (1 / mass - units::square(track_radius) / inertia);
    }

    void Bot::update(bool lock) {
        mutex.lock();
        if (lock)
            port_mutex_take_all();
        uint8_t lc = 0;
        uint8_t rc = 0;

        uint32_t time = pros::millis();
        Time dt = time * ms - ptime * ms;
        ptime = time;
        Voltage lV = 0_volt, rV = 0_volt;

        for (uint8_t port: left) {
            V5_DeviceT dev = registry_get_device(port - 1)->device_info;
            if (dev->exists && dev->type == kDeviceTypeMotorSensor) {
                Voltage mv = dev->motor.voltage * volt / 1000;
                lc++;
                if (lV == 0_volt || units::abs(mv) > units::abs(lV))
                    lV = -mv;
            }
        }
        for (uint8_t port: right) {
            V5_DeviceT dev = registry_get_device(port - 1)->device_info;
            if (dev->exists && dev->type == kDeviceTypeMotorSensor) {
                Voltage mv = dev->motor.voltage * volt / 1000;
                rc++;
                if (rV == 0_volt || units::abs(mv) > units::abs(rV))
                    rV = mv;
            }
        }
        // determine the actual torques, currents, etc for each drivetrain side based on motor count
        Torque stall_torque_l = lc * STALL_TORQUE;
        Torque stall_torque_r = rc * STALL_TORQUE;
        Current stall_current_l = lc * STALL_CURRENT;
        Current stall_current_r = rc * STALL_CURRENT;
        Current free_current_l = lc * FREE_CURRENT;
        Current free_current_r = rc * FREE_CURRENT;

        Quantity<std::ratio<1>, std::ratio<2>, std::ratio<-2>, std::ratio<0>, std::ratio<-1>> torque_const_l =
                stall_torque_l / stall_current_l;
        Quantity<std::ratio<1>, std::ratio<2>, std::ratio<-2>, std::ratio<0>, std::ratio<-1>> torque_const_r =
                stall_torque_r / stall_current_r;

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

        // create the matrices
        algebra::Matrix<double, 2, 2> lA_l({(D1 * C1_l).raw(), (D2 * C1_l).raw(),
                                                   (D2 * C1_l).raw(), (D1 * C1_l).raw()});
        algebra::Matrix<double, 2, 2> lA_r({(D1 * C1_r).raw(), (D2 * C1_r).raw(),
                                                   (D2 * C1_l).raw(), (D1 * C1_r).raw()});
        algebra::Matrix<double, 2, 2> lB_l({(D1 * C2_l).raw(), (D2 * C2_l).raw(),
                                                   (D2 * C2_l).raw(), (D1 * C2_l).raw()});
        algebra::Matrix<double, 2, 2> lB_r({(D1 * C2_r).raw(), (D2 * C2_r).raw(),
                                                   (D2 * C2_r).raw(), (D1 * C2_r).raw()});

        auto pairL = to_discrete<2>(lA_l, lB_l, dt.convert(sec));

        auto pairR = to_discrete<2>(lA_r, lB_r, dt.convert(sec));

        A_l = pairL.first;
        B_l = pairL.second;
        A_r = pairR.first;
        B_r = pairR.second;

        algebra::Matrix<double, 2, 1> e({lV.convert(volt), rV.convert(volt)});
        auto y_l = (A_l * X_l) + (B_l * e);
        auto y_r = (A_r * X_r) + (B_r * e);
        X_l = y_l;
        X_r = y_r;

        LinearVelocity leftSpeed = y_l(0, 0) * mps;
        LinearVelocity rightSpeed = y_r(1, 0) * mps;
        AngularVelocity leftOmega = leftSpeed * rad / (M_2_PI * wheel_radius);
        AngularVelocity rightOmega = leftSpeed * rad / (M_2_PI * wheel_radius);

        LinearVelocity linVel = (leftSpeed + rightSpeed) / 2;
        AngularVelocity omega = ((rightSpeed - leftSpeed) * rad) / (track_radius * 2);
        LinearVelocity vx = linVel * units::cos(theta);
        LinearVelocity vy = linVel * units::sin(theta);

        V2Position delta(vx * dt, vy * dt);

        vel = V2Velocity(vx, vy);

        pos += delta;
        theta += omega * dt;

        while (theta > 1_rot) theta -= 1_rot;
        while (theta < 0_rad) theta += 1_rot;


        for (uint8_t port: left) {
            _V5_Device dev = emu_smart_ports[port - 1];
            if (dev.exists && dev.type == kDeviceTypeMotorSensor) {
                double d = leftOmega.convert((rot / sec) / 180000);
                dev.motor.velocity = d;
                dev.motor.position += d;
                dev.timestamp = time;
            }
        }
        for (uint8_t port: right) {
            _V5_Device dev = emu_smart_ports[port - 1];
            if (dev.exists && dev.type == kDeviceTypeMotorSensor) {
                double d = rightOmega.convert((rot / sec) / 180000);
                dev.motor.velocity = d;
                dev.motor.position += d;
                dev.timestamp = time;
            }
        }
        for (uint8_t i = 0; i < V5_MAX_DEVICE_PORTS; i++) {
            _V5_Device dev = emu_smart_ports[i];
            if (!dev.exists || dev.type == kDeviceTypeNoSensor) continue;
            switch (dev.type) {
                case kDeviceTypeImuSensor:
                    dev.imu.rotation.x += omega.convert(degps) * dt.convert(sec);
                    dev.timestamp = time;
                    break;
                case kDeviceTypeGpsSensor:
                    dev.gps.position.x += delta.x.convert(in);
                    dev.gps.position.x += delta.y.convert(in);
                    dev.timestamp = time;
                    break;
                default:
                    break;
            }
        }
        timestamp = time;
        if (lock)
            port_mutex_give_all();
        mutex.give();
    }

    V2Position Bot::getPos() {
        mutex.lock();
        V2Position x = pos;
        mutex.unlock();
        return x;
    }

    std::pair<V2Position, V2Position> Bot::getWheelPos() {
        V2Position offset = {0_m, track_radius};
        mutex.lock();
        offset.rotateBy(theta);
        V2Position x = pos;
        mutex.unlock();
        return std::make_pair<V2Position, V2Position>(x + offset, x - offset);
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