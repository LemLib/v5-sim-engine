//
// Created by aang on 12/20/23.
//


#include "../include/bot.h"

namespace sim {
    Length trackWidth;
    Length wheelDiameter;
    AngularVelocity driveSpeed;
    Mass mass;
    Inertia inertia;

    std::vector<uint8_t> left, right;

    void setup() {

    }

    void update() {
        int lc, rc;
        Voltage lV, rV;

        for(uint8_t port : left) {
            if(!claim_port_try(port, pros::c::E_DEVICE_MOTOR)) continue;
            V5_DeviceT dev = registry_get_device(port)->device_info;
            if(dev->exists && dev->type == kDeviceTypeMotorSensor) {
                lV += dev->motor.voltage * (volt / 1000);
                lc++;
            }
            port_mutex_give(dev->port);
        }
        lV /= lc;

        for(uint8_t port : right) {
            if(!claim_port_try(port, pros::c::E_DEVICE_MOTOR)) continue;
            V5_DeviceT dev = registry_get_device(port)->device_info;
            if(dev->exists && dev->type == kDeviceTypeMotorSensor) {
                rV += dev->motor.voltage * (volt / 1000);
                rc++;
            }
            port_mutex_give(dev->port);
        }
        rV /= rc;
    }
}