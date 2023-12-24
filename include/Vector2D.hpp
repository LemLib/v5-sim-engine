#pragma once

#include <oneapi/tbb/version.h>

#include "units.hpp"

namespace sim {
    template<isQuantity T>
    class Vector2D {

    public:
        T x;
        T y;

        Vector2D() : x(0.0), y(0.0) {
        }

        Vector2D(T nx, T ny) : x(nx), y(ny) {
        }

        static Vector2D<T> fromPolar(Angle t, T m) {
            m = m.abs();
            return Vector2D(m * units::cos(t), m * units::sin(t));
        }

        static Vector2D<T> unitVector(Angle t) { return fromPolar(t,  T(1.0)); }

        T getX() { return x; }

        T getY() { return y; }

        Vector2D<T> operator+(Vector2D<T>& other) { return Vector2D<T>(x + other.x, y + other.y); }

        Vector2D<T> operator-(Vector2D<T>& other) { return Vector2D<T>(x - other.x, y - other.y); }

        Vector2D<T> operator*(double factor) { return Vector2D<T>(x * factor, y * factor); }

        Vector2D<T> operator/(double factor) { return Vector2D<T>(x / factor, y / factor); }

        double dot(Vector2D<T>& other) { return (x * other.x + y * other.y).getValue(); }

        Angle theta() { return atan2(y, x); }

        T magnitude() { return sqrt(square(x) + square(y)); }

        Vector2D vectorTo(Vector2D& other) { return Vector2D(other.x - x, other.y - y); }

        Angle angleTo(Vector2D& other) { return atan2(other.y - y, other.x - x); }

        T distance(Vector2D& other) { return sqrt(square(x - other.x, 2) + square(y - other.y, 2)); }

        Vector2D normalize() {
            T m = magnitude();
            return Vector2D(x / m, y / m);
        }

        void rotateBy(Angle angle) {
            T m = magnitude();
            Angle t = theta() + angle;
            x = m * units::cos(t);
            y = m * units::sin(t);
        }

        void rotateTo(Angle angle) {
            T m = magnitude();
            x = m * units::cos(angle);
            y = m * units::sin(angle);
        }

        Vector2D rotatedBy(Angle angle) {
            T m = magnitude();
            Angle t = theta() + angle;
            return fromPolar(t, m);
        }

        Vector2D rotatedTo(Angle angle) {
            T m = magnitude();
            return fromPolar(angle, m);
        }
    };

    typedef Vector2D<Length> V2Position;
    typedef Vector2D<LinearVelocity> V2Velocity;
    typedef Vector2D<LinearAcceleration> V2Acceleration;
    typedef Vector2D<Force> V2Force;

    template<isQuantity T>  Vector2D<T>& operator+=(Vector2D<T>& lhs, Vector2D<T>& rhs) {
        lhs.x += rhs.x;
        lhs.y += rhs.y;
        return lhs;
    }

    template<isQuantity T> Vector2D<T>& operator-=(Vector2D<T>& lhs, Vector2D<T>& rhs) {
        lhs.x -= rhs.x;
        lhs.y -= rhs.y;
        return lhs;
    }

    template<isQuantity T> Vector2D<T>& operator*=(T lhs, double factor) {
        lhs.x *= factor;
        lhs.y *= factor;
        return lhs;
    }

    template<isQuantity T> Vector2D<T>& operator/=(T lhs, double factor) {
        lhs.x /= factor;
        lhs.y /= factor;
        return lhs;
    }
}
