#ifndef LINEAR_ALGEBRA_H_
#define LINEAR_ALGEBRA_H_

#include <cmath>
#include <cstddef>

struct Vector3f {
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;

    // Default constructors
    Vector3f() = default;
    Vector3f(float x_val, float y_val, float z_val) : x(x_val), y(y_val), z(z_val) {}

    // Array-like subscript operators for compatibility
    float operator[](std::size_t index) const {
        if (index == 0) return x;
        if (index == 1) return y;
        return z;
    }

    float& operator[](std::size_t index) {
        if (index == 0) return x;
        if (index == 1) return y;
        return z;
    }

    // Vector operations
    Vector3f operator+(const Vector3f& other) const {
        return Vector3f(x + other.x, y + other.y, z + other.z);
    }

    Vector3f operator-(const Vector3f& other) const {
        return Vector3f(x - other.x, y - other.y, z - other.z);
    }

    Vector3f operator*(float scalar) const {
        return Vector3f(x * scalar, y * scalar, z * scalar);
    }

    friend Vector3f operator*(float scalar, const Vector3f& vec) {
        return vec * scalar;
    }

    Vector3f operator/(float scalar) const {
        if (scalar != 0.0f) {
            float inv = 1.0f / scalar;
            return Vector3f(x * inv, y * inv, z * inv);
        }
        return Vector3f(0.0f, 0.0f, 0.0f);
    }

    Vector3f& operator+=(const Vector3f& other) {
        x += other.x;
        y += other.y;
        z += other.z;
        return *this;
    }

    Vector3f& operator-=(const Vector3f& other) {
        x -= other.x;
        y -= other.y;
        z -= other.z;
        return *this;
    }

    Vector3f& operator*=(float scalar) {
        x *= scalar;
        y *= scalar;
        z *= scalar;
        return *this;
    }

    Vector3f& operator/=(float scalar) {
        if (scalar != 0.0f) {
            float inv = 1.0f / scalar;
            x *= inv;
            y *= inv;
            z *= inv;
        } else {
            x = 0.0f;
            y = 0.0f;
            z = 0.0f;
        }
        return *this;
    }

    // Mathematical methods
    float dot(const Vector3f& other) const {
        return x * other.x + y * other.y + z * other.z;
    }

    Vector3f cross(const Vector3f& other) const {
        return Vector3f(
            y * other.z - z * other.y,
            z * other.x - x * other.z,
            x * other.y - y * other.x
        );
    }

    float magnitude() const {
        return std::sqrt(x * x + y * y + z * z);
    }

    float magSq() const {
        return x * x + y * y + z * z;
    }

    Vector3f normalized() const {
        float mag = magnitude();
        return (mag > 0.0f) ? (*this / mag) : Vector3f(0.0f, 0.0f, 0.0f);
    }
};

#endif // LINEAR_ALGEBRA_H_
