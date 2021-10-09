#pragma once
#include <iostream>
#include <cassert>
#include <cmath>
#include <vector>
using namespace std;

struct GLVec3
{

    float x, y, z;

    GLVec3() {}
    GLVec3(float x, float y, float z) : x(x), y(y), z(z) {}
    template <class C>
    explicit GLVec3(const C &c) : x(c[0]), y(c[1]), z(c[2]) {} // forbid  B b4 = {4, 5};
    // GLVec3(const GLVec3 &c) : x(c[0]), y(c[1]), z(c[2]) {}
    GLVec3 &operator=(const GLVec3 &v)
    {
        x = v.x;
        y = v.y;
        z = v.z;
        return *this;
    }

    float operator[](int i) const
    {
        return (&x)[i];
    }

    float &operator[](int i)
    {
        return (&x)[i];
    }

    operator const float *() const
    {
        return &x;
    }

    operator float *()
    {
        return &x;
    }

    //--------------------- Algebric Computations ----------------------------
    friend GLVec3 operator+(const GLVec3 &a, const GLVec3 &b)
    {
        return GLVec3(a.x + b.x, a.y + b.y, a.z + b.z);
    }

    friend GLVec3 operator-(const GLVec3 &a, const GLVec3 &b)
    {
        return GLVec3(a.x - b.x, a.y - b.y, a.z - b.z);
    }

    friend GLVec3 operator-(const GLVec3 &a) { return GLVec3(-a.x, -a.y, -a.z); }

    friend GLVec3 operator*(const GLVec3 &a, float k)
    {
        return GLVec3(a.x * k, a.y * k, a.z * k);
    }

    friend GLVec3 operator*(float k, const GLVec3 &a) { return a * k; }

    friend GLVec3 operator/(const GLVec3 &a, float k)
    {
        if (fabs(k) < 1.0e-10)
        {
            assert(false && "too small to be divided!");
        }
        return GLVec3(a.x / k, a.y / k, a.z / k);
    }

    friend bool operator!=(const GLVec3 &a, const GLVec3 &b) { return !(a == b); }

    friend bool operator==(const GLVec3 &a, const GLVec3 &b)
    {
        const float epsilon = 1.0E-10;
        return (a - b).squaredNorm() < epsilon;
    }

    GLVec3 &operator+=(const GLVec3 &a)
    {
        x += a.x;
        y += a.y;
        z += a.z;
        return *this;
    }

    GLVec3 &operator-=(const GLVec3 &a)
    {
        x -= a.x;
        y -= a.y;
        z -= a.z;
        return *this;
    }

    GLVec3 &operator*=(float k)
    {
        x *= k;
        y *= k;
        z *= k;
        return *this;
    }

    friend float operator*(const GLVec3 &a, const GLVec3 &b)
    {
        return a.x * b.x + a.y * b.y + a.z * b.z;
    }

    GLVec3 &operator/=(float k)
    {
        if (fabs(k) < 1.0E-10)
            assert(false && "Vec::operator /= : dividing by a null value (%f)");
        x /= k;
        y /= k;
        z /= k;
        return *this;
    }

    friend GLVec3 cross(const GLVec3 &a, const GLVec3 &b)
    {
        return GLVec3(a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x);
    }

    GLVec3 orthogonalVec() const
    {
        if (fabs(y) >= 0.9 * fabs(x) && fabs(z) >= 0.9 * fabs(x))
        {
            return GLVec3(0.0, -z, y);
        }
        else if ((fabs(x) >= 0.9 * fabs(y)) && (fabs(z) >= 0.9 * fabs(y)))
            return GLVec3(-z, 0.0, x);
        else
            return GLVec3(-y, x, 0.0);
    }

    float squaredNorm() const { return x * x + y * y + z * z; }

    float norm() const { return sqrt(x * x + y * y + z * z); }

    float normalize()
    {
        const float n = norm();
        if (n < 1.0E-10)
            assert(false && "Vec::normalize: normalizing a null vector (norm=%f)");
        *this /= n;
        return n;
    }

    GLVec3 unit() const
    {
        GLVec3 v = *this;
        v.normalize();
        return v;
    }

    void projectOnAxis(const GLVec3 &direction)
    {
        *this = (((*this) * direction) / direction.squaredNorm()) * direction;
    }

    void projectOnPlane(const GLVec3 &normal)
    {
        *this -= (((*this) * normal) / normal.squaredNorm()) * normal;
    }
};

inline std::ostream &operator<<(std::ostream &out, const GLVec3 &v)
{
    out << v.x << ", " << v.y << ", " << v.z;
    return out;
}

struct GLVec2
{
    float x, y;
    GLVec2() {}
    GLVec2(float x, float y) : x(x), y(y) {}
};

struct GLVertex3
{
    GLVec3 position;
    GLVec2 tex_coord;
};

typedef std::vector<GLVec3> GLLine3;