#pragma once

#include "rtweekend.hpp"
#include <cmath>
#include <format>

class vec3
{
public:
    double e[3];
    vec3() : e(0.0, 0.0, 0.0) {}
    vec3(double e0, double e1, double e2) : e(e0, e1, e2) {}

    double x() const { return e[0]; }
    double y() const { return e[1]; }
    double z() const { return e[2]; }

    vec3 operator-() const { return vec3(-e[0], -e[1], -e[2]); }
    double operator[](int i) const { return e[i]; }
    double& operator[](int i) { return e[i]; }

    vec3& operator*=(double t)
    {
        e[0] *= t;
        e[1] *= t;
        e[2] *= t;
        return *this;
    }
    vec3& operator/=(double t) { return *this *= 1 / t; }

    double length() const { return std::sqrt(length_squared()); }
    double length_squared() const { return e[0] * e[0] + e[1] * e[1] + e[2] * e[2]; }
    vec3 unit() const;

    static vec3 random()
    {
        return vec3(random_double(), random_double(), random_double());
    }
    static vec3 random(double min, double max)
    {
        return vec3(random_double(min, max), random_double(min, max), random_double(min, max));
    }
    static vec3 random_unit();
    static vec3 random_on_hemisphere(const vec3& normal);
    static vec3 random_in_unit_disk();

    bool near_zero() const
    {
        return (std::fabs(e[0]) < epsilon) && (std::fabs(e[1]) < epsilon) && (std::fabs(e[2]) < epsilon);
    }

    std::string to_string() const { return std::format("{} {} {}", e[0], e[1], e[2]); }
};

// point3 is just an alias for vec3, but useful for geometric clarity in the code.
using point3 = vec3;

// Vector Utility Functions

inline vec3 operator+(const vec3& u, const vec3& v)
{
    return vec3(u.e[0] + v.e[0], u.e[1] + v.e[1], u.e[2] + v.e[2]);
}

inline vec3 operator-(const vec3& u, const vec3& v)
{
    return vec3(u.e[0] - v.e[0], u.e[1] - v.e[1], u.e[2] - v.e[2]);
}

inline vec3 operator*(const vec3& u, const vec3& v)
{
    return vec3(u.e[0] * v.e[0], u.e[1] * v.e[1], u.e[2] * v.e[2]);
}

inline vec3 operator*(double t, const vec3& v)
{
    return vec3(t * v.e[0], t * v.e[1], t * v.e[2]);
}

inline vec3 operator*(const vec3& v, double t)
{
    return t * v;
}

inline vec3 operator/(const vec3& v, double t)
{
    return (1 / t) * v;
}

inline double dot(const vec3& u, const vec3& v)
{
    return u.e[0] * v.e[0] + u.e[1] * v.e[1] + u.e[2] * v.e[2];
}

inline vec3 cross(const vec3& u, const vec3& v)
{
    return vec3(u.e[1] * v.e[2] - u.e[2] * v.e[1],
                u.e[2] * v.e[0] - u.e[0] * v.e[2],
                u.e[0] * v.e[1] - u.e[1] * v.e[0]);
}

inline vec3 unit_vector(const vec3& v)
{
    return v / v.length();
}

inline vec3 vec3::random_unit()
{
    while (true)
    {
        vec3 p = random(-1, 1);
        double lensq = p.length_squared();
        if (epsilon < lensq && lensq <= 1)
            return p / sqrt(lensq);
    }
}

inline vec3 vec3::random_on_hemisphere(const vec3& normal)
{
    vec3 on_unit_sphere = random_unit();
    if (dot(on_unit_sphere, normal) > 0.0) // In the same hemisphere as the normal
        return on_unit_sphere;
    else
        return -on_unit_sphere;
}

inline vec3 vec3::unit() const
{
    return (*this) / this->length();
}

inline vec3 reflect(const vec3& v, const vec3& n)
{
    return v - 2 * dot(v, n) * n;
}

inline vec3 refract(const vec3& v_unit, const vec3 n, double etai_over_etat)
{
    auto cos_theta = std::fmin(dot(-v_unit, n), 1.0);
    vec3 r_out_perp = etai_over_etat * (v_unit + cos_theta * n);
    vec3 r_out_parallel = -std::sqrt(std::fabs(1.0 - r_out_perp.length_squared())) * n;
    return r_out_perp + r_out_parallel;
}

inline vec3 vec3::random_in_unit_disk()
{
    while (true)
    {
        auto p = vec3(random_double(-1, 1), random_double(-1, 1), 0);
        if (p.length_squared() < 1)
            return p;
    }
}
