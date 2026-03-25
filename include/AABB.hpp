#pragma once

#include "vec3.hpp"

struct AABB {
    vec3 min;
    vec3 max;
    AABB() = default;
    AABB(const vec3& min, const vec3& max) : min(min), max(max) {};

    static AABB combine(const AABB& a, const AABB& b)
    {
        return AABB(
            vec3(std::min(a.min.x(), b.min.x()), std::min(a.min.y(), b.min.y()), std::min(a.min.z(), b.min.z())),
            vec3(std::max(a.max.x(), b.max.x()), std::max(a.max.y(), b.max.y()), std::max(a.max.z(), b.max.z()))
        );
    }
};
