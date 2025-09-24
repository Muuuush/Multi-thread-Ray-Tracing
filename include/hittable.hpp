#pragma once

#include "interval.hpp"
#include "ray.hpp"
#include "rtweekend.hpp"

class material;

struct hit_record
{
    point3 p;
    vec3 normal;
    double t;
    bool front_face;
    shared_ptr<material> mat;

    void set_face_norm(const ray& r, const vec3& outward_norm)
    {
        front_face = dot(r.direction(), outward_norm) < 0;
        normal = front_face ? outward_norm : -outward_norm;
    }
};

class hittable
{
public:
    virtual ~hittable() = default;
    virtual bool hit(const ray& r, interval ray_t, hit_record& record) const = 0;
};
