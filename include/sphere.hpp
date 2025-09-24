#pragma once

#include "hittable.hpp"
#include "material.hpp"

class sphere : public hittable
{
public:
    sphere(const point3& center, double radius, shared_ptr<material> material)
        : center(center), radius(std::fmax(radius, 0)), mat(material) {}

    bool hit(const ray& r, interval ray_t, hit_record& record) const override
    {
        const auto oc = center - r.origin();
        const auto a = r.direction().length_squared();
        const auto h = dot(r.direction(), oc);
        const auto c = oc.length_squared() - radius * radius;
        const auto discriminant = h * h - a * c;

        if (discriminant < 0)
            return false;

        const auto sqrtd = std::sqrt(discriminant);
        double t = (h - sqrtd) / a;
        if (!ray_t.surrounds(t))
        {
            t = (h + sqrtd) / a;
            if (!ray_t.surrounds(t))
                return false;
        }

        record.t = t;
        record.p = r.at(t);
        vec3 outward_norm = (record.p - center) / radius;
        record.set_face_norm(r, outward_norm);
        record.mat = mat;

        return true;
    }

private:
    point3 center;
    double radius;
    shared_ptr<material> mat;
};
