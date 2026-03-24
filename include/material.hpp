#pragma once

#include "color.hpp"
#include "hittable.hpp"

class material
{
public:
    virtual ~material() = default;

    virtual bool scatter(
        const ray& r_in, const hit_record& record, color& attenuation, ray& scattered, double& p)
        const
    {
        return false;
    }
};

class lambertian : public material
{
public:
    lambertian(const color& albedo) : albedo(albedo) {}
    bool scatter(
        const ray& r_in, const hit_record& record, color& attenuation, ray& scattered, double& p)
        const override
    {
        auto scattered_direction = record.normal + vec3::random_unit();
        if (scattered_direction.near_zero())
            scattered_direction = record.normal;
        double accept_p = 0;
        vec3 dir;
        do
        {
            dir = vec3::random_on_hemisphere(record.normal);
            accept_p = dot(dir, record.normal);
        } while (random_double() > accept_p);
        scattered = ray(record.p, dir);
        attenuation = albedo;

        auto theta = std::acos(dot(dir, record.normal));
        p = std::sin(theta * 2) / 2 / pi;
        return true;
    }

private:
    color albedo;
};

class metal : public material
{
public:
    metal(const color& albedo, double fuzz = 0.0)
        : albedo(albedo), fuzz(std::fmin(fuzz, 1)) {}
    bool scatter(
        const ray& r_in, const hit_record& record, color& attenuation, ray& scattered, double& p)
        const override
    {
        vec3 reflected = reflect(r_in.direction(), record.normal).unit();

        vec3 u0 = cross(record.normal, reflected);
        while (u0.near_zero())
            u0 = cross(record.normal, record.normal + vec3::random_unit());
        vec3 v0 = cross(u0, reflected);
        vec3 u = u0.unit();
        vec3 v = v0.unit();

        auto rand_unit = vec3::random_in_unit_disk();
        auto rand_fuzz = fuzz * rand_unit;
        auto fuzzed = reflected + rand_fuzz.x() * u + rand_fuzz.y() * v;

        scattered = ray(record.p, unit_vector(fuzzed));
        attenuation = albedo;
        p = 1;

        // if dot product is negative
        // then scatter failed with p
        return dot(fuzzed, record.normal) >= 0;
    }

private:
    color albedo;
    double fuzz;
};

class dielectric : public material
{
public:
    dielectric(double refraction_index) : refraction_index(refraction_index) {}

    bool scatter(const ray& r_in, const hit_record& rec, color& attenuation, ray& scattered, double& p)
        const override
    {
        attenuation = color(1.0, 1.0, 1.0);
        double ri = rec.front_face ? (1.0 / refraction_index) : refraction_index;

        vec3 unit_direction = unit_vector(r_in.direction());
        double cos_theta = std::fmin(dot(-unit_direction, rec.normal), 1.0);
        double sin_theta = std::sqrt(1.0 - cos_theta * cos_theta);

        bool cannot_refract = ri * sin_theta > 1.0;
        vec3 direction;

        double reflectance = this->reflectance(cos_theta, ri);
        if (cannot_refract || reflectance > random_double())
        {
            direction = reflect(unit_direction, rec.normal);
            p = reflectance;
        }
        else
        {
            direction = refract(unit_direction, rec.normal, ri);
            p = 1 - reflectance;
        }

        scattered = ray(rec.p, unit_vector(direction));
        return true;
    }

private:
    // Refractive index in vacuum or air, or the ratio of the material's refractive index over
    // the refractive index of the enclosing media
    double refraction_index;

    static double reflectance(double cosine, double refraction_index)
    {
        // Use Schlick's approximation for reflectance.
        auto r0 = (1 - refraction_index) / (1 + refraction_index);
        r0 = r0 * r0;
        return r0 + (1 - r0) * std::pow((1 - cosine), 5);
    }
};
