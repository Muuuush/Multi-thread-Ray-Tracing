#pragma once

#include "color.hpp"
#include "hittable.hpp"

class material
{
public:
    virtual ~material() = default;

    virtual bool scatter(
        const ray& r_in, const hit_record& record, color& attenuation, ray& scattered)
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
        const ray& r_in, const hit_record& record, color& attenuation, ray& scattered)
        const override
    {
        auto scattered_direction = record.normal + vec3::random_unit();
        if (scattered_direction.near_zero())
            scattered_direction = record.normal;
        scattered = ray(record.p, scattered_direction);
        attenuation = albedo;
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
        const ray& r_in, const hit_record& record, color& attenuation, ray& scattered)
        const override
    {
        vec3 reflected = reflect(r_in.direction(), record.normal).unit();
        vec3 fuzzed;
        do {
            fuzzed = reflected + (fuzz * vec3::random_unit());
        } while (dot(fuzzed, record.normal) < 0);
        scattered = ray(record.p, fuzzed);
        attenuation = albedo;
        return true;
    }

private:
    color albedo;
    double fuzz;
};

class dielectric : public material
{
public:
    dielectric(double refraction_index) : refraction_index(refraction_index) {}

    bool scatter(const ray& r_in, const hit_record& rec, color& attenuation, ray& scattered)
        const override
    {
        attenuation = color(1.0, 1.0, 1.0);
        double ri = rec.front_face ? (1.0 / refraction_index) : refraction_index;

        vec3 unit_direction = unit_vector(r_in.direction());
        double cos_theta = std::fmin(dot(-unit_direction, rec.normal), 1.0);
        double sin_theta = std::sqrt(1.0 - cos_theta * cos_theta);

        bool cannot_refract = ri * sin_theta > 1.0;
        vec3 direction;

        if (cannot_refract || reflectance(cos_theta, ri) > random_double())
            direction = reflect(unit_direction, rec.normal);
        else
            direction = refract(unit_direction, rec.normal, ri);

        scattered = ray(rec.p, direction);
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
