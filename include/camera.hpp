#pragma once

#include "color.hpp"
#include "ray.hpp"
#define STB_IMAGE_IMPLEMENTATION
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include <BBLog/BBLog.hpp>
#include <chrono>
#include <stb/stb_image.h>
#include <stb/stb_image_write.h>
#include <thread>

#include "hittable.hpp"
#include "material.hpp"
#include "rtweekend.hpp"

class camera
{
    using uchar = unsigned char;

public:
    int n = 3;
    double aspect = 1.0;
    int img_width = 100;
    std::string img_file = "out.png";

    double vfov = 90;
    point3 lookfrom = point3(0, 0, 0);
    point3 lookat = point3(0, 0, -1);
    vec3 vup = vec3(0, 1, 0);

    double defocus_angle = 0;
    double focus_dist = 10;

    int samples_per_pixel = 10;
    double terminated_posibility = 0.05;

    int thread_num = 16;

    camera() = default;

    void render(const hittable& world)
    {
        initialize();

        bb::log("image size: {} * {}", img_width, img_height);
        bb::log("channel number: {}", n);
        bb::log("camera position: {}", lookfrom.to_string());
        bb::log("vfov: {}", vfov);
        bb::log("samples per pixel: {}", samples_per_pixel);
        bb::log("terminated posibility: {}", terminated_posibility);
        bb::log("rendering...");

        // output image
        const int total_pixels = img_width * img_height;
        const auto start_time = std::chrono::system_clock::now();
        int rendered_pixels = 0;
        uchar* img = new uchar[total_pixels * n];

        std::vector<std::thread> threads(thread_num);

        for (int j = 0; j < img_height; j++)
        {
            std::cout << "\rRendered: " << rendered_pixels << " / " << total_pixels;
            const auto now_time = std::chrono::system_clock::now();
            const std::chrono::duration<double> duration = now_time - start_time;
            const double percentage = static_cast<double>(rendered_pixels) / total_pixels;
            std::cout << " || " << static_cast<int>(percentage * 1000) / 10.0 << "%";
            std::cout << " || " << duration.count() << " / " << (duration.count() / percentage) << " seconds                           ";
            std::flush(std::cout);

            auto render_pixel = [&](int from, int to) {
                for (int i = from; i < to; i++)
                {
                    color pixel_color;
                    for (int sample = 0; sample < samples_per_pixel; sample++)
                        pixel_color = pixel_color + ray_color(get_ray(i, j), world);
                    write_color(img, pixel_samples_scales * pixel_color, i, j, img_width, n);
                }
            };

            const int step = img_width / thread_num;
            for (int t = 0; t < thread_num; t++)
            {
                if (t < thread_num - 1)
                    threads[t] = std::thread(render_pixel, step * t, step * t + step);
                else
                    threads[t] = std::thread(render_pixel, step * t, img_width);
            }
            for (int t = 0; t < thread_num; t++)
                threads[t].join();
            rendered_pixels += img_width;
        }
        std::cout << "\r" << std::flush;

        // output
        auto w_res = stbi_write_png(img_file.c_str(), img_width, img_height, n, img, 0);
        bb::log("{} written: {}", img_file, w_res);
        // free
        delete[] img;

        bb::log("Done");
    }

private:
    int img_height;
    point3 center;
    point3 pixel00;
    vec3 delta_u;
    vec3 delta_v;
    double pixel_samples_scales;
    vec3 u, v, w;
    vec3 defocus_disk_u;
    vec3 defocus_disk_v;

    void initialize()
    {
        img_height = int(img_width / aspect);
        img_height = (img_height < 1) ? 1 : img_height;

        // Camera
        center = lookfrom;
        const double theta = degrees_to_radians(vfov);
        const double h = std::tan(theta / 2);
        const double viewport_height = 2.0 * h * focus_dist;
        const double viewport_width = viewport_height * (double(img_width) / img_height);

        // viewport uv and delta uv
        w = unit_vector(lookfrom - lookat);
        u = unit_vector(cross(vup, w));
        v = cross(w, u);
        auto viewport_u = viewport_width * u;
        auto viewport_v = -viewport_height * v;
        delta_u = viewport_u / img_width;
        delta_v = viewport_v / img_height;

        // upper left corner
        auto uppper_left = center - focus_dist * w - viewport_u / 2 - viewport_v / 2;
        pixel00 = uppper_left + 0.5 * (delta_u + delta_v);

        // Calculate the camera defocus disk basis vectors.
        auto defocus_radius = focus_dist * std::tan(degrees_to_radians(defocus_angle / 2));
        defocus_disk_u = u * defocus_radius;
        defocus_disk_v = v * defocus_radius;

        // sampling
        pixel_samples_scales = 1.0 / samples_per_pixel;
    }

    ray get_ray(int i, int j) const
    {
        // Construct a camera ray originating from the origin and directed at randomly sampled
        // point around the pixel location i, j.

        auto offset = sample_square();
        auto pixel_sample = pixel00 + ((i + offset.x()) * delta_u) + ((j + offset.y()) * delta_v);

        auto ray_origin = (defocus_angle <= 0) ? center : defocus_disk_sample();
        auto ray_direction = pixel_sample - ray_origin;

        return ray(ray_origin, ray_direction);
    }

    vec3 sample_square() const
    {
        // Returns the vector to a random point in the [-.5,-.5]-[+.5,+.5] unit square.
        return vec3(random_double() - 0.5, random_double() - 0.5, 0);
    }

    point3 defocus_disk_sample() const
    {
        // Returns a random point in the camera defocus disk.
        auto p = vec3::random_in_unit_disk();
        return center + (p[0] * defocus_disk_u) + (p[1] * defocus_disk_v);
    }

    color ray_color(const ray& r, const hittable& world) const
    {
        if (random_double() <= terminated_posibility)
            return color(0, 0, 0);

        hit_record record;
        if (world.hit(r, interval(epsilon, infinity), record))
        {
            ray scattered;
            color attenuation;
            if (record.mat->scatter(r, record, attenuation, scattered))
                return attenuation * ray_color(scattered, world) / (1 - terminated_posibility);
            return color();
        }

        vec3 unit_direction = unit_vector(r.direction());
        auto a = 0.5 * (unit_direction.y() + 1.0);
        return (1.0 - a) * color(1.0, 1.0, 1.0) + a * color(0.5, 0.7, 1.0);
    }
};
