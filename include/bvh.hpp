#pragma once

#include "vec3.hpp"
#include "hittable.hpp"
#include "AABB.hpp"

struct bvh : public hittable
{
    bvh() = default;
    bvh(const std::shared_ptr<hittable>& left, const std::shared_ptr<hittable>& right) :
        left(left),
        right(right),
        aabb(AABB::combine(left->get_AABB(), right->get_AABB())) {};
    bvh(const std::vector<std::shared_ptr<hittable>>& objects) {
        if (objects.empty()) {
            left = right = nullptr;
            aabb = AABB();
            return;
        }
        if (objects.size() == 1) {
            left = objects[0];
            right = nullptr;
            aabb = left->get_AABB();
            return;
        }

        AABB total_aabb = objects[0]->get_AABB();
        for (size_t i = 1; i < objects.size(); ++i) {
            total_aabb = AABB::combine(total_aabb, objects[i]->get_AABB());
        }

        vec3 extent = total_aabb.max - total_aabb.min;
        int axis = 0;
        if (extent.y() > extent.x()) axis = 1;
        if (extent.z() > extent[axis]) axis = 2;

        std::vector<std::shared_ptr<hittable>> sorted_objects = objects;
        std::sort(sorted_objects.begin(), sorted_objects.end(),
            [axis](const std::shared_ptr<hittable>& a, const std::shared_ptr<hittable>& b) {
                AABB aabb_a = a->get_AABB();
                AABB aabb_b = b->get_AABB();
                double center_a = (aabb_a.min[axis] + aabb_a.max[axis]) / 2.0;
                double center_b = (aabb_b.min[axis] + aabb_b.max[axis]) / 2.0;
                return center_a < center_b;
            });

        size_t mid = sorted_objects.size() / 2;
        std::vector<std::shared_ptr<hittable>> left_objects(sorted_objects.begin(), sorted_objects.begin() + mid);
        std::vector<std::shared_ptr<hittable>> right_objects(sorted_objects.begin() + mid, sorted_objects.end());

        left = std::make_shared<bvh>(left_objects);
        right = std::make_shared<bvh>(right_objects);
        aabb = AABB::combine(left->get_AABB(), right->get_AABB());
    }
    std::shared_ptr<hittable> left;
    std::shared_ptr<hittable> right;
    AABB aabb;

    virtual bool hit(const ray& r, interval ray_t, hit_record& record) const override {
        double tmin = ray_t.min;
        double tmax = ray_t.max;
        const auto& min = aabb.min;
        const auto& max = aabb.max;

        for (int i = 0; i < 3; i++) {
            double invD = 1.0 / r.direction()[i];
            double t0 = (min[i] - r.origin()[i]) * invD;
            double t1 = (max[i] - r.origin()[i]) * invD;

            if (invD < 0.0) {
                std::swap(t0, t1);
            }

            tmin = t0 > tmin ? t0 : tmin;
            tmax = t1 < tmax ? t1 : tmax;

            if (tmax <= tmin) {
                return false;
            }
        }

        std::vector<hit_record> hits;
        hit_record rec;
        if (left && left->hit(r, ray_t, rec)) {
            hits.emplace_back(std::move(rec));
        }
        if (right && right->hit(r, ray_t, rec)) {
            hits.emplace_back(std::move(rec));
        }

        if (hits.empty()){
            return false;
        } else {
            record = *std::min_element(hits.begin(), hits.end(), [](const hit_record& a, const hit_record& b) {
                return a.t < b.t;
            });
            return true;
        }
    }

    virtual AABB get_AABB() const override {
        return this->aabb;
    }
};
