#include "core.h"
#include "bvh.h"
#include "splitStrategy.h"
#include <vector>
#include <memory>

struct BVHWrapper {
    std::vector<triangle> triangles;
    std::unique_ptr<BVH> bvh;
};

extern "C" {
    void* build_bvh(const double* vertices, int num_triangles, int strategy_type) {
        BVHWrapper* wrapper = new BVHWrapper();
        wrapper->triangles.reserve(num_triangles);
        for (int i = 0; i < num_triangles; i++) {
            triangle t;
            t.v0.x = vertices[i * 9 + 0]; t.v0.y = vertices[i * 9 + 1]; t.v0.z = vertices[i * 9 + 2];
            t.v1.x = vertices[i * 9 + 3]; t.v1.y = vertices[i * 9 + 4]; t.v1.z = vertices[i * 9 + 5];
            t.v2.x = vertices[i * 9 + 6]; t.v2.y = vertices[i * 9 + 7]; t.v2.z = vertices[i * 9 + 8];
            wrapper->triangles.push_back(t);
        }

        std::unique_ptr<SplitStrategy> strategy;
        if (strategy_type == 1) strategy = std::make_unique<SpatialMedianStrategy>();
        else if (strategy_type == 2) strategy = std::make_unique<ObjectMedianStrategy>();
        else if (strategy_type == 3) strategy = std::make_unique<SAHStrategy>();
        else if (strategy_type == 4) strategy = std::make_unique<RandomStrategy>();
        else if (strategy_type == 5) strategy = std::make_unique<SAH_alt>(-0.2);
        else strategy = std::make_unique<SAH_alt>(0.2);

        wrapper->bvh = std::make_unique<BVH>(std::move(strategy));
        wrapper->bvh->build(wrapper->triangles);

        return wrapper;
    }

    void destroy_bvh(void* ptr) {
        if (ptr) {
            delete static_cast<BVHWrapper*>(ptr);
        }
    }

    void intersect_rays(void* ptr, const double* rays_data, int num_rays, double* hit_results, long long* out_stats) {
        BVHWrapper* wrapper = static_cast<BVHWrapper*>(ptr);
        long long aabb_tests = 0;
        long long tri_tests = 0;

        for (int i = 0; i < num_rays; i++) {
            point origin = {rays_data[i*6 + 0], rays_data[i*6 + 1], rays_data[i*6 + 2]};
            vec3 direction = {rays_data[i*6 + 3], rays_data[i*6 + 4], rays_data[i*6 + 5]};
            ray r(origin, direction);

            hitRecord rec;
            traverseStats stats;
            bool hit = wrapper->bvh->intersect(r, rec, stats);

            aabb_tests += stats.aabb_tests;
            tri_tests += stats.triangle_tests;

            if (hit) {
                hit_results[i] = rec.t;
            } 
            else {
                hit_results[i] = -1.0;
            }
        }
        
        out_stats[0] = aabb_tests;
        out_stats[1] = tri_tests;
    }
}
