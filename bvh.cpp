#include "bvh.h"

void BVH::update_node_bounds(int node_idx) {
    BVHNode& node = nodes[node_idx];
    node.bounds.reset();

    for (int i = 0; i < node.primitive_count; i++) {
        const triangle& tri = (*primitives_ref)[primitive_indices[node.left_first + i]];
        node.bounds.expand(tri.v0);
        node.bounds.expand(tri.v1);
        node.bounds.expand(tri.v2);
    }
}

void BVH::subdivide(int node_idx) {
    BVHNode& node = nodes[node_idx];

    if (node.primitive_count <= 2) return;

    int split_axis = 0;
    int left_count = splitter->split(node, primitive_indices, *primitives_ref, split_axis);

    if (left_count == 0 || left_count == node.primitive_count) return;

    int left_child_idx = nodes.size();
    nodes.push_back(BVHNode());
    int right_child_idx = nodes.size();
    nodes.push_back(BVHNode());

    nodes[left_child_idx].left_first = nodes[node_idx].left_first;
    nodes[left_child_idx].primitive_count = left_count;
    update_node_bounds(left_child_idx);

    nodes[right_child_idx].left_first = nodes[node_idx].left_first + left_count;
    nodes[right_child_idx].primitive_count = nodes[node_idx].primitive_count - left_count;
    update_node_bounds(right_child_idx);

    nodes[node_idx].left_first = left_child_idx;
    nodes[node_idx].primitive_count = 0; // 内部ノード化
    nodes[node_idx].split_axis = split_axis;

    subdivide(left_child_idx);
    subdivide(right_child_idx);
}

// Möller-Trumbore intersection algorithm
bool BVH::intersect_triangle(const ray& r, const triangle& tri, double& t_hit) const {
    vec3 edge1 = {tri.v1.x - tri.v0.x, tri.v1.y - tri.v0.y, tri.v1.z - tri.v0.z};
    vec3 edge2 = {tri.v2.x - tri.v0.x, tri.v2.y - tri.v0.y, tri.v2.z - tri.v0.z};
    vec3 h = {r.direction.y * edge2.z - r.direction.z * edge2.y,
              r.direction.z * edge2.x - r.direction.x * edge2.z,
              r.direction.x * edge2.y - r.direction.y * edge2.x};
    double a = edge1.x * h.x + edge1.y * h.y + edge1.z * h.z;

    if (a > -EPSILON && a < EPSILON) return false;

    double f = 1.0 / a;
    vec3 s = {r.origin.x - tri.v0.x, r.origin.y - tri.v0.y, r.origin.z - tri.v0.z};
    double u = f * (s.x * h.x + s.y * h.y + s.z * h.z);
    if (u < 0.0 || u > 1.0) return false;

    vec3 q = {s.y * edge1.z - s.z * edge1.y,
              s.z * edge1.x - s.x * edge1.z,
              s.x * edge1.y - s.y * edge1.x};
    double v = f * (r.direction.x * q.x + r.direction.y * q.y + r.direction.z * q.z);
    if (v < 0.0 || u + v > 1.0) return false;

    double t = f * (edge2.x * q.x + edge2.y * q.y + edge2.z * q.z);
    if (t > EPSILON) {
        t_hit = t;
        return true;
    }
    return false;
}

BVH::BVH(std::unique_ptr<SplitStrategy> strategy) 
    : primitives_ref(nullptr), splitter(std::move(strategy)) {}

void BVH::build(const std::vector<triangle>& triangles) {
    primitives_ref = &triangles;
    int n = triangles.size();
    
    primitive_indices.resize(n);
    for (int i = 0; i < n; i++) {
        primitive_indices[i] = i;
    }

    nodes.clear();
    nodes.push_back(BVHNode());
    nodes[0].left_first = 0;
    nodes[0].primitive_count = n;

    update_node_bounds(0);
    subdivide(0);
}

bool BVH::intersect(const ray& r, hitRecord& rec, traverseStats& stats) const {
    bool hit_anything = false;
    rec.t = std::numeric_limits<double>::infinity();
    rec.triangle_ptr = nullptr;

    std::vector<int> stack;
    stack.reserve(INITIAL_STACK_CAPACITY);
    stack.push_back(0);

    // 探索木をDFS
    while (!stack.empty()) {
        int node_idx = stack.back();
        stack.pop_back();
        
        const BVHNode& node = nodes[node_idx];

        stats.aabb_tests++;
        double t_near, t_far;
        if (!node.bounds.intersect(r, t_near, t_far)) continue;
        
        if (t_near >= rec.t) continue;

        if (node.is_leaf()) {
            for (int i = 0; i < node.primitive_count; i++) {
                int prim_idx = primitive_indices[node.left_first + i];
                const triangle& tri = (*primitives_ref)[prim_idx];
                
                stats.triangle_tests++;
                double t_hit;
                if (intersect_triangle(r, tri, t_hit)) {
                    if (t_hit < rec.t) {
                        rec.t = t_hit;
                        rec.triangle_ptr = &tri;
                        hit_anything = true;
                    }
                }
            }
        } 
        else {
            int child1 = node.left_first;
            int child2 = child1 + 1;

            bool is_dir_positive = true;
            if (node.split_axis == 1) {
                is_dir_positive = r.direction.x > 0;
            }
            else if (node.split_axis == 2) {
                is_dir_positive = r.direction.y > 0;
            }
            else if (node.split_axis == 3) {
                is_dir_positive = r.direction.z > 0;
            }

            if (is_dir_positive) {
                stack.push_back(child2);
                stack.push_back(child1);
            } 
            else {
                stack.push_back(child1);
                stack.push_back(child2);
            }
        }
    }

    return hit_anything;
}
