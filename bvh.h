#pragma once

#include "core.h"
#include "splitStrategy.h"
#include <vector>
#include <memory>

class BVH : public ADS {
private:
    std::vector<BVHNode> nodes;
    std::vector<int> primitive_indices;
    const std::vector<triangle>* primitives_ref;
    
    std::unique_ptr<SplitStrategy> splitter;

    void update_node_bounds(int node_idx);
    void subdivide(int node_idx);
    bool intersect_triangle(const ray& r, const triangle& tri, double& t_hit) const;

public:
    BVH(std::unique_ptr<SplitStrategy> strategy);

    void build(const std::vector<triangle>& triangles) override;
    bool intersect(const ray& r, hitRecord& rec, traverseStats& stats) const override;
};
