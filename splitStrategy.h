#pragma once

#include "core.h"

// 1. 空間中央分割 (Spatial Median)
class SpatialMedianStrategy : public SplitStrategy {
public:
    int split(BVHNode& node, std::vector<int>& indices, const std::vector<triangle>& primitives, int& out_split_axis) override;
};

// 2. オブジェクト中央分割 (Object Median)
class ObjectMedianStrategy : public SplitStrategy {
public:
    int split(BVHNode& node, std::vector<int>& indices, const std::vector<triangle>& primitives, int& out_split_axis) override;
};

// 3. 表面積ヒューリスティック (Binned SAH)
class SAHStrategy : public SplitStrategy {
    static constexpr int NUM_BINS = 16;
public:
    int split(BVHNode& node, std::vector<int>& indices, const std::vector<triangle>& primitives, int& out_split_axis) override;
};

// 4. Random Strategy
class RandomStrategy : public SplitStrategy {
public:
    int split(BVHNode& node, std::vector<int>& indices, const std::vector<triangle>& primitives, int& out_split_axis) override;
};

// 5. SAH Alternative (with variable K)
class SAH_alt : public SplitStrategy {
    static constexpr int NUM_BINS = 16;
    double K;
public:
    SAH_alt(double k) : K(k) {}
    int split(BVHNode& node, std::vector<int>& indices, const std::vector<triangle>& primitives, int& out_split_axis) override;
};
