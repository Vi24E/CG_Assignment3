#pragma once

#include <iostream>
#include <vector>
#include <cmath>
#include <limits>
#include <cassert>
#include <memory>
#include <algorithm>
#include <random>

constexpr double EPSILON = 1e-8;
constexpr int INITIAL_STACK_CAPACITY = 64;

struct point {
    double x, y, z;
};

struct vec3 {
    double x, y, z;
};

struct triangle {
    point v0, v1, v2;
};

struct ray {
    point origin;
    vec3 direction;

    ray(point o, vec3 d) : origin(o) {
        double len = std::hypot(d.x, d.y, d.z); 
        assert(len > 0);
        direction.x = d.x / len;
        direction.y = d.y / len;
        direction.z = d.z / len;
    }
};

inline double get_centroid(const triangle& tri, int axis) {
    if (axis == 1) return (tri.v0.x + tri.v1.x + tri.v2.x) / 3.0;
    if (axis == 2) return (tri.v0.y + tri.v1.y + tri.v2.y) / 3.0;
    return (tri.v0.z + tri.v1.z + tri.v2.z) / 3.0;
}

inline double get_triangle_area(const triangle& tri) {
    vec3 edge1 = {tri.v1.x - tri.v0.x, tri.v1.y - tri.v0.y, tri.v1.z - tri.v0.z};
    vec3 edge2 = {tri.v2.x - tri.v0.x, tri.v2.y - tri.v0.y, tri.v2.z - tri.v0.z};
    vec3 cross = {
        edge1.y * edge2.z - edge1.z * edge2.y,
        edge1.z * edge2.x - edge1.x * edge2.z,
        edge1.x * edge2.y - edge1.y * edge2.x
    };
    return 0.5 * std::hypot(cross.x, cross.y, cross.z);
}

// レイトレーシングで本当に必要な交差情報
struct hitRecord {
    double t;
    // 安全性とConst正確性を保つため、constポインタを使用
    const triangle* triangle_ptr; 
};

// パフォーマンス測定用の統計情報
struct traverseStats {
    int aabb_tests = 0;
    int triangle_tests = 0;
};

// インターフェース
class ADS {
public:
    virtual ~ADS() = default;

    virtual void build(const std::vector<triangle>& triangles) = 0;
    virtual bool intersect(const ray& r, hitRecord& rec, traverseStats& stats) const = 0;
};

struct AABB {
    point min, max;

    void reset() {
        min = {std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity()};
        max = {-std::numeric_limits<double>::infinity(), -std::numeric_limits<double>::infinity(), -std::numeric_limits<double>::infinity()};
    }

    void expand(const point& p) {
        min.x = std::min(min.x, p.x); min.y = std::min(min.y, p.y); min.z = std::min(min.z, p.z);
        max.x = std::max(max.x, p.x); max.y = std::max(max.y, p.y); max.z = std::max(max.z, p.z);
    }

    bool intersect(const ray& r, double& t_near, double& t_far) const {
        double tx1 = (min.x - r.origin.x) / r.direction.x;
        double tx2 = (max.x - r.origin.x) / r.direction.x;
        double tmin = std::min(tx1, tx2);
        double tmax = std::max(tx1, tx2);

        double ty1 = (min.y - r.origin.y) / r.direction.y;
        double ty2 = (max.y - r.origin.y) / r.direction.y;
        tmin = std::max(tmin, std::min(ty1, ty2));
        tmax = std::min(tmax, std::max(ty1, ty2));

        double tz1 = (min.z - r.origin.z) / r.direction.z;
        double tz2 = (max.z - r.origin.z) / r.direction.z;
        tmin = std::max(tmin, std::min(tz1, tz2));
        tmax = std::min(tmax, std::max(tz1, tz2));

        t_near = tmin;
        t_far = tmax;
        return tmax >= tmin && tmax > 0.0;
    }

    int longest_axis() const {
        double dx = max.x - min.x;
        double dy = max.y - min.y;
        double dz = max.z - min.z;
        if (dx > dy && dx > dz) return 1;
        if (dy > dz) return 2;
        return 3;
    }

    void merge(const AABB& other) {
        min.x = std::min(min.x, other.min.x);
        min.y = std::min(min.y, other.min.y);
        min.z = std::min(min.z, other.min.z);
        max.x = std::max(max.x, other.max.x);
        max.y = std::max(max.y, other.max.y);
        max.z = std::max(max.z, other.max.z);
    }

    double surface_area() const {
        double dx = std::max(0.0, max.x - min.x);
        double dy = std::max(0.0, max.y - min.y);
        double dz = std::max(0.0, max.z - min.z);
        return 2.0 * (dx * dy + dy * dz + dz * dx);
    }
};

struct BVHNode {
    AABB bounds;
    int left_first; 
    int primitive_count; 
    // 0: 葉ノード/未定義, 1: X軸, 2: Y軸, 3: Z軸
    char split_axis = 0;

    bool is_leaf() const {return primitive_count > 0;}
};

// 分割戦略インターフェース
class SplitStrategy {
public:
    virtual ~SplitStrategy() = default;
    // 戻り値で左ノードの要素数を返し、out_split_axis に分割軸を設定する
    virtual int split(
        BVHNode& node, 
        std::vector<int>& indices, 
        const std::vector<triangle>& primitives,
        int& out_split_axis
    ) = 0;
};
