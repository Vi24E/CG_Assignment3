#include <iostream>
#include <vector>
#include <cmath>
#include <limits>
#include <cassert>
#include <memory>
#include <algorithm>

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
};

struct BVHNode {
    AABB bounds;
    int left_first; 
    int primitive_count; 
	// 0: 葉ノード/未定義, 1: X軸, 2: Y軸, 3: Z軸
    char split_axis = 0;

    bool is_leaf() const {return primitive_count > 0;}
};

// 分割戦略
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

class BVH : public ADS {
private:
    std::vector<BVHNode> nodes;
    std::vector<int> primitive_indices;
    const std::vector<triangle>* primitives_ref;
    
    std::unique_ptr<SplitStrategy> splitter;

    void update_node_bounds(int node_idx) {
        BVHNode& node = nodes[node_idx];
        node.bounds.reset();

        for (int i = 0; i < node.primitive_count; i++) {
            const triangle& tri = (*primitives_ref)[primitive_indices[node.left_first + i]];
            node.bounds.expand(tri.v0);
            node.bounds.expand(tri.v1);
            node.bounds.expand(tri.v2);
        }
    }

    void subdivide(int node_idx) {
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
    bool intersect_triangle(const ray& r, const triangle& tri, double& t_hit) const {
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

public:
    BVH(std::unique_ptr<SplitStrategy> strategy) 
        : primitives_ref(nullptr), splitter(std::move(strategy)) {}

    void build(const std::vector<triangle>& triangles) override {
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

    bool intersect(const ray& r, hitRecord& rec, traverseStats& stats) const override {
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
};

// 1. 空間中央分割 (Spatial Median)
class SpatialMedianStrategy : public SplitStrategy {
public:
    int split(BVHNode& node, std::vector<int>& indices, const std::vector<triangle>& primitives, int& out_split_axis) override {
        out_split_axis = node.bounds.longest_axis();
        double split_pos;
        if (out_split_axis == 1) split_pos = (node.bounds.min.x + node.bounds.max.x) * 0.5;
        else if (out_split_axis == 2) split_pos = (node.bounds.min.y + node.bounds.max.y) * 0.5;
        else split_pos = (node.bounds.min.z + node.bounds.max.z) * 0.5;

        auto start = indices.begin() + node.left_first;
        auto end = start + node.primitive_count;
        
        auto it = std::partition(start, end, [&](int idx) {
            return get_centroid(primitives[idx], out_split_axis) < split_pos;
        });
        
        int left_count = std::distance(start, it);
        // 片方に全て寄ってしまった場合は要素数の中央で分割するフォールバック
        if (left_count == 0 || left_count == node.primitive_count) {
            left_count = node.primitive_count / 2;
        }
        return left_count;
    }
};

// 2. オブジェクト中央分割 (Object Median)
class ObjectMedianStrategy : public SplitStrategy {
public:
    int split(BVHNode& node, std::vector<int>& indices, const std::vector<triangle>& primitives, int& out_split_axis) override {
        out_split_axis = node.bounds.longest_axis();
        int mid = node.primitive_count / 2;
        auto start = indices.begin() + node.left_first;
        auto end = start + node.primitive_count;
        
        // $O(N)$ のIntroselectを用いて中央値を境界に分割
        std::nth_element(start, start + mid, end, [&](int a, int b) {
            return get_centroid(primitives[a], out_split_axis) < get_centroid(primitives[b], out_split_axis);
        });
        
        return mid;
    }
};

// 3. 表面積ヒューリスティック (Binned SAH)
class SAHStrategy : public SplitStrategy {
    static constexpr int NUM_BINS = 16;
public:
    int split(BVHNode& node, std::vector<int>& indices, const std::vector<triangle>& primitives, int& out_split_axis) override {
        double min_cost = std::numeric_limits<double>::infinity();
        int best_axis = -1;
        int best_split_index = -1;
        
        // 分割候補を決めるための重心のAABBを算出
        AABB centroid_bounds;
        centroid_bounds.reset();
        for (int i = 0; i < node.primitive_count; i++) {
            int prim_idx = indices[node.left_first + i];
            point c = {get_centroid(primitives[prim_idx], 1), get_centroid(primitives[prim_idx], 2), get_centroid(primitives[prim_idx], 3)};
            centroid_bounds.expand(c);
        }

        // X, Y, Z 全軸に対してSAHコストを評価
        for (int axis = 1; axis <= 3; axis++) {
            struct Bin {
                AABB bounds;
                int count = 0;
                Bin() { bounds.reset(); }
            } bins[NUM_BINS];

            double min_c = (axis == 1) ? centroid_bounds.min.x : ((axis == 2) ? centroid_bounds.min.y : centroid_bounds.min.z);
            double max_c = (axis == 1) ? centroid_bounds.max.x : ((axis == 2) ? centroid_bounds.max.y : centroid_bounds.max.z);
            
            if (max_c == min_c) continue; 

            // 各プリミティブをビンに割り当て
            for (int i = 0; i < node.primitive_count; i++) {
                int prim_idx = indices[node.left_first + i];
                const triangle& tri = primitives[prim_idx];
                double c = get_centroid(tri, axis);
                int bin_idx = NUM_BINS * ((c - min_c) / (max_c - min_c));
                if (bin_idx == NUM_BINS) bin_idx = NUM_BINS - 1;

                bins[bin_idx].count++;
                bins[bin_idx].bounds.expand(tri.v0);
                bins[bin_idx].bounds.expand(tri.v1);
                bins[bin_idx].bounds.expand(tri.v2);
            }

            // 左側への蓄積
            AABB left_bounds[NUM_BINS - 1];
            int left_counts[NUM_BINS - 1];
            AABB current_left_bounds;
            current_left_bounds.reset();
            int current_left_count = 0;
            for (int i = 0; i < NUM_BINS - 1; i++) {
                current_left_bounds.merge(bins[i].bounds);
                current_left_count += bins[i].count;
                left_bounds[i] = current_left_bounds;
                left_counts[i] = current_left_count;
            }

            // 右側からの蓄積とコスト評価
            AABB current_right_bounds;
            current_right_bounds.reset();
            int current_right_count = 0;
            for (int i = NUM_BINS - 1; i > 0; i--) {
                current_right_bounds.merge(bins[i].bounds);
                current_right_count += bins[i].count;
                
                double cost = left_bounds[i-1].surface_area() * left_counts[i-1] + current_right_bounds.surface_area() * current_right_count;
                if (cost < min_cost) {
                    min_cost = cost;
                    best_axis = axis;
                    best_split_index = i;
                }
            }
        }

        // 葉ノードのままでいるコスト (Traverseコストは単純化して無視)
        double leaf_cost = node.bounds.surface_area() * node.primitive_count;
        
        // どの軸でもコストが改善しない場合
        if (best_axis == -1 || min_cost >= leaf_cost) {
            return 0; 
        }

        out_split_axis = best_axis;
        double min_c = (best_axis == 1) ? centroid_bounds.min.x : ((best_axis == 2) ? centroid_bounds.min.y : centroid_bounds.min.z);
        double max_c = (best_axis == 1) ? centroid_bounds.max.x : ((best_axis == 2) ? centroid_bounds.max.y : centroid_bounds.max.z);

        auto start = indices.begin() + node.left_first;
        auto end = start + node.primitive_count;
        auto it = std::partition(start, end, [&](int idx) {
            double c = get_centroid(primitives[idx], best_axis);
            int bin_idx = NUM_BINS * ((c - min_c) / (max_c - min_c));
            if (bin_idx == NUM_BINS) bin_idx = NUM_BINS - 1;
            return bin_idx < best_split_index;
        });

        int left_count = std::distance(start, it);
        
        // 精度等の問題で分割が偏った場合の安全装置
        if (left_count == 0 || left_count == node.primitive_count) {
            left_count = node.primitive_count / 2;
            std::nth_element(start, start + left_count, end, [&](int a, int b) {
                return get_centroid(primitives[a], best_axis) < get_centroid(primitives[b], best_axis);
            });
        }
        return left_count;
    }
};