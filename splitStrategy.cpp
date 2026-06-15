#include "splitStrategy.h"

// 1. 空間中央分割 (Spatial Median)
int SpatialMedianStrategy::split(BVHNode& node, std::vector<int>& indices, const std::vector<triangle>& primitives, int& out_split_axis) {
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

// 2. オブジェクト中央分割 (Object Median)
int ObjectMedianStrategy::split(BVHNode& node, std::vector<int>& indices, const std::vector<triangle>& primitives, int& out_split_axis) {
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

// 3. 表面積ヒューリスティック (Binned SAH)
int SAHStrategy::split(BVHNode& node, std::vector<int>& indices, const std::vector<triangle>& primitives, int& out_split_axis) {
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


// 4. Random Strategy
int RandomStrategy::split(BVHNode& node, std::vector<int>& indices, const std::vector<triangle>& primitives, int& out_split_axis) {
    out_split_axis = node.bounds.longest_axis();
    
    auto start = indices.begin() + node.left_first;
    auto end = start + node.primitive_count;

    // ランダムな要素をピボットとして選択
    static std::mt19937 rng(std::random_device{}());
    std::uniform_int_distribution<int> dist(0, node.primitive_count - 1);
    int pivot_idx = *(start + dist(rng));
    double pivot_pos = get_centroid(primitives[pivot_idx], out_split_axis);

    auto it = std::partition(start, end, [&](int idx) {
        return get_centroid(primitives[idx], out_split_axis) < pivot_pos;
    });

    int left_count = std::distance(start, it);

    // 全要素がピボットと同じ座標にあった場合などの安全装置
    if (left_count == 0 || left_count == node.primitive_count) {
        left_count = node.primitive_count / 2;
    }
    return left_count;
}

// 5. SAH Alternative (with variable K)
int SAH_alt::split(BVHNode& node, std::vector<int>& indices, const std::vector<triangle>& primitives, int& out_split_axis) {
    double min_cost = std::numeric_limits<double>::infinity();
    int best_axis = -1;
    int best_split_index = -1;
    
    AABB centroid_bounds;
    centroid_bounds.reset();
    for (int i = 0; i < node.primitive_count; i++) {
        int prim_idx = indices[node.left_first + i];
        point c = {get_centroid(primitives[prim_idx], 1), get_centroid(primitives[prim_idx], 2), get_centroid(primitives[prim_idx], 3)};
        centroid_bounds.expand(c);
    }

    for (int axis = 1; axis <= 3; axis++) {
        struct Bin {
            AABB bounds;
            int count = 0;
            Bin() { bounds.reset(); }
        } bins[NUM_BINS];

        double min_c = (axis == 1) ? centroid_bounds.min.x : ((axis == 2) ? centroid_bounds.min.y : centroid_bounds.min.z);
        double max_c = (axis == 1) ? centroid_bounds.max.x : ((axis == 2) ? centroid_bounds.max.y : centroid_bounds.max.z);
        
        if (max_c == min_c) continue; 

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

        AABB current_right_bounds;
        current_right_bounds.reset();
        int current_right_count = 0;
        for (int i = NUM_BINS - 1; i > 0; i--) {
            current_right_bounds.merge(bins[i].bounds);
            current_right_count += bins[i].count;
            
            // 提案された 1+k 乗モデルのコスト計算
            double cost = left_bounds[i-1].surface_area() * std::pow(left_counts[i-1], 1.0 + K) + 
                          current_right_bounds.surface_area() * std::pow(current_right_count, 1.0 + K);
            
            if (cost < min_cost) {
                min_cost = cost;
                best_axis = axis;
                best_split_index = i;
            }
        }
    }

    if (best_axis == -1) {
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
    
    if (left_count == 0 || left_count == node.primitive_count) {
        left_count = node.primitive_count / 2;
        std::nth_element(start, start + left_count, end, [&](int a, int b) {
            return get_centroid(primitives[a], best_axis) < get_centroid(primitives[b], best_axis);
        });
    }
    return left_count;
}
