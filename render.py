import ctypes
import math
import time
import os
import sys

# ---------------------------------------------------------
# .obj ファイルの読み込み
# ---------------------------------------------------------
def load_obj(filename):
    print(f"Loading {filename} ...")
    vertices = []
    triangles = []
    
    with open(filename, 'r') as f:
        for line in f:
            if line.startswith('v '):
                parts = line.split()
                vertices.append((float(parts[1]), float(parts[2]), float(parts[3])))
            elif line.startswith('f '):
                parts = line.split()
                # f v1/vt1/vn1 v2/vt2/vn2 v3/vt3/vn3 ... の形式に対応
                def parse_idx(p):
                    i = int(p.split('/')[0])
                    return len(vertices) + i if i < 0 else i - 1

                if len(parts) >= 4:
                    idx0 = parse_idx(parts[1])
                    for i in range(2, len(parts) - 1):
                        idx1 = parse_idx(parts[i])
                        idx2 = parse_idx(parts[i+1])
                        triangles.append((idx0, idx1, idx2))
                
    # AABBの計算（カメラの自動配置用）
    min_x = min_y = min_z = float('inf')
    max_x = max_y = max_z = float('-inf')
    for v in vertices:
        min_x, max_x = min(min_x, v[0]), max(max_x, v[0])
        min_y, max_y = min(min_y, v[1]), max(max_y, v[1])
        min_z, max_z = min(min_z, v[2]), max(max_z, v[2])
        
    center = ((min_x + max_x) / 2, (min_y + max_y) / 2, (min_z + max_z) / 2)
    size = max(max_x - min_x, max_y - min_y, max_z - min_z)
                
    # C++ に渡すためのフラットな配列を作成
    flat_triangles = []
    for t in triangles:
        v0, v1, v2 = vertices[t[0]], vertices[t[1]], vertices[t[2]]
        flat_triangles.extend(v0)
        flat_triangles.extend(v1)
        flat_triangles.extend(v2)
        
    print(f"Loaded {len(triangles)} triangles.")
    return flat_triangles, len(triangles), center, size


# ---------------------------------------------------------
# カメラ設定とレイの生成
# ---------------------------------------------------------
def normalize(v):
    l = math.sqrt(v[0]*v[0] + v[1]*v[1] + v[2]*v[2])
    if l == 0: return (0, 0, 0)
    return (v[0]/l, v[1]/l, v[2]/l)

def cross(v1, v2):
    return (
        v1[1]*v2[2] - v1[2]*v2[1],
        v1[2]*v2[0] - v1[0]*v2[2],
        v1[0]*v2[1] - v1[1]*v2[0]
    )

def generate_rays(width, height, center, size):
    print("Generating rays...")
    rays = []
    
    # カメラはZ軸方向の少し手前に配置し、中心を見る
    cam_pos = (center[0], center[1], center[2] + size * 1.5)
    look_at = center
    up = (0, 1, 0)
    
    w = normalize((cam_pos[0]-look_at[0], cam_pos[1]-look_at[1], cam_pos[2]-look_at[2]))
    u = normalize(cross(up, w))
    v = cross(w, u)
    
    fov = 60.0
    aspect_ratio = width / height
    viewport_height = 2.0 * math.tan(math.radians(fov) / 2.0)
    viewport_width = aspect_ratio * viewport_height
    
    horizontal = (u[0]*viewport_width, u[1]*viewport_width, u[2]*viewport_width)
    vertical = (v[0]*viewport_height, v[1]*viewport_height, v[2]*viewport_height)
    
    lower_left_corner = (
        cam_pos[0] - horizontal[0]/2 - vertical[0]/2 - w[0],
        cam_pos[1] - horizontal[1]/2 - vertical[1]/2 - w[1],
        cam_pos[2] - horizontal[2]/2 - vertical[2]/2 - w[2]
    )
    
    for y in range(height):
        # 画像の上から下へ
        v_coord = (height - 1 - y) / (height - 1)
        for x in range(width):
            u_coord = x / (width - 1)
            dir_x = lower_left_corner[0] + u_coord*horizontal[0] + v_coord*vertical[0] - cam_pos[0]
            dir_y = lower_left_corner[1] + u_coord*horizontal[1] + v_coord*vertical[1] - cam_pos[1]
            dir_z = lower_left_corner[2] + u_coord*horizontal[2] + v_coord*vertical[2] - cam_pos[2]
            
            d = normalize((dir_x, dir_y, dir_z))
            rays.extend([cam_pos[0], cam_pos[1], cam_pos[2], d[0], d[1], d[2]])
            
    return rays

# ---------------------------------------------------------
# メイン処理
# ---------------------------------------------------------
def main():
    if len(sys.argv) < 2:
        print("Usage: python render.py <path_to_obj>")
        sys.exit(1)
        
    obj_path = sys.argv[1]
    
    # 共有ライブラリのロード
    lib_ext = '.dylib' if sys.platform == 'darwin' else '.so'
    lib_path = os.path.join(os.path.dirname(__file__), 'libbvh' + lib_ext)
    bvh_lib = ctypes.CDLL(lib_path)
    
    bvh_lib.build_bvh.argtypes = [ctypes.POINTER(ctypes.c_double), ctypes.c_int, ctypes.c_int]
    bvh_lib.build_bvh.restype = ctypes.c_void_p
    
    bvh_lib.intersect_rays.argtypes = [ctypes.c_void_p, ctypes.POINTER(ctypes.c_double), ctypes.c_int, ctypes.POINTER(ctypes.c_double), ctypes.POINTER(ctypes.c_longlong)]
    bvh_lib.intersect_rays.restype = None
    
    bvh_lib.destroy_bvh.argtypes = [ctypes.c_void_p]
    bvh_lib.destroy_bvh.restype = None

    # OBJの読み込み
    flat_triangles, num_triangles, center, size = load_obj(obj_path)
    
    # C++ 配列への変換
    TriArrayType = ctypes.c_double * len(flat_triangles)
    c_triangles = TriArrayType(*flat_triangles)
    
    # BVHの構築
    # strategy_type: 1=Spatial, 2=Object, 3=SAH
    strategy_type = 3
    print(f"Building BVH (Strategy: {strategy_type})...")
    start_time = time.perf_counter()
    bvh_ptr = bvh_lib.build_bvh(c_triangles, num_triangles, strategy_type)
    build_time = time.perf_counter() - start_time
    print(f"BVH Build Time: {build_time:.4f} seconds")
    
    # レイの生成
    width, height = 512, 512
    rays = generate_rays(width, height, center, size)
    num_rays = width * height
    
    RayArrayType = ctypes.c_double * len(rays)
    c_rays = RayArrayType(*rays)
    
    HitResultArrayType = ctypes.c_double * num_rays
    c_hits = HitResultArrayType()
    
    StatsArrayType = ctypes.c_longlong * 2
    c_stats = StatsArrayType()
    
    # レンダリング（交差判定の一括処理）
    print(f"Intersecting {num_rays} rays...")
    start_time = time.perf_counter()
    bvh_lib.intersect_rays(bvh_ptr, c_rays, num_rays, c_hits, c_stats)
    render_time = time.perf_counter() - start_time
    
    # 統計情報の出力
    print("-" * 30)
    print("=== Render Statistics ===")
    print(f"Resolution     : {width} x {height}")
    print(f"Render Time    : {render_time:.4f} seconds")
    print(f"Total Rays     : {num_rays}")
    print(f"AABB Tests     : {c_stats[0]}")
    print(f"Triangle Tests : {c_stats[1]}")
    print(f"Avg AABB/Ray   : {c_stats[0]/num_rays:.1f}")
    print(f"Avg Tri/Ray    : {c_stats[1]/num_rays:.1f}")
    print("-" * 30)
    
    # BVHの破棄
    bvh_lib.destroy_bvh(bvh_ptr)
    
    # 結果の画像出力 (PPMフォーマットで深度マップとして出力)
    obj_name = os.path.splitext(os.path.basename(obj_path))[0]
    out_filename = f"result/{obj_name}.ppm"
    
    print(f"Saving image to {out_filename} ...")
    max_t = 0.0
    min_t = float('inf')
    for i in range(num_rays):
        if c_hits[i] > 0.0 and c_hits[i] < float('inf'):
            max_t = max(max_t, c_hits[i])
            min_t = min(min_t, c_hits[i])
            
    with open(out_filename, "w") as f:
        f.write(f"P3\n{width} {height}\n255\n")
        for i in range(num_rays):
            t = c_hits[i]
            if t <= 0.0 or t == float('inf'):
                # 背景色 (黒)
                f.write("0 0 0\n")
            else:
                # 深度に応じた色 (近いほど白、遠いほど暗い)
                if max_t > min_t:
                    val = 1.0 - ((t - min_t) / (max_t - min_t))
                else:
                    val = 1.0
                val = max(0.0, min(1.0, val))
                c = int(val * 255)
                f.write(f"{c} {c} {c}\n")

if __name__ == "__main__":
    main()
