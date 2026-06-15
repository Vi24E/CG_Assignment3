import ctypes
import math
import time
import os
import glob

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
                def parse_idx(p):
                    i = int(p.split('/')[0])
                    return len(vertices) + i if i < 0 else i - 1

                if len(parts) >= 4:
                    idx0 = parse_idx(parts[1])
                    for i in range(2, len(parts) - 1):
                        idx1 = parse_idx(parts[i])
                        idx2 = parse_idx(parts[i+1])
                        triangles.append((idx0, idx1, idx2))
                
    min_x = min_y = min_z = float('inf')
    max_x = max_y = max_z = float('-inf')
    for v in vertices:
        min_x, max_x = min(min_x, v[0]), max(max_x, v[0])
        min_y, max_y = min(min_y, v[1]), max(max_y, v[1])
        min_z, max_z = min(min_z, v[2]), max(max_z, v[2])
        
    center = ((min_x + max_x) / 2, (min_y + max_y) / 2, (min_z + max_z) / 2)
    size = max(max_x - min_x, max_y - min_y, max_z - min_z)
                
    flat_triangles = []
    for t in triangles:
        v0, v1, v2 = vertices[t[0]], vertices[t[1]], vertices[t[2]]
        flat_triangles.extend(v0)
        flat_triangles.extend(v1)
        flat_triangles.extend(v2)
        
    return flat_triangles, len(triangles), center, size


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
    rays = []
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
        v_coord = (height - 1 - y) / (height - 1)
        for x in range(width):
            u_coord = x / (width - 1)
            dir_x = lower_left_corner[0] + u_coord*horizontal[0] + v_coord*vertical[0] - cam_pos[0]
            dir_y = lower_left_corner[1] + u_coord*horizontal[1] + v_coord*vertical[1] - cam_pos[1]
            dir_z = lower_left_corner[2] + u_coord*horizontal[2] + v_coord*vertical[2] - cam_pos[2]
            
            d = normalize((dir_x, dir_y, dir_z))
            rays.extend([cam_pos[0], cam_pos[1], cam_pos[2], d[0], d[1], d[2]])
            
    return rays

def main():
    lib_ext = '.dylib'
    lib_path = os.path.join(os.path.dirname(__file__), 'libbvh' + lib_ext)
    bvh_lib = ctypes.CDLL(lib_path)
    
    bvh_lib.build_bvh.argtypes = [ctypes.POINTER(ctypes.c_double), ctypes.c_int, ctypes.c_int]
    bvh_lib.build_bvh.restype = ctypes.c_void_p
    bvh_lib.intersect_rays.argtypes = [ctypes.c_void_p, ctypes.POINTER(ctypes.c_double), ctypes.c_int, ctypes.POINTER(ctypes.c_double), ctypes.POINTER(ctypes.c_longlong)]
    bvh_lib.intersect_rays.restype = None
    bvh_lib.destroy_bvh.argtypes = [ctypes.c_void_p]
    bvh_lib.destroy_bvh.restype = None

    obj_files = sorted(glob.glob("dataset/*.obj"))
    strategies = {1: "Spatial", 2: "Object", 3: "SAH", 4: "Random", 5: "SAH_alt(k=-0.5)", 6: "SAH_alt(k=0.5)"}
    
    print("Starting benchmark. Output will be written to result.txt")
    with open("result.txt", "w") as f_out:
        f_out.write("BVH Performance Benchmark\n")
        f_out.write("="*105 + "\n")
        f_out.write(f"{'Model':<20} | {'Triangles':<10} | {'Strategy':<10} | {'Build(s)':<8} | {'Render(s)':<9} | {'AABB Tests':<12} | {'Tri Tests':<12}\n")
        f_out.write("-" * 105 + "\n")
        
        for obj_path in obj_files:
            model_name = os.path.splitext(os.path.basename(obj_path))[0]
            flat_triangles, num_triangles, center, size = load_obj(obj_path)
            
            TriArrayType = ctypes.c_double * len(flat_triangles)
            c_triangles = TriArrayType(*flat_triangles)
            
            width, height = 512, 512
            rays = generate_rays(width, height, center, size)
            num_rays = width * height
            
            RayArrayType = ctypes.c_double * len(rays)
            c_rays = RayArrayType(*rays)
            
            HitResultArrayType = ctypes.c_double * num_rays
            c_hits = HitResultArrayType()
            
            StatsArrayType = ctypes.c_longlong * 2
            c_stats = StatsArrayType()
            
            for s_type, s_name in strategies.items():
                print(f"  Testing {model_name} with {s_name} Strategy ...")
                
                # Build BVH
                start_time = time.perf_counter()
                bvh_ptr = bvh_lib.build_bvh(c_triangles, num_triangles, s_type)
                build_time = time.perf_counter() - start_time
                
                # Render (Intersect Rays)
                start_time = time.perf_counter()
                bvh_lib.intersect_rays(bvh_ptr, c_rays, num_rays, c_hits, c_stats)
                render_time = time.perf_counter() - start_time
                
                aabb_tests = c_stats[0]
                tri_tests = c_stats[1]
                
                # Output to file
                line = f"{model_name:<20} | {num_triangles:<10} | {s_name:<10} | {build_time:<8.4f} | {render_time:<9.4f} | {aabb_tests:<12} | {tri_tests:<12}\n"
                f_out.write(line)
                f_out.flush()
                
                # Cleanup
                bvh_lib.destroy_bvh(bvh_ptr)
            
            f_out.write("-" * 105 + "\n")

    print("Benchmark complete! Results saved in result.txt")

if __name__ == "__main__":
    main()
