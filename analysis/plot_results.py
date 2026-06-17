import matplotlib.pyplot as plt
import os

def main():
    data = []
    # result.txt は 1つ上の階層にあると想定
    result_path = '../result.txt' if os.path.exists('../result.txt') else 'result.txt'
    
    if not os.path.exists(result_path):
        print(f"Error: {result_path} not found.")
        return

    with open(result_path, 'r') as f:
        for line in f:
            if '|' in line and 'Strategy' not in line:
                parts = [p.strip() for p in line.split('|')]
                if len(parts) >= 7:
                    model = parts[0]
                    tri_count = int(parts[1])
                    strategy = parts[2]
                    build_s = float(parts[3])
                    render_s = float(parts[4])
                    aabb = int(parts[5])
                    tri_tests = int(parts[6])
                    data.append({
                        'model': model,
                        'tri_count': tri_count,
                        'strategy': strategy,
                        'build': build_s,
                        'render': render_s,
                        'aabb': aabb,
                        'tri_tests': tri_tests
                    })

    # Get unique strategies
    strategies = list(dict.fromkeys([d['strategy'] for d in data]))

    def plot_metric(metric_key, ylabel, filename, is_y_log=True, is_x_log=True):
        fig, ax = plt.subplots(figsize=(10, 6))
        
        for strat in strategies:
            # Filter and sort by tri_count
            strat_data = [d for d in data if d['strategy'] == strat]
            strat_data.sort(key=lambda x: x['tri_count'])
            
            x = [d['tri_count'] for d in strat_data]
            y = [d[metric_key] for d in strat_data]
            
            ax.plot(x, y, marker='o', label=strat)

        ax.set_xlabel('Number of Triangles (N)')
        ax.set_ylabel(ylabel)
        ax.set_title(ylabel + ' vs Number of Triangles')
        
        if is_x_log:
            ax.set_xscale('log')
        if is_y_log:
            ax.set_yscale('log')
            
        ax.legend()
        ax.grid(True, which="both", ls="--", alpha=0.5)
        
        plt.tight_layout()
        # カレントディレクトリに保存 (analysisディレクトリ内で実行される前提)
        out_dir = os.path.dirname(os.path.abspath(__file__))
        plt.savefig(os.path.join(out_dir, filename))
        plt.close()

    plot_metric('build', 'Build Time (s)', 'build_time.png', is_y_log=True, is_x_log=True)
    plot_metric('render', 'Render Time (s)', 'render_time.png', is_y_log=False, is_x_log=True)
    plot_metric('aabb', 'AABB Tests Count', 'aabb_tests.png', is_y_log=False, is_x_log=True)
    plot_metric('tri_tests', 'Triangle Tests Count', 'tri_tests.png', is_y_log=False, is_x_log=True)

    print("Line charts generated successfully.")

if __name__ == '__main__':
    main()
