#!/usr/bin/env python3
"""
Comprehensive benchmark script that compares three path planners:
1. jps_planner_bindings
2. BL_JPS
3. silas single_agent_planning_2d

Tests across three distance categories (short, middle, long) with 100 trials each.
Results are saved to organized folder structure.
"""

import sys
import time
import random
import json
import csv
from pathlib import Path
import numpy as np
import cv2
from datetime import datetime
import matplotlib.pyplot as plt
import seaborn as sns
from dataclasses import dataclass, asdict
from typing import List, Tuple, Optional, Dict, Any

# Import planners
try:
    import jps_planner_bindings
    JPS_AVAILABLE = True
    print("✓ jps_planner_bindings imported successfully")
except ImportError as e:
    JPS_AVAILABLE = False
    print(f"✗ jps_planner_bindings import failed: {e}")

try:
    import BL_JPS
    BLJPS_AVAILABLE = True
    print("✓ BL_JPS imported successfully")
except ImportError as e:
    BLJPS_AVAILABLE = False
    print(f"✗ BL_JPS import failed: {e}")

# Import silas
SILAS_OS_PATH = Path('.').resolve() / '3rdparty' / 'silas-os-python'
sys.path.append(str(SILAS_OS_PATH))
try:
    from silas.os.kernel.tspatial_processing.planning.astar.astar_2d import AStar
    SILAS_AVAILABLE = True
    print("✓ silas AStar imported successfully")
except ImportError as e:
    SILAS_AVAILABLE = False
    print(f"✗ silas import failed: {e}")

@dataclass
class BenchmarkResult:
    """Single benchmark trial result."""
    planner: str
    distance_category: str
    trial_id: int
    start_point: Tuple[int, int]
    end_point: Tuple[int, int]
    euclidean_distance: float
    success: bool
    runtime_ms: float
    path_length: int
    path_distance: float
    path_efficiency: float
    error_message: Optional[str] = None

@dataclass
class BenchmarkSummary:
    """Summary statistics for a planner-category combination."""
    planner: str
    distance_category: str
    total_trials: int
    successful_trials: int
    success_rate: float
    avg_runtime_ms: float
    std_runtime_ms: float
    min_runtime_ms: float
    max_runtime_ms: float
    avg_path_efficiency: float
    std_path_efficiency: float

class PathPlannerInterface:
    """Base interface for path planners."""

    def __init__(self, name: str):
        self.name = name

    def plan(self, grid: np.ndarray, start: Tuple[int, int], goal: Tuple[int, int]) -> Tuple[bool, List[Tuple[int, int]], float]:
        """
        Plan a path from start to goal.

        Returns:
            (success, path, runtime_ms)
        """
        raise NotImplementedError

class JPSPlannerWrapper(PathPlannerInterface):
    """Wrapper for jps_planner_bindings."""

    def __init__(self):
        super().__init__("JPS")
        self.resolution = 1.0

    def plan(self, grid: np.ndarray, start: Tuple[int, int], goal: Tuple[int, int]) -> Tuple[bool, List[Tuple[int, int]], float]:
        if not JPS_AVAILABLE:
            return False, [], 0.0

        try:
            height, width = grid.shape
            origin = [0, 0]
            dim = [width, height]

            # Convert grid to format expected by JPS (flatten and convert to int)
            map_data = []
            for y in range(height):
                for x in range(width):
                    if grid[y, x]:  # True = obstacle
                        map_data.append(1)
                    else:
                        map_data.append(0)

            # Convert start/goal from (row, col) to world coordinates
            start_world = [float(start[1]), float(start[0])]  # (row, col) -> (x, y)
            goal_world = [float(goal[1]), float(goal[0])]

            result = jps_planner_bindings.plan_2d(
                origin, dim, map_data, start_world, goal_world, self.resolution, True
            )

            # Use planner's built-in timing
            runtime_ms = result.time_spent

            if result.path and len(result.path) > 0:
                # Convert path back to (row, col) format
                path = [(int(p[1]), int(p[0])) for p in result.path]  # (x, y) -> (row, col)
                return True, path, runtime_ms
            else:
                return False, [], runtime_ms

        except Exception as e:
            print(f"JPS planning error: {e}")
            return False, [], 0.0

class BLJPSPlannerWrapper(PathPlannerInterface):
    """Wrapper for BL_JPS."""

    def __init__(self):
        super().__init__("BL_JPS")
        self.planner = BL_JPS.BL_JPS() if BLJPS_AVAILABLE else None

    def plan(self, grid: np.ndarray, start: Tuple[int, int], goal: Tuple[int, int]) -> Tuple[bool, List[Tuple[int, int]], float]:
        if not BLJPS_AVAILABLE or self.planner is None:
            return False, [], 0.0

        try:
            height, width = grid.shape
            origin = (0, 0)

            # Convert grid to format expected by BL_JPS
            map_data = grid.astype(np.int32).flatten().tolist()

            # BL_JPS expects (x, y) coordinates
            start_x, start_y = start[1], start[0]  # (row, col) -> (x, y)
            goal_x, goal_y = goal[1], goal[0]

            result = self.planner.plan_2d(
                map_data, width=width, height=height,
                startX=start_x, startY=start_y,
                endX=goal_x, endY=goal_y,
                originX=origin[0], originY=origin[1],
                resolution=1
            )

            # Use planner's built-in timing
            runtime_ms = result.time_spent

            if result.path and len(result.path) > 0:
                # Uncompress and convert path to (row, col) format
                path = self._uncompress_bljps_path(result.path)
                path = [(p[1], p[0]) for p in path]  # (x, y) -> (row, col)
                return True, path, runtime_ms
            else:
                return False, [], runtime_ms

        except Exception as e:
            print(f"BL_JPS planning error: {e}")
            return False, [], 0.0

    def _uncompress_bljps_path(self, path):
        """Uncompress BL_JPS path format."""
        if not path:
            return []

        uncompressed_path = []
        for p_id in range(len(path) - 1):
            current_p = [path[p_id][0], path[p_id][1]]
            uncompressed_path.append(tuple(current_p))

            while current_p[0] != path[p_id + 1][0] or current_p[1] != path[p_id + 1][1]:
                if path[p_id + 1][0] != current_p[0]:
                    if path[p_id + 1][0] > current_p[0]:
                        current_p[0] += 1
                    else:
                        current_p[0] -= 1
                    uncompressed_path.append(tuple(current_p))
                if path[p_id + 1][1] != current_p[1]:
                    if path[p_id + 1][1] > current_p[1]:
                        current_p[1] += 1
                    else:
                        current_p[1] -= 1
                    uncompressed_path.append(tuple(current_p))

        return uncompressed_path

class SilasPlannerWrapper(PathPlannerInterface):
    """Wrapper for silas single_agent_planning_2d."""

    def __init__(self):
        super().__init__("Silas")

    def plan(self, grid: np.ndarray, start: Tuple[int, int], goal: Tuple[int, int]) -> Tuple[bool, List[Tuple[int, int]], float]:
        if not SILAS_AVAILABLE:
            return False, [], 0.0

        try:
            # Convert grid to format expected by AStar (True = free space, False = obstacle)
            free_space_grid = np.invert(grid.astype(bool))

            # Create AStar instance
            astar = AStar(free_space_grid)

            start_time = time.perf_counter()
            path = astar.search(start, goal)
            end_time = time.perf_counter()

            runtime_ms = (end_time - start_time) * 1000

            if path and len(path) > 0:
                return True, path, runtime_ms
            else:
                return False, [], runtime_ms

        except Exception as e:
            print(f"Silas planning error: {e}")
            return False, [], 0.0

class BenchmarkRunner:
    """Main benchmark runner."""

    def __init__(self, image_path: Path, results_dir: Path):
        self.image_path = image_path
        self.results_dir = results_dir
        self.results_dir.mkdir(parents=True, exist_ok=True)

        # Initialize planners
        self.planners = {}
        if JPS_AVAILABLE:
            self.planners["JPS"] = JPSPlannerWrapper()
        if BLJPS_AVAILABLE:
            self.planners["BL_JPS"] = BLJPSPlannerWrapper()
        if SILAS_AVAILABLE:
            self.planners["Silas"] = SilasPlannerWrapper()

        # Load and process image
        self.image = None
        self.grid = None
        self.free_mask = None
        self.image_stats = {}

        self._load_image()

    def _load_image(self):
        """Load and process the image."""
        print(f"Loading image: {self.image_path}")

        self.image = cv2.imread(str(self.image_path), cv2.IMREAD_GRAYSCALE)
        if self.image is None:
            raise ValueError(f"Could not load image: {self.image_path}")

        print(f"Image shape: {self.image.shape}")
        print(f"Image dtype: {self.image.dtype}")
        print(f"Pixel value range: {self.image.min()} - {self.image.max()}")

        # Create boolean grid (True = obstacle, False = free)
        self.grid = self.image >= 128

        # Create free space mask
        self.free_mask = self.image < 128

        # Calculate statistics
        free_pixels = np.sum(self.free_mask)
        total_pixels = self.image.shape[0] * self.image.shape[1]
        free_percentage = (free_pixels / total_pixels) * 100

        self.image_stats = {
            "shape": self.image.shape,
            "total_pixels": total_pixels,
            "free_pixels": int(free_pixels),
            "obstacle_pixels": int(total_pixels - free_pixels),
            "free_percentage": free_percentage
        }

        print(f"Free space: {free_pixels}/{total_pixels} pixels ({free_percentage:.1f}%)")

    def find_random_free_points(self, num_points: int = 2, max_attempts: int = 1000) -> List[Tuple[int, int]]:
        """Find random points in free space."""
        height, width = self.image.shape
        free_points = []

        attempts = 0
        while len(free_points) < num_points and attempts < max_attempts:
            row = random.randint(0, height - 1)
            col = random.randint(0, width - 1)

            if self.free_mask[row, col]:
                free_points.append((row, col))  # Return as (row, col)

            attempts += 1

        if len(free_points) < num_points:
            raise ValueError(f"Could not find {num_points} free points after {max_attempts} attempts")

        return free_points

    def categorize_distance(self, distance: float, image_diagonal: float) -> str:
        """Categorize distance as short, middle, or long."""
        # Define thresholds as percentages of image diagonal
        short_threshold = 0.2 * image_diagonal  # 20% of diagonal
        long_threshold = 0.6 * image_diagonal   # 60% of diagonal

        if distance <= short_threshold:
            return "short"
        elif distance <= long_threshold:
            return "middle"
        else:
            return "long"

    def generate_point_pairs_by_category(self, trials_per_category: int = 100) -> Dict[str, List[Tuple[Tuple[int, int], Tuple[int, int], float]]]:
        """Generate point pairs categorized by distance."""
        height, width = self.image.shape
        image_diagonal = np.sqrt(height**2 + width**2)

        categories = {"short": [], "middle": [], "long": []}
        max_attempts = trials_per_category * 50  # Allow many attempts to find good distributions

        print(f"Generating point pairs (image diagonal: {image_diagonal:.1f} pixels)...")
        print(f"Distance thresholds - Short: ≤{0.2*image_diagonal:.1f}, Middle: ≤{0.6*image_diagonal:.1f}, Long: >{0.6*image_diagonal:.1f}")

        attempts = 0
        while (len(categories["short"]) < trials_per_category or
               len(categories["middle"]) < trials_per_category or
               len(categories["long"]) < trials_per_category) and attempts < max_attempts:

            try:
                points = self.find_random_free_points(2)
                start, goal = points[0], points[1]

                distance = np.sqrt((goal[0] - start[0])**2 + (goal[1] - start[1])**2)
                category = self.categorize_distance(distance, image_diagonal)

                if len(categories[category]) < trials_per_category:
                    categories[category].append((start, goal, distance))

                attempts += 1

                if attempts % 1000 == 0:
                    print(f"  Progress: Short={len(categories['short'])}, Middle={len(categories['middle'])}, Long={len(categories['long'])} (attempts: {attempts})")

            except ValueError:
                attempts += 1
                continue

        # Report final counts
        for category, pairs in categories.items():
            print(f"Generated {len(pairs)} {category} distance pairs")
            if pairs:
                distances = [pair[2] for pair in pairs]
                print(f"  {category.capitalize()} distance range: {min(distances):.1f} - {max(distances):.1f} pixels")

        return categories

    def calculate_path_metrics(self, path: List[Tuple[int, int]], euclidean_distance: float) -> Tuple[int, float, float]:
        """Calculate path metrics."""
        if not path or len(path) == 0:
            return 0, 0.0, 0.0

        path_length = len(path)

        if len(path) == 1:
            path_distance = 0.0
            path_efficiency = 1.0 if euclidean_distance == 0 else 0.0
        else:
            # Calculate total path distance
            path_distance = 0.0
            for i in range(1, len(path)):
                prev_row, prev_col = path[i-1]
                curr_row, curr_col = path[i]
                segment_dist = np.sqrt((curr_col - prev_col)**2 + (curr_row - prev_row)**2)
                path_distance += segment_dist

            # Calculate efficiency (1.0 = straight line)
            path_efficiency = euclidean_distance / path_distance if path_distance > 0 else 0.0

        return path_length, path_distance, path_efficiency

    def run_single_trial(self, planner_name: str, category: str, trial_id: int,
                        start: Tuple[int, int], goal: Tuple[int, int],
                        euclidean_distance: float) -> BenchmarkResult:
        """Run a single benchmark trial."""
        planner = self.planners[planner_name]

        try:
            success, path, runtime_ms = planner.plan(self.grid, start, goal)

            if success and path:
                path_length, path_distance, path_efficiency = self.calculate_path_metrics(path, euclidean_distance)
            else:
                path_length, path_distance, path_efficiency = 0, 0.0, 0.0

            return BenchmarkResult(
                planner=planner_name,
                distance_category=category,
                trial_id=trial_id,
                start_point=start,
                end_point=goal,
                euclidean_distance=euclidean_distance,
                success=success,
                runtime_ms=runtime_ms,
                path_length=path_length,
                path_distance=path_distance,
                path_efficiency=path_efficiency
            )

        except Exception as e:
            return BenchmarkResult(
                planner=planner_name,
                distance_category=category,
                trial_id=trial_id,
                start_point=start,
                end_point=goal,
                euclidean_distance=euclidean_distance,
                success=False,
                runtime_ms=0.0,
                path_length=0,
                path_distance=0.0,
                path_efficiency=0.0,
                error_message=str(e)
            )

    def run_benchmark(self, trials_per_category: int = 100) -> List[BenchmarkResult]:
        """Run the complete benchmark."""
        print(f"\n=== Starting Comprehensive Path Planning Benchmark ===")
        print(f"Available planners: {list(self.planners.keys())}")
        print(f"Trials per category: {trials_per_category}")

        # Generate point pairs by category
        point_pairs = self.generate_point_pairs_by_category(trials_per_category)

        all_results = []
        total_trials = len(self.planners) * 3 * trials_per_category
        completed_trials = 0

        start_time = datetime.now()

        for planner_name in self.planners.keys():
            print(f"\n--- Testing {planner_name} ---")

            for category in ["short", "middle", "long"]:
                print(f"  Running {category} distance trials...")

                pairs = point_pairs[category]
                for trial_id, (start, goal, distance) in enumerate(pairs):
                    result = self.run_single_trial(
                        planner_name, category, trial_id, start, goal, distance
                    )
                    all_results.append(result)

                    completed_trials += 1
                    if completed_trials % 50 == 0:
                        elapsed = datetime.now() - start_time
                        progress = completed_trials / total_trials * 100
                        print(f"    Progress: {completed_trials}/{total_trials} ({progress:.1f}%) - Elapsed: {elapsed}")

        print(f"\nBenchmark completed! Total trials: {len(all_results)}")
        return all_results

    def save_results(self, results: List[BenchmarkResult], timestamp: str):
        """Save benchmark results to files."""
        print(f"\nSaving results to {self.results_dir}...")

        # Save raw results as JSON
        json_path = self.results_dir / f"raw_results_{timestamp}.json"
        with open(json_path, 'w') as f:
            json.dump([asdict(r) for r in results], f, indent=2)
        print(f"Raw results saved to: {json_path}")

        # Save results as CSV
        csv_path = self.results_dir / f"results_{timestamp}.csv"
        with open(csv_path, 'w', newline='') as f:
            if results:
                writer = csv.DictWriter(f, fieldnames=asdict(results[0]).keys())
                writer.writeheader()
                for result in results:
                    writer.writerow(asdict(result))
        print(f"CSV results saved to: {csv_path}")

        # Generate and save summary statistics
        summary_stats = self.generate_summary_statistics(results)
        summary_path = self.results_dir / f"summary_{timestamp}.json"
        with open(summary_path, 'w') as f:
            json.dump([asdict(s) for s in summary_stats], f, indent=2)
        print(f"Summary statistics saved to: {summary_path}")

        # Save metadata
        metadata = {
            "timestamp": timestamp,
            "image_path": str(self.image_path),
            "image_stats": self.image_stats,
            "available_planners": list(self.planners.keys()),
            "total_trials": len(results),
            "trials_per_category": len([r for r in results if r.distance_category == "short" and r.planner == list(self.planners.keys())[0]])
        }
        metadata_path = self.results_dir / f"metadata_{timestamp}.json"
        with open(metadata_path, 'w') as f:
            json.dump(metadata, f, indent=2)
        print(f"Metadata saved to: {metadata_path}")

    def generate_summary_statistics(self, results: List[BenchmarkResult]) -> List[BenchmarkSummary]:
        """Generate summary statistics for each planner-category combination."""
        summaries = []

        for planner in self.planners.keys():
            for category in ["short", "middle", "long"]:
                # Filter results for this combination
                filtered_results = [r for r in results if r.planner == planner and r.distance_category == category]

                if not filtered_results:
                    continue

                successful_results = [r for r in filtered_results if r.success]

                total_trials = len(filtered_results)
                successful_trials = len(successful_results)
                success_rate = successful_trials / total_trials if total_trials > 0 else 0.0

                if successful_results:
                    runtimes = [r.runtime_ms for r in successful_results]
                    efficiencies = [r.path_efficiency for r in successful_results if r.path_efficiency > 0]

                    avg_runtime = np.mean(runtimes)
                    std_runtime = np.std(runtimes)
                    min_runtime = np.min(runtimes)
                    max_runtime = np.max(runtimes)

                    avg_efficiency = np.mean(efficiencies) if efficiencies else 0.0
                    std_efficiency = np.std(efficiencies) if efficiencies else 0.0
                else:
                    avg_runtime = std_runtime = min_runtime = max_runtime = 0.0
                    avg_efficiency = std_efficiency = 0.0

                summary = BenchmarkSummary(
                    planner=planner,
                    distance_category=category,
                    total_trials=total_trials,
                    successful_trials=successful_trials,
                    success_rate=success_rate,
                    avg_runtime_ms=avg_runtime,
                    std_runtime_ms=std_runtime,
                    min_runtime_ms=min_runtime,
                    max_runtime_ms=max_runtime,
                    avg_path_efficiency=avg_efficiency,
                    std_path_efficiency=std_efficiency
                )
                summaries.append(summary)

        return summaries

    def create_visualizations(self, results: List[BenchmarkResult], timestamp: str):
        """Create visualization plots."""
        print(f"\nCreating visualizations...")

        # Set up plotting style
        plt.style.use('default')
        sns.set_palette("husl")

        # Create subplots
        fig, axes = plt.subplots(2, 2, figsize=(15, 12))
        fig.suptitle(f'Path Planning Benchmark Results - {timestamp}', fontsize=16)

        # Prepare data for plotting
        df_data = []
        for result in results:
            if result.success:  # Only include successful trials
                df_data.append({
                    'Planner': result.planner,
                    'Category': result.distance_category,
                    'Runtime (ms)': result.runtime_ms,
                    'Path Efficiency': result.path_efficiency,
                    'Success': result.success
                })

        if not df_data:
            print("No successful trials to visualize")
            return

        import pandas as pd
        df = pd.DataFrame(df_data)

        # 1. Runtime comparison by planner and category
        sns.boxplot(data=df, x='Category', y='Runtime (ms)', hue='Planner', ax=axes[0,0])
        axes[0,0].set_title('Runtime Comparison by Distance Category')
        axes[0,0].set_yscale('log')

        # 2. Path efficiency comparison
        sns.boxplot(data=df, x='Category', y='Path Efficiency', hue='Planner', ax=axes[0,1])
        axes[0,1].set_title('Path Efficiency by Distance Category')

        # 3. Success rate by planner and category
        success_data = []
        for planner in self.planners.keys():
            for category in ["short", "middle", "long"]:
                filtered = [r for r in results if r.planner == planner and r.distance_category == category]
                if filtered:
                    success_rate = sum(1 for r in filtered if r.success) / len(filtered)
                    success_data.append({
                        'Planner': planner,
                        'Category': category,
                        'Success Rate': success_rate
                    })

        if success_data:
            success_df = pd.DataFrame(success_data)
            sns.barplot(data=success_df, x='Category', y='Success Rate', hue='Planner', ax=axes[1,0])
            axes[1,0].set_title('Success Rate by Distance Category')
            axes[1,0].set_ylim(0, 1.1)

        # 4. Runtime vs Distance scatter plot
        scatter_data = []
        for result in results:
            if result.success:
                scatter_data.append({
                    'Planner': result.planner,
                    'Euclidean Distance': result.euclidean_distance,
                    'Runtime (ms)': result.runtime_ms
                })

        if scatter_data:
            scatter_df = pd.DataFrame(scatter_data)
            for planner in self.planners.keys():
                planner_data = scatter_df[scatter_df['Planner'] == planner]
                if not planner_data.empty:
                    axes[1,1].scatter(planner_data['Euclidean Distance'], planner_data['Runtime (ms)'],
                                    label=planner, alpha=0.6)
            axes[1,1].set_xlabel('Euclidean Distance (pixels)')
            axes[1,1].set_ylabel('Runtime (ms)')
            axes[1,1].set_title('Runtime vs Distance')
            axes[1,1].set_yscale('log')
            axes[1,1].legend()

        plt.tight_layout()

        # Save plot
        plot_path = self.results_dir / f"benchmark_plots_{timestamp}.png"
        plt.savefig(plot_path, dpi=300, bbox_inches='tight')
        plt.close()
        print(f"Visualizations saved to: {plot_path}")

def main():
    """Main function to run the benchmark."""
    # Configuration
    image_path = Path("data/image.png")
    results_dir = Path("benchmark_results")
    trials_per_category = 50

    # Check if image exists
    if not image_path.exists():
        print(f"Error: Image not found at {image_path}")
        return False

    # Check if at least one planner is available
    if not (JPS_AVAILABLE or BLJPS_AVAILABLE or SILAS_AVAILABLE):
        print("Error: No planners available!")
        return False

    # Create timestamp for this run
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")

    try:
        # Initialize benchmark runner
        runner = BenchmarkRunner(image_path, results_dir)

        # Run benchmark
        results = runner.run_benchmark(trials_per_category)

        # Save results
        runner.save_results(results, timestamp)

        # Create visualizations
        try:
            runner.create_visualizations(results, timestamp)
        except ImportError:
            print("Warning: Could not create visualizations (pandas/seaborn not available)")
        except Exception as e:
            print(f"Warning: Could not create visualizations: {e}")

        # Print summary
        print(f"\n=== Benchmark Summary ===")
        summary_stats = runner.generate_summary_statistics(results)

        for summary in summary_stats:
            print(f"\n{summary.planner} - {summary.distance_category.capitalize()}:")
            print(f"  Success Rate: {summary.success_rate:.1%} ({summary.successful_trials}/{summary.total_trials})")
            if summary.successful_trials > 0:
                print(f"  Avg Runtime: {summary.avg_runtime_ms:.2f} ± {summary.std_runtime_ms:.2f} ms")
                print(f"  Runtime Range: {summary.min_runtime_ms:.2f} - {summary.max_runtime_ms:.2f} ms")
                print(f"  Avg Path Efficiency: {summary.avg_path_efficiency:.3f} ± {summary.std_path_efficiency:.3f}")

        print(f"\nAll results saved to: {results_dir}")
        return True

    except Exception as e:
        print(f"Benchmark failed: {e}")
        import traceback
        traceback.print_exc()
        return False

if __name__ == "__main__":
    success = main()
    print(f"\nBenchmark {'completed successfully' if success else 'failed'}")
    sys.exit(0 if success else 1)
