#!/usr/bin/env python3
"""
Benchmark query latency for retrieving first-move matrices from:
1. MP4 video (grayscale frames + mapping file)
2. CPD file via cpp_first_move_matrix bindings
"""

import argparse
import json
import random
import time
from pathlib import Path
from typing import List, Tuple, Dict

import cv2
import numpy as np

try:
    import cpp_first_move_matrix
    CPP_AVAILABLE = True
except ImportError:
    CPP_AVAILABLE = False

from test_video_compression import frame_to_matrix


def load_mapping(mapping_path: Path) -> List[Tuple[Tuple[int, int], int]]:
    """Load goal-to-frame mapping created during video generation."""
    with open(mapping_path, "r") as f:
        data = json.load(f)

    mapping = []
    for entry in data:
        goal = tuple(entry["goal"])
        frame_idx = entry["frame"]
        mapping.append((goal, frame_idx))
    return mapping


def sample_goals(mapping: List[Tuple[Tuple[int, int], int]], sample_size: int, seed: int) -> List[Tuple[Tuple[int, int], int]]:
    rng = random.Random(seed)
    sample_size = min(sample_size, len(mapping))
    return rng.sample(mapping, sample_size)


def benchmark_video_queries(video_path: Path, samples: List[Tuple[Tuple[int, int], int]]) -> Dict[str, float]:
    cap = cv2.VideoCapture(str(video_path))
    if not cap.isOpened():
        raise RuntimeError(f"Failed to open video file: {video_path}")

    start = time.perf_counter()
    for _, frame_idx in samples:
        cap.set(cv2.CAP_PROP_POS_FRAMES, frame_idx)
        ok, frame = cap.read()
        if not ok:
            raise RuntimeError(f"Failed to read frame {frame_idx}")
        _ = frame_to_matrix(frame)
    elapsed = time.perf_counter() - start
    cap.release()
    return {
        "total_time": elapsed,
        "avg_time": elapsed / len(samples),
        "queries_per_sec": len(samples) / elapsed if elapsed > 0 else float("inf"),
    }


def benchmark_cpd_queries(preprocessed_path: Path, map_path: Path, samples: List[Tuple[Tuple[int, int], int]]) -> Dict[str, float]:
    if not CPP_AVAILABLE:
        raise RuntimeError("cpp_first_move_matrix module not available.")

    goals = [goal for goal, _ in samples]
    start = time.perf_counter()
    _ = cpp_first_move_matrix.extract_first_move_matrices(
        str(preprocessed_path),
        str(map_path),
        goals,
    )
    elapsed = time.perf_counter() - start
    return {
        "total_time": elapsed,
        "avg_time": elapsed / len(samples),
        "queries_per_sec": len(samples) / elapsed if elapsed > 0 else float("inf"),
    }


def main():
    parser = argparse.ArgumentParser(description="Benchmark query speed for video vs CPD retrieval.")
    parser.add_argument("--video", type=Path, default=Path("data/first_move_matrices.mp4"), help="Path to MP4 video file.")
    parser.add_argument("--mapping", type=Path, default=Path("data/first_move_matrices_mapping.json"), help="Path to mapping JSON.")
    parser.add_argument("--preprocessed", type=Path, default=Path("data/first_move_matrix_cpp.map"), help="Path to CPD data file.")
    parser.add_argument("--map", type=Path, default=Path("data/map.map"), help="Path to octile map file.")
    parser.add_argument("--samples", type=int, default=100, help="Number of random queries to benchmark.")
    parser.add_argument("--seed", type=int, default=42, help="RNG seed for sampling goals.")
    args = parser.parse_args()

    mapping = load_mapping(args.mapping)
    samples = sample_goals(mapping, args.samples, args.seed)
    print(f"Benchmarking with {len(samples)} random goals.")

    print("\n=== Video Query Benchmark ===")
    video_stats = benchmark_video_queries(args.video, samples)
    for k, v in video_stats.items():
        print(f"{k}: {v:.6f}s" if "time" in k else f"{k}: {v:.2f}")

    print("\n=== CPD Query Benchmark ===")
    cpd_stats = benchmark_cpd_queries(args.preprocessed, args.map, samples)
    for k, v in cpd_stats.items():
        print(f"{k}: {v:.6f}s" if "time" in k else f"{k}: {v:.2f}")


if __name__ == "__main__":
    main()
