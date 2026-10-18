"""Streaming pipeline runner for camera-based racecar localization."""
import csv
import time
from pathlib import Path
from mcap.reader import make_reader
from mcap_ros2.decoder import DecoderFactory

from .track_map import TrackMap
from .filter import RacecarFilter
from . import config

def run_localization(mcap_path, output_csv_path, initial_pose=None, bounds_path=None):
    """
    Execute causal localization pipeline.
    
    mcap_path: path to input .mcap bag
    output_csv_path: path to write submission.csv
    initial_pose: optional tuple (x0, y0, yaw0, t0). If None, reads first pose from gt.csv or dataset.
    bounds_path: optional path to track_bounds.json
    """
    mcap_path = Path(mcap_path)
    output_csv_path = Path(output_csv_path)
    output_csv_path.parent.mkdir(parents=True, exist_ok=True)
    
    if bounds_path is None:
        bounds_path = config.TRACK_BOUNDS_PATH
        
    print(f"Loading track geometry from {bounds_path}...")
    track_map = TrackMap(bounds_path)
    
    # Obtain initial pose
    if initial_pose is None:
        print(f"Reading initial pose from {config.GT_PATH}...")
        with open(config.GT_PATH, "r") as f:
            reader = csv.DictReader(f)
            first_row = next(reader)
            initial_pose = (
                float(first_row["x"]),
                float(first_row["y"]),
                float(first_row["yaw"]),
                int(first_row["timestamp_ns"])
            )
            
    print(f"Initial Pose: x={initial_pose[0]:.4f}, y={initial_pose[1]:.4f}, yaw={initial_pose[2]:.4f}, t0={initial_pose[3]}")
    
    # Initialize Filter
    estimator = RacecarFilter(initial_pose, track_map, config)
    
    camera_topic = config.CAMERA_FRONT_TOPIC
    state_topic = config.STATE_TOPIC
    
    poses_written = 0
    t_start = time.time()
    
    print(f"Starting causal streaming from {mcap_path.name}...")
    
    with open(mcap_path, "rb") as f, open(output_csv_path, "w", newline="") as out_f:
        writer = csv.writer(out_f)
        writer.writerow(["timestamp_ns", "x", "y", "yaw"])
        
        reader = make_reader(f, decoder_factories=[DecoderFactory()])
        
        for schema, channel, message, msg in reader.iter_decoded_messages(topics=[state_topic, camera_topic]):
            topic = channel.topic
            stamp_ns = msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec
            
            if topic == state_topic:
                estimator.predict(stamp_ns, msg.vx_mps, msg.vy_mps, msg.wz_radps)
                
            elif topic == camera_topic:
                pose_x, pose_y, pose_yaw = estimator.get_pose(stamp_ns)
                writer.writerow([stamp_ns, f"{pose_x:.4f}", f"{pose_y:.4f}", f"{pose_yaw:.6f}"])
                poses_written += 1
                
                if poses_written % 2000 == 0:
                    t_now = time.time()
                    fps = poses_written / (t_now - t_start)
                    print(f"  Processed {poses_written} camera frames ({fps:.1f} fps, {1000/fps:.2f} ms/frame)...")
                    
    t_end = time.time()
    total_time = t_end - t_start
    avg_ms = (total_time / max(poses_written, 1)) * 1000
    fps = poses_written / max(total_time, 1e-4)
    
    print("\n--- Pipeline Completed Successfully ---")
    print(f"Total poses generated: {poses_written}")
    print(f"Total processing time: {total_time:.2f} s")
    print(f"Average frame latency: {avg_ms:.2f} ms ({fps:.1f} fps)")
    print(f"Submission saved to:   {output_csv_path}")
    
    return {
        "poses_written": poses_written,
        "total_time_s": total_time,
        "latency_ms": avg_ms,
        "fps": fps,
        "output_path": str(output_csv_path)
    }
