import json
import numpy as np
from scipy.spatial import cKDTree

class TrackMap:
    def __init__(self, bounds_path):
        with open(bounds_path, "r") as f:
            self.bounds_data = json.load(f)
            
        self.left_pts = np.array(self.bounds_data.get("track_left", []))[:, :2]
        self.right_pts = np.array(self.bounds_data.get("track_right", []))[:, :2]
        self.pit_left = np.array(self.bounds_data.get("pitlane_left", []))[:, :2]
        self.pit_right = np.array(self.bounds_data.get("pitlane_right", []))[:, :2]
        
        self.kdtree_left = cKDTree(self.left_pts)
        self.kdtree_right = cKDTree(self.right_pts)
        
        self._build_centerline()
        
    def _build_centerline(self):
        dists, idxs = self.kdtree_right.query(self.left_pts)
        matched_right = self.right_pts[idxs]
        center_raw = 0.5 * (self.left_pts + matched_right)
        
        diffs = np.diff(center_raw, axis=0)
        lens = np.hypot(diffs[:, 0], diffs[:, 1])
        keep = np.insert(lens > 0.05, 0, True)
        center_raw = center_raw[keep]
        
        diffs = np.diff(center_raw, axis=0)
        seg_lens = np.hypot(diffs[:, 0], diffs[:, 1])
        s_vals = np.insert(np.cumsum(seg_lens), 0, 0.0)
        self.lap_length = s_vals[-1]
        
        s_uniform = np.arange(0, self.lap_length, 0.5)
        cx = np.interp(s_uniform, s_vals, center_raw[:, 0], period=self.lap_length)
        cy = np.interp(s_uniform, s_vals, center_raw[:, 1], period=self.lap_length)
        self.center_pts = np.column_stack([cx, cy])
        self.s_grid = s_uniform
        self.kdtree_center = cKDTree(self.center_pts)
        
        dx = np.gradient(cx)
        dy = np.gradient(cy)
        self.thetas = np.arctan2(dy, dx)
        
        d_left, _ = self.kdtree_left.query(self.center_pts)
        d_right, _ = self.kdtree_right.query(self.center_pts)
        self.width_left = d_left
        self.width_right = d_right
        
    def xy_to_sn(self, x, y):
        pt = np.array([x, y])
        dist, idx = self.kdtree_center.query(pt)
        c_pt = self.center_pts[idx]
        s = self.s_grid[idx]
        theta = self.thetas[idx]
        
        dx = x - c_pt[0]
        dy = y - c_pt[1]
        n = -np.sin(theta) * dx + np.cos(theta) * dy
        ds = np.cos(theta) * dx + np.sin(theta) * dy
        s = (s + ds) % self.lap_length
        return s, n, theta
        
    def sn_to_xy(self, s, n):
        s = s % self.lap_length
        idx = int(np.round(s / 0.5)) % len(self.center_pts)
        c_pt = self.center_pts[idx]
        theta = self.thetas[idx]
        x = c_pt[0] - n * np.sin(theta)
        y = c_pt[1] + n * np.cos(theta)
        return x, y, theta

if __name__ == "__main__":
    tm = TrackMap(r"C:\Users\NARESHSARATHY S\.gemini\antigravity\scratch\racecar_camloc\data\track_bounds.json")
    print(f"Track centerline initialized! Lap length: {tm.lap_length:.1f} m, points: {len(tm.center_pts)}")
    test_x, test_y = tm.center_pts[100] + [2.0, -1.0]
    s, n, th = tm.xy_to_sn(test_x, test_y)
    rx, ry, _ = tm.sn_to_xy(s, n)
    print(f"Original: ({test_x:.3f}, {test_y:.3f}) -> (s={s:.2f}, n={n:.2f}) -> Reconstructed: ({rx:.3f}, {ry:.3f})")
    err = np.hypot(test_x - rx, test_y - ry)
    print(f"Roundtrip error: {err:.4f} m")
