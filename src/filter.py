"""High-performance causal filter for camera-rate racecar localization."""
import math
import numpy as np

class RacecarFilter:
    def __init__(self, init_pose, track_map, config):
        """
        init_pose: (x0, y0, yaw0, t0)
        track_map: TrackMap instance
        config: config module
        """
        self.x = float(init_pose[0])
        self.y = float(init_pose[1])
        self.yaw = float(init_pose[2])
        self.t_last = int(init_pose[3])
        
        self.track_map = track_map
        self.cfg = config
        
        self.bg = getattr(config, "DEFAULT_GYRO_BIAS", -0.000204)
        self.scale_v = getattr(config, "SPEED_SCALE_FACTOR", 1.000214)
        
        self.last_vx = 0.0
        self.last_vy = 0.0
        self.last_wz = 0.0
        self.t0 = self.t_last
        
    def predict(self, stamp_ns, vx, vy, wz):
        """Causal state update at 106 Hz."""
        dt = (stamp_ns - self.t_last) / 1e9
        if dt <= 0.0 or dt > 1.0:
            self.t_last = stamp_ns
            return
            
        vx_corr = vx * self.scale_v
        wz_corr = wz - self.bg
        
        # Midpoint integration for orientation
        mid_yaw = self.yaw + 0.5 * wz_corr * dt
        self.yaw += wz_corr * dt
        
        # Vehicle body velocity projection into map coordinates
        cos_y = math.cos(mid_yaw)
        sin_y = math.sin(mid_yaw)
        dx = (vx_corr * cos_y - vy * sin_y) * dt
        dy = (vx_corr * sin_y + vy * cos_y) * dt
        self.x += dx
        self.y += dy
        
        self.t_last = stamp_ns
        self.last_vx = vx_corr
        self.last_vy = vy
        self.last_wz = wz_corr
        
    def get_pose(self, cam_stamp_ns):
        """Compute causal pose (x, y, yaw) at exact camera exposure timestamp."""
        dt_cam = (cam_stamp_ns - self.t_last) / 1e9
        out_x = self.x
        out_y = self.y
        out_yaw = self.yaw
        
        if 0 < dt_cam < 0.1:
            mid_yaw = out_yaw + 0.5 * self.last_wz * dt_cam
            out_yaw += self.last_wz * dt_cam
            cos_y = math.cos(mid_yaw)
            sin_y = math.sin(mid_yaw)
            out_x += (self.last_vx * cos_y - self.last_vy * sin_y) * dt_cam
            out_y += (self.last_vx * sin_y + self.last_vy * cos_y) * dt_cam
            
        # Normalize yaw to [-pi, pi]
        out_yaw = (out_yaw + math.pi) % (2 * math.pi) - math.pi
        return out_x, out_y, out_yaw
