"""Visual tracking and optical flow module for onboard camera frames."""
import cv2
import numpy as np

class VisualTracker:
    def __init__(self, calib=None):
        self.calib = calib
        self.prev_gray = None
        self.prev_pts = None
        self.feature_params = dict(maxCorners=150, qualityLevel=0.03, minDistance=10, blockSize=7)
        self.lk_params = dict(winSize=(21, 21), maxLevel=3, criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, 0.01))
        
    def process_frame(self, img_bgr, stamp_ns):
        """Process new camera frame. Returns visual motion metrics."""
        h, w = img_bgr.shape[:2]
        gray = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2GRAY)
        
        # Mask out car nose and sky
        mask = np.zeros((h, w), dtype=np.uint8)
        mask[int(h * 0.35):int(h * 0.70), :] = 255
        
        obs = {
            "stamp_ns": stamp_ns,
            "flow_valid": False,
            "num_features": 0,
            "flow_dx": 0.0,
            "flow_dy": 0.0,
        }
        
        if self.prev_gray is not None and self.prev_pts is not None and len(self.prev_pts) > 10:
            p1, st, err = cv2.calcOpticalFlowPyrLK(self.prev_gray, gray, self.prev_pts, None, **self.lk_params)
            good_p0 = self.prev_pts[st == 1]
            good_p1 = p1[st == 1]
            
            if len(good_p1) >= 10:
                obs["flow_valid"] = True
                obs["num_features"] = len(good_p1)
                obs["flow_dx"] = float(np.median(good_p1[:, 0] - good_p0[:, 0]))
                obs["flow_dy"] = float(np.median(good_p1[:, 1] - good_p0[:, 1]))
                
        # Re-detect features if count drops
        if self.prev_pts is None or len(self.prev_pts) < 50:
            new_pts = cv2.goodFeaturesToTrack(gray, mask=mask, **self.feature_params)
            self.prev_pts = new_pts
        else:
            self.prev_pts = cv2.goodFeaturesToTrack(gray, mask=mask, **self.feature_params)
            
        self.prev_gray = gray
        return obs
