import math
import numpy as np
import matplotlib.pyplot as plt

def _wrap_pi(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))

def _ang_diff_abs(a: float, b: float) -> float:
    """Shortest absolute angular difference in radians."""
    return abs(_wrap_pi(a - b))

class VFHPlus:
    def __init__(
        self,
        *,
        sector_deg: float = 5.0,             # histogram sector width
        r_min: float = 0.35,                  # beams >= robot edge
        r_max: float = 6.0,                  # max active range
        robot_radius: float = 0.40,          # body radius
        safety_margin: float = 0.25,         # extra safety
        tau_low: float =  65.0,             # hysteresis thresholds
        tau_high: float = 70.0,
        opening_narrow_sectors: int = 12,    # maximum number of sectors in narrow opening 
        mu_goal: float = 6.0,                # VFH+ cost weights
        mu_smooth: float = 2.0,
        mu_commit: float = 2.0,
        step_min: float = 0.10,              # min/max waypoint step
        step_max: float = 0.35
    ):
        # --- histogram geometry ---
        self.sector_width = math.radians(sector_deg)
        self.num_sectors = int(round(2 * math.pi / self.sector_width))
        self.edges = -math.pi + self.sector_width * np.arange(self.num_sectors)
        self.centers = self.edges + 0.5 * self.sector_width

        # --- parameters ---
        self.r_min = float(r_min)
        self.r_max = float(r_max)
        self.robot_radius = float(robot_radius)
        self.safety_margin = float(safety_margin)
        self.expand_radius = self.robot_radius + self.safety_margin
        self.tau_low = float(tau_low)
        self.tau_high = float(tau_high)
        self.smax = int(opening_narrow_sectors)
        self.mu1, self.mu2, self.mu3 = float(mu_goal), float(mu_smooth), float(mu_commit)
        assert self.mu1 > (self.mu2 + self.mu3) - 1e-9, "Set mu_goal > mu_smooth + mu_commit (VFH+ req.)"
        self.step_min, self.step_max = float(step_min), float(step_max)

        
        self.prev_mask = None              # Stage-2 memory (bool array)
        self.prev_dir_bl = 0.0             # last commanded dir in base_link
        self.cached_scan_frame = None
        '''
        self.kappa   = 4.0            # m(0) = kappa * c^2 ; m(R) = 1 * c^2
        self.a       = self.kappa
        self.b       = (self.kappa - 1.0) / max(self.r_max**2, 1e-9)
        '''
        self.E = 4.0
        self.B = 16.0
        self.D = 10.0

    def _magnitude(self, c: float, d: float) -> float:
        # obstacle vector magnitude
        #m = (c * c) * (self.a - self.b * d * d)
        m = (c * c) * (math.exp((-1/self.B) * (math.pow((d/self.D),self.E))))
        return float(max(0.0, m))

    # Stage 1: Primary polar histogram 
    def _angles_from_scan(self, scan):
        n = len(scan.ranges)
        return scan.angle_min + np.arange(n) * scan.angle_increment

    def _primary_histogram(self, ranges: np.ndarray, angles_bl: np.ndarray) -> np.ndarray:
        """
        Build H' per VFH+:
        - keep beams in [r_min, r_max]
        - each beam contributes weight m to sectors in [beta - gamma, beta + gamma]
        - gamma = asin(expand_radius / r) 
        """
        S = self.num_sectors
        H = np.zeros(S, dtype=float)

        finite = np.isfinite(ranges)
        mask = finite & (ranges >= self.r_min) & (ranges <= self.r_max)
        if not np.any(mask):
            return H

        r = ranges[mask].astype(float)
        th = angles_bl[mask].astype(float)
        # wrap to (-pi, pi]
        th = (th + np.pi) % (2*np.pi) - np.pi

        # enlargement half-angle per beam
        with np.errstate(invalid='ignore'):
            ratio = np.clip(self.expand_radius / np.maximum(r, 1e-6), 0.0, 1.0)
            gamma = np.arcsin(ratio)

        # weighs closer beams higher 
        w = np.array([self._magnitude(1.0, float(ri)) for ri in r], dtype=float)

        # accumulate contributions
        sec_w = self.sector_width
        def ang2k(a):
            return (np.floor((a + np.pi) / sec_w).astype(int)) % S

        aL = th - gamma
        aR = th + gamma
        kL = ang2k(aL)
        kR = ang2k(aR)

        for i in range(len(r)):
            m = w[i]
            l = int(kL[i]); rr = int(kR[i])
            if l <= rr:
                H[l:rr+1] += m
            else:
                H[l:] += m
                H[:rr+1] += m

        return H

    # Stage 2: Binary polar histogram (hysteresis)
    def _binary_histogram(self, H: np.ndarray) -> np.ndarray:
        """Return mask_free: True = free, False = blocked."""
        if self.prev_mask is None:
            # initialize: free where safely below high threshold
            self.prev_mask = (H < self.tau_high)
        blocked_now = H > self.tau_high
        free_now    = H < self.tau_low
        out = self.prev_mask.copy()
        out[blocked_now] = False
        out[free_now]    = True
        self.prev_mask = out
        return out

   
    # Stage 3: Candidate selection & cost
    def _find_openings(self, mask_free: np.ndarray):
        S = len(mask_free)
        openings = []
        i = 0
        while i < S:
            if mask_free[i]:
                start = i
                while i < S and mask_free[i]:
                    i += 1
                end = (i - 1) % S
                openings.append((start, end))
            else:
                i += 1
        return openings

    def _opening_width(self, kl: int, kr: int) -> int:
        if kl <= kr:
            return kr - kl + 1
        return kr + self.num_sectors - kl + 1

    def _opening_center(self, kl: int, kr: int) -> int:
        S = self.num_sectors
        if kl <= kr:
            return (kl + kr) // 2
        return ((kl + (kr + S)) // 2) % S

    def _in_opening(self, k: int, kl: int, kr: int) -> bool:
        if kl <= kr:
            return (k >= kl) and (k <= kr)
        return (k >= kl) or (k <= kr)

    def _pick_direction(self, mask_free: np.ndarray, target_dir_bl: float, H_primary: np.ndarray) -> float:
        """
        Return best steering direction in base_link, radians.
        Implements wide/narrow candidate rules + VFH+ cost (mu1,mu2,mu3).
        """
        S = self.num_sectors
        openings = self._find_openings(mask_free)
        if not openings:
            # fallback: choose least cluttered sector
            k = int(np.argmin(H_primary))
            return float(self.centers[k])

        # target/current/previous in sector indices
        k_t = int(round(((target_dir_bl + math.pi) / self.sector_width))) % S
        k_theta = 0  # by definition base_link's forward is 0 rad
        k_prev = int(round(((self.prev_dir_bl + math.pi) / self.sector_width))) % S

        candidates = []
        for kl, kr in openings:
            w = self._opening_width(kl, kr)
            if w <= self.smax:
                candidates.append(self._opening_center(kl, kr))
            else:
                # right-side and left-side
                candidates.append((kl + self.smax // 2) % S)
                candidates.append((kr - self.smax // 2) % S)
                # target if inside gap
                if self._in_opening(k_t, kl, kr):
                    candidates.append(k_t)

        # Score with VFH+ cost
        cand_arr = np.asarray(candidates, dtype=int)
        print(f"Candidates: {(-math.pi + self.sector_width * cand_arr)*(180/math.pi)}")
        print(f"k_t = {(-math.pi + self.sector_width * k_t)*(180/math.pi)}")
        mu1, mu2, mu3 = self.mu1, self.mu2, self.mu3
        def gap(a, b):  # in sectors
            d = (a - b) % S
            d = min(d, S - d)
            return d * self.sector_width

        best_k = min(candidates, key=lambda c: mu1 * gap(c, k_t) +
                                        mu2 * gap(c, k_theta) +
                                        mu3 * gap(c, k_prev))
        print(f"best_k: {(-math.pi + self.sector_width * best_k)*(180/math.pi)}")
        return float(self.centers[best_k])

    # Utilities 
    def _free_distance_along(self, ranges: np.ndarray, angles_bl: np.ndarray,
                             dir_bl: float, tol_rad: float) -> float:
        finite = np.isfinite(ranges)
        if not np.any(finite):
            return self.step_min
        a = angles_bl[finite]
        r = ranges[finite]
        d = np.abs(((a - dir_bl + np.pi) % (2*np.pi)) - np.pi)
        mask = (d <= tol_rad) & (r >= self.r_min)
        if not np.any(mask):
            return self.step_min
        rmin = float(np.clip(np.min(r[mask]) - self.expand_radius, 0.0, self.r_max))
        return float(np.clip(0.5 * rmin, self.step_min, self.step_max))


    def plan(self, *, scan, robot_xy_yaw_map, goal_xy_map,
             dt: float = 0.05, v_xy: float = 0.0,
             yaw_rate_max: float = 2.0):
        """
        Returns: (wp_x_map, wp_y_map, yaw_cmd_map)

        - scan: sensor_msgs/LaserScan
        - robot_xy_yaw_map: (rx, ry, yaw_map)
        - goal_xy_map: (gx, gy)
        - dt, v_xy, yaw_rate_max,
        """
        rx, ry, yaw_map = robot_xy_yaw_map
        gx, gy = goal_xy_map

        angles_bl = self._angles_from_scan(scan)
        ranges = np.asarray(scan.ranges, dtype=float)

        # Stage 1
        H_primary = self._primary_histogram(ranges, angles_bl)
        
        print("Polar hist: ")
        print(H_primary)
        '''
        plt.bar(np.degrees(self.centers), H_primary, width=np.degrees(self.sector_width))
        plt.title("Primary Histogram")
        plt.show()
        '''

        # Stage 2
        mask_free = self._binary_histogram(H_primary)  # True=free

        # Stage 3
        # target direction in base_link
        ang_to_goal_map = math.atan2(gy - ry, gx - rx)
        target_dir_bl = _wrap_pi(ang_to_goal_map - yaw_map)

        dir_bl = self._pick_direction(mask_free, target_dir_bl, H_primary)
        print(f"Picked direction : {dir_bl* (180/3.14)}")

        # step size from free distance within small angular tolerance 
        step = self._free_distance_along(ranges, angles_bl, dir_bl,
                                         tol_rad=0.5 * self.sector_width)

        # convert to MAP waypoint/yaw
        yaw_cmd_map = _wrap_pi(yaw_map + dir_bl)
        wp_x = rx + step * math.cos(yaw_cmd_map)
        wp_y = ry + step * math.sin(yaw_cmd_map)
        print(f"Robots x,y: {rx}, {ry}")
        print(f"Step = {step}")
        print(f"yaw_cmd_map: {yaw_cmd_map}")
        print(f"Added term x, y: {math.cos(yaw_cmd_map)}, {math.sin(yaw_cmd_map)}")
        return float(wp_x), float(wp_y), float(yaw_cmd_map)
