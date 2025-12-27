"""
Circuit track scenario using pyclothoids for smooth clothoid-based track geometry.

Provides:
- CircuitTrack: Core geometry class with arc-length parameterized queries
- CircuitTrackController: Path-following controller
- CircuitTrackScenario: Complete scenario with cones and vehicle
"""

import math
import bisect
from typing import List, Tuple

from pyclothoids import Clothoid

from pydrivingsim import TrafficCone, Vehicle, Coin, Agent


def _wrap_angle(value: float) -> float:
    """Wrap angle to [-pi, pi]."""
    return math.atan2(math.sin(value), math.cos(value))


def _clamp(value: float, min_value: float, max_value: float) -> float:
    """Clamp value to [min_value, max_value]."""
    return max(min_value, min(max_value, value))


class CircuitTrack:
    """
    Closed-circuit race track represented as a sequence of clothoid segments.

    Provides arc-length parameterized access to:
    - Position (x, y)
    - Heading (theta)
    - Curvature (kappa)
    - Closest point projection

    Example:
        segments = [...]  # List of pyclothoids.Clothoid objects
        track = CircuitTrack(segments, track_width=5.0)

        # Get position at 50 meters along track
        x, y = track.position(50.0)

        # Project vehicle position onto track
        s, lat_error, heading = track.project(veh_x, veh_y)
    """

    def __init__(self, segments: List[Clothoid], track_width: float = 4.0):
        """
        Initialize track from a list of clothoid segments.

        Args:
            segments: List of Clothoid objects forming a closed loop.
                      The end of the last segment should connect to the start
                      of the first segment (within tolerance).
            track_width: Full width of the track in meters.
        """
        if not segments:
            raise ValueError("At least one segment is required")

        self._segments = segments
        self._track_width = track_width

        # Build cumulative length array for O(log n) segment lookup
        self._cumulative_lengths = [0.0]
        for seg in segments:
            self._cumulative_lengths.append(
                self._cumulative_lengths[-1] + seg.length
            )
        self._total_length = self._cumulative_lengths[-1]

        # Validate closure: end of last segment should connect to start of first
        end_x, end_y = segments[-1].XEnd, segments[-1].YEnd
        end_theta = segments[-1].ThetaEnd
        start_x, start_y = segments[0].XStart, segments[0].YStart
        start_theta = segments[0].ThetaStart

        pos_error = math.sqrt((end_x - start_x)**2 + (end_y - start_y)**2)
        angle_error = abs(_wrap_angle(end_theta - start_theta))

        if pos_error > 0.1:  # 10cm tolerance
            print(f"Warning: Track closure position error: {pos_error:.3f}m")
        if angle_error > 0.05:  # ~3 degrees tolerance
            print(f"Warning: Track closure angle error: {math.degrees(angle_error):.2f} deg")

    @property
    def total_length(self) -> float:
        """Total arc length of the track (one full lap)."""
        return self._total_length

    @property
    def track_width(self) -> float:
        """Full track width in meters."""
        return self._track_width

    @property
    def num_segments(self) -> int:
        """Number of clothoid segments."""
        return len(self._segments)

    def _wrap_s(self, s: float) -> float:
        """Wrap arc-length to [0, total_length)."""
        if self._total_length == 0:
            return 0.0
        s = s % self._total_length
        if s < 0:
            s += self._total_length
        return s

    def _find_segment(self, s: float) -> Tuple[int, float]:
        """
        Find which segment contains arc-length s.

        Returns:
            (segment_index, local_s) where local_s is the arc-length
            within that segment.
        """
        s = self._wrap_s(s)
        # Binary search to find segment index
        idx = bisect.bisect_right(self._cumulative_lengths, s) - 1
        idx = max(0, min(idx, len(self._segments) - 1))
        local_s = s - self._cumulative_lengths[idx]
        # Clamp local_s to segment length to avoid numerical issues
        local_s = min(local_s, self._segments[idx].length)
        return idx, local_s

    def position(self, s: float) -> Tuple[float, float]:
        """
        Get centerline position at arc-length s.

        Args:
            s: Arc-length from start (wraps around at total_length).

        Returns:
            (x, y) coordinates of centerline.
        """
        idx, local_s = self._find_segment(s)
        seg = self._segments[idx]
        return (seg.X(local_s), seg.Y(local_s))

    def heading(self, s: float) -> float:
        """
        Get track heading (tangent angle) at arc-length s.

        Args:
            s: Arc-length from start (wraps around).

        Returns:
            Heading angle in radians.
        """
        idx, local_s = self._find_segment(s)
        seg = self._segments[idx]
        return seg.Theta(local_s)

    def curvature(self, s: float) -> float:
        """
        Get curvature at arc-length s.

        Args:
            s: Arc-length from start (wraps around).

        Returns:
            Curvature (1/radius). Positive = left turn, negative = right turn.
        """
        idx, local_s = self._find_segment(s)
        seg = self._segments[idx]
        # Curvature = kappa_start + dk * local_s
        return seg.KappaStart + seg.dk * local_s

    def reference_at(self, s: float) -> Tuple[float, float, float, float]:
        """
        Get full reference state at arc-length s.

        Args:
            s: Arc-length from start (wraps around).

        Returns:
            (x, y, heading, curvature) tuple.
        """
        idx, local_s = self._find_segment(s)
        seg = self._segments[idx]
        x = seg.X(local_s)
        y = seg.Y(local_s)
        theta = seg.Theta(local_s)
        kappa = seg.KappaStart + seg.dk * local_s
        return (x, y, theta, kappa)

    def project(self, x: float, y: float, s_hint: float = None,
                search_window: float = None) -> Tuple[float, float, float]:
        """
        Project a point onto the track centerline.

        This is the critical method for controller feedback - finds the
        closest point on the track and returns progress + lateral error.

        Args:
            x, y: World coordinates to project.
            s_hint: Optional hint for expected arc-length position. When provided,
                    search is constrained to a window around this value to avoid
                    matching the wrong track section when the track loops back.
            search_window: Size of search window around s_hint (default: 50m).
                          Only used when s_hint is provided.

        Returns:
            (s, lateral_error, heading_at_s) tuple where:
            - s: Arc-length of closest point on centerline
            - lateral_error: Signed distance from centerline
              (positive = left of track, negative = right)
            - heading_at_s: Track heading at the closest point
        """
        sample_spacing = 1.0  # meters

        if s_hint is not None:
            # Constrained search around the hint position
            if search_window is None:
                search_window = 50.0  # Default 50m window

            # Search only within the window around s_hint
            half_window = search_window / 2.0
            s_start = s_hint - half_window
            s_end = s_hint + half_window

            best_s = s_hint
            best_dist_sq = float('inf')

            # Sample within the window
            num_samples = int(search_window / sample_spacing) + 1
            for i in range(num_samples):
                s_test = self._wrap_s(s_start + i * sample_spacing)
                px, py = self.position(s_test)
                dist_sq = (x - px)**2 + (y - py)**2
                if dist_sq < best_dist_sq:
                    best_dist_sq = dist_sq
                    best_s = s_test
        else:
            # Full track search (original behavior)
            num_samples = int(self._total_length / sample_spacing) + 1

            best_s = 0.0
            best_dist_sq = float('inf')

            for i in range(num_samples):
                s_test = i * sample_spacing
                px, py = self.position(s_test)
                dist_sq = (x - px)**2 + (y - py)**2
                if dist_sq < best_dist_sq:
                    best_dist_sq = dist_sq
                    best_s = s_test

        # Fine search: refine within local neighborhood using golden section
        search_radius = sample_spacing * 1.5
        s_low = self._wrap_s(best_s - search_radius)
        s_high = self._wrap_s(best_s + search_radius)

        # Handle wrap-around case
        if s_high < s_low:
            # Search crosses the start/finish line
            # Try both regions and pick the better one
            best_s = self._refine_projection(x, y, 0.0, s_high)
            best_s2 = self._refine_projection(x, y, s_low, self._total_length)

            px1, py1 = self.position(best_s)
            px2, py2 = self.position(best_s2)
            if (x - px2)**2 + (y - py2)**2 < (x - px1)**2 + (y - py1)**2:
                best_s = best_s2
        else:
            best_s = self._refine_projection(x, y, s_low, s_high)

        # Compute lateral error (signed distance)
        px, py = self.position(best_s)
        theta = self.heading(best_s)

        # Vector from track point to query point
        dx, dy = x - px, y - py

        # Cross product for signed distance (positive = left of track direction)
        lateral_error = -dx * math.sin(theta) + dy * math.cos(theta)

        return best_s, lateral_error, theta

    def _refine_projection(self, x: float, y: float, s_low: float, s_high: float) -> float:
        """Refine projection using golden section search."""
        golden_ratio = (math.sqrt(5) - 1) / 2

        tolerance = 0.01  # 1cm accuracy

        a, b = s_low, s_high
        c = b - golden_ratio * (b - a)
        d = a + golden_ratio * (b - a)

        def dist_sq(s):
            px, py = self.position(s)
            return (x - px)**2 + (y - py)**2

        while abs(b - a) > tolerance:
            if dist_sq(c) < dist_sq(d):
                b = d
                d = c
                c = b - golden_ratio * (b - a)
            else:
                a = c
                c = d
                d = a + golden_ratio * (b - a)

        return (a + b) / 2

    def sample_centerline(self, num_points: int = 200) -> List[Tuple[float, float]]:
        """Sample centerline points for visualization."""
        points = []
        for i in range(num_points):
            s = (i / num_points) * self._total_length
            points.append(self.position(s))
        return points

    def sample_boundaries(self, num_points: int = 200) -> Tuple[List[Tuple[float, float]], List[Tuple[float, float]]]:
        """
        Sample left and right boundary points.

        Returns:
            (left_boundary_points, right_boundary_points)
        """
        left_points = []
        right_points = []
        half_width = self._track_width / 2.0

        for i in range(num_points):
            s = (i / num_points) * self._total_length
            cx, cy = self.position(s)
            theta = self.heading(s)

            # Normal vector (perpendicular to track direction)
            nx = -math.sin(theta)
            ny = math.cos(theta)

            left_points.append((cx + nx * half_width, cy + ny * half_width))
            right_points.append((cx - nx * half_width, cy - ny * half_width))

        return left_points, right_points


class CircuitTrackController:
    """
    Path-following controller for closed-circuit tracks.

    Uses lateral error and heading error feedback similar to
    SinusoidalTrackController, but works with arc-length parameterized tracks.
    """

    def __init__(self, vehicle: Vehicle, track: CircuitTrack,
                 min_speed: float = 6.0, max_speed: float = 15.0,
                 search_window: float = 50.0):
        """
        Args:
            vehicle: Vehicle instance to control.
            track: CircuitTrack instance for reference generation.
            min_speed: Minimum target speed (m/s).
            max_speed: Maximum target speed (m/s).
            search_window: Search window size for track projection (m).
                          Constrains projection to avoid matching wrong track
                          section when track loops back close to itself.
        """
        self.vehicle = vehicle
        self.track = track
        self.min_speed = min_speed
        self.max_speed = max_speed
        self.search_window = search_window

        # Controller gains (matching SinusoidalTrackController)
        self.k_lat = 2.0
        self.k_heading = -1.5
        self.k_speed = 0.6
        self.speed_gain = 15.0  # Curvature-to-speed reduction factor

        # Track progress state
        self._last_s = 0.0
        self._lap_count = 0
        self._initialized = False

    def compute_action(self) -> List[float]:
        """
        Compute steering and acceleration commands.

        Returns:
            [acceleration, steering] control action.
        """
        state, _ = self.vehicle.get_state()
        if state is None:
            return [0.0, 0.0]  # Default: no action if state unavailable
        x_pos, y_pos, yaw = float(state[0]), float(state[1]), float(state[2])
        vel = float(state[3])

        # Project vehicle position onto track
        # Use last known s as hint to avoid matching wrong track section
        if self._initialized:
            s, lateral_error, track_heading = self.track.project(
                x_pos, y_pos, s_hint=self._last_s, search_window=self.search_window
            )
        else:
            # First call: search entire track
            s, lateral_error, track_heading = self.track.project(x_pos, y_pos)
            self._initialized = True

        # Update progress tracking (detect lap completion)
        if s < self._last_s - self.track.total_length / 2:
            self._lap_count += 1
        self._last_s = s

        # Get curvature for speed adaptation
        curvature = self.track.curvature(s)

        # Heading error (wrap to [-pi, pi])
        heading_error = _wrap_angle(track_heading - yaw)

        # Steering control (Stanley-like)
        steer = self.k_lat * lateral_error + self.k_heading * heading_error
        steer = _clamp(steer, -0.6, 0.6)

        # Speed control (reduce speed in curves)
        speed_ref = self.max_speed - self.speed_gain * abs(curvature)
        speed_ref = _clamp(speed_ref, self.min_speed, self.max_speed)
        accel = self.k_speed * (speed_ref - vel)
        accel = _clamp(accel, -3.0, 3.0)

        return [accel, steer]

    @property
    def progress(self) -> float:
        """Current progress as arc-length from start."""
        return self._last_s

    @property
    def laps_completed(self) -> int:
        """Number of complete laps."""
        return self._lap_count


class CircuitTrackScenario:
    """
    Closed-circuit race track scenario using external agent controller.

    The controller logic runs in the external C++ agent (basic_agent_st).
    This scenario sets up the track geometry, places cones and coins,
    and communicates with the external agent via the Agent class.

    Usage:
        scenario = CircuitTrackScenario.oval(straight_length=80, turn_radius=25)
        # or
        scenario = CircuitTrackScenario.figure_eight(size=50)
        # or
        track = create_custom_track(...)
        scenario = CircuitTrackScenario(track)
    """

    def __init__(self, track: CircuitTrack,
                 cone_spacing: float = 3.0,
                 place_coins: bool = True,
                 **kwargs):
        """
        Args:
            track: CircuitTrack geometry.
            cone_spacing: Distance between cones along track (meters).
            place_coins: Whether to place coins along centerline for lane tracking.
            **kwargs: Additional arguments (ignored, for compatibility).
        """
        self.track = track

        # Initialize vehicle at track start
        self.vehicle = Vehicle()
        x0, y0 = track.position(0)
        theta0 = track.heading(0)
        self.vehicle.set_pos_ang((x0, y0, theta0))
        self.vehicle.set_screen_here()

        # Initialize external agent (C++ controller in basic_agent_st)
        self.agent = Agent(self.vehicle, track=self.track)

        # Place cones along track boundaries
        self._place_cones(cone_spacing)

        # Place coins along centerline (used for lane tracking by the agent)
        if place_coins:
            self._place_coins(cone_spacing * 2)

    def _place_cones(self, spacing: float):
        """Place traffic cones along both track boundaries."""
        half_width = self.track.track_width / 2.0
        num_cones = int(self.track.total_length / spacing)

        for i in range(num_cones):
            s = i * spacing
            cx, cy = self.track.position(s)
            theta = self.track.heading(s)

            # Normal vector (perpendicular to track direction)
            nx = -math.sin(theta)
            ny = math.cos(theta)

            # Left boundary cone
            cone_left = TrafficCone()
            cone_left.set_pos((cx + nx * half_width, cy + ny * half_width))

            # Right boundary cone
            cone_right = TrafficCone()
            cone_right.set_pos((cx - nx * half_width, cy - ny * half_width))

    def _place_coins(self, spacing: float):
        """Place coins along centerline for visual reference."""
        num_coins = int(self.track.total_length / spacing)
        for i in range(num_coins):
            s = i * spacing
            cx, cy = self.track.position(s)
            coin = Coin()
            coin.set_pos((cx, cy))

    def update(self):
        """Called each simulation step."""
        self.agent.compute()
        action = self.agent.get_action()

        self.vehicle.set_screen_here()
        self.vehicle.control([action[0], action[1]])

    def terminate(self):
        """Called on scenario end."""
        self.agent.terminate()

    # === Factory Methods for Preset Tracks ===

    @classmethod
    def oval(cls, straight_length: float = 80.0, turn_radius: float = 25.0,
             track_width: float = 5.0, **kwargs):
        """Create an oval track scenario."""
        from scenarios.circuit_presets import create_oval_track
        track = create_oval_track(straight_length, turn_radius, track_width)
        return cls(track, **kwargs)

    @classmethod
    def figure_eight(cls, size: float = 50.0, track_width: float = 5.0, **kwargs):
        """Create a figure-8 track scenario."""
        from scenarios.circuit_presets import create_figure_eight_track
        track = create_figure_eight_track(size, track_width)
        return cls(track, **kwargs)

    @classmethod
    def racetrack(cls, scale: float = 1.0, track_width: float = 5.0, **kwargs):
        """Create a more complex racetrack scenario."""
        from scenarios.circuit_presets import create_racetrack
        track = create_racetrack(scale, track_width)
        return cls(track, **kwargs)

    @classmethod
    def sinusoidal(cls, amplitude: float = 3.0, wavelength: float = 40.0,
                   length: float = 200.0, track_width: float = 4.0, **kwargs):
        """Create a sinusoidal track scenario."""
        from scenarios.circuit_presets import create_sinusoidal_track
        track = create_sinusoidal_track(amplitude, wavelength, length, track_width)
        return cls(track, **kwargs)

    @classmethod
    def halfcircle(cls, radius: float = 30.0, straight_length: float = 20.0,
                   track_width: float = 5.0, **kwargs):
        """
        Create a half-circle track scenario for basic testing.

        Simple track with straight entry, 180-degree turn, straight return,
        and another 180-degree turn to close the loop.

        Args:
            radius: Radius of the semicircular turns (m).
            straight_length: Length of straight sections (m).
            track_width: Full width of track (m).
            **kwargs: Additional arguments passed to scenario constructor.

        Returns:
            CircuitTrackScenario instance.
        """
        from scenarios.circuit_presets import create_halfcircle_track
        track = create_halfcircle_track(radius, straight_length, track_width)
        return cls(track, **kwargs)
