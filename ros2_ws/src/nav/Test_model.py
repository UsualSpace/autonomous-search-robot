#!/usr/bin/env python3
"""
test_integration.py
 
Full system integration test for the object-finding robot.
Launches launch_sim.py and nav_stack.py automatically.
 
Usage:
    # Gazebo must be running on host first, then:
    python3 test_integration.py --target "person"
    python3 test_integration.py --target "chair" --results-file results.json
    python3 test_integration.py --test-all --results-file results.json
    python3 test_integration.py --list-objects
"""
 
import argparse
import json
import subprocess
import sys
import time
import threading
from datetime import datetime
from pathlib import Path
 
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from nav_msgs.msg import Odometry
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Twist
 
 
# ─── Allowed objects ────────────────────────────────────────────────────────
ALLOWED_OBJECTS = [
    "backpack",
    "bicycle",
    "refrigerator",
    "teddy bear",
    # Add more as needed
]
 
# ─── Paths ──────────────────────────────────────────────────────────────────
NAV_STACK    = Path("/DEV_WS/ros2_ws/src/nav/nav_stack.py")
LAUNCH_PKG   = "my_robot_description"
LAUNCH_FILE  = "launch_sim.py"
 
# ─── Timing ─────────────────────────────────────────────────────────────────
DEFAULT_TIMEOUT        = 120  # seconds for mission
LAUNCH_SIM_WAIT_SEC    = 8     # wait after launch_sim.py starts
NAV_STACK_WAIT_SEC     = 6     # wait after nav_stack.py starts
ROBOT_STOP_TOLERANCE   = 0.02  # m/s below this = stopped
ROBOT_STOP_DURATION    = 3.0   # seconds must stay stopped to confirm
 
 

# ─── Mission monitor node ────────────────────────────────────────────────────
class MissionMonitor(Node):
    def __init__(self):
        super().__init__('integration_test_monitor')
 
        self.found = False
        self.found_time = None
        self.map_received = False
        self.map_width = 0
        self.map_height = 0
        self.robot_stopped = False
        self.robot_stop_start = None
        self.odom_positions = []
 
        self.create_subscription(Bool, '/detections/found', self._on_found, 10)
        self.create_subscription(OccupancyGrid, '/map', self._on_map, 10)
        self.create_subscription(Odometry, '/odom', self._on_odom, 10)
        self.create_subscription(Twist, '/cmd_vel', self._on_cmd_vel, 10)
 
    def _on_found(self, msg):
        if msg.data and not self.found:
            self.found = True
            self.found_time = time.time()
            self.get_logger().info('✓ /detections/found = True')
 
    def _on_map(self, msg):
        self.map_received = True
        self.map_width = msg.info.width
        self.map_height = msg.info.height
 
    def _on_odom(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        self.odom_positions.append((x, y))
 
    def _on_cmd_vel(self, msg):
        moving = (abs(msg.linear.x) > ROBOT_STOP_TOLERANCE or
                  abs(msg.angular.z) > ROBOT_STOP_TOLERANCE)
        if not moving:
            if self.robot_stop_start is None:
                self.robot_stop_start = time.time()
            elif time.time() - self.robot_stop_start >= ROBOT_STOP_DURATION:
                self.robot_stopped = True
        else:
            self.robot_stop_start = None
            self.robot_stopped = False
 
    @property
    def robot_moved(self):
        if len(self.odom_positions) < 2:
            return False
        xs = [p[0] for p in self.odom_positions]
        ys = [p[1] for p in self.odom_positions]
        dist = ((max(xs)-min(xs))**2 + (max(ys)-min(ys))**2) ** 0.5
        return dist > 0.1
 
 
# ─── Integration test ────────────────────────────────────────────────────────
class IntegrationTest:
    def __init__(self, target: str, timeout: int,
                 disable_completion_stop: bool = True,
                 results_file: Path = None,
                 world: str = 'RG1'):
        self.target = target
        self.timeout = timeout
        self.disable_completion_stop = disable_completion_stop
        self.results_file = results_file
        self.world = world
        self.launch_sim_proc = None
        self.nav_proc = None
        self.results = {}
 
    # ── Process management ───────────────────────────────────────────────────
 
    def _launch_sim(self):
        """Start launch_sim.py with the target object as user_input."""
        print(f"  Starting launch_sim.py with user_input={self.target}...")
        self.launch_sim_proc = subprocess.Popen([
            'ros2', 'launch', LAUNCH_PKG, LAUNCH_FILE,
            f'user_input:={self.target}',
        ])
        time.sleep(LAUNCH_SIM_WAIT_SEC)
        print("  ✓ launch_sim.py started")
 
    def _launch_nav_stack(self):
        """Start nav_stack.py."""
        print("  Starting nav_stack.py...")
        cmd = ['python3', str(NAV_STACK), '--no-stop-on-exit']
        if self.disable_completion_stop:
            cmd.append('--disable-completion-stop')
        self.nav_proc = subprocess.Popen(cmd)
        time.sleep(NAV_STACK_WAIT_SEC)
        print("  ✓ nav_stack.py started")
 
    def _stop_all(self):
        """Terminate both processes cleanly."""
        for name, proc in [('nav_stack', self.nav_proc),
                            ('launch_sim', self.launch_sim_proc)]:
            if proc and proc.poll() is None:
                print(f"  Stopping {name}...")
                proc.terminate()
                try:
                    proc.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    proc.kill()
 
    def _stop_robot(self):
        subprocess.run([
            'ros2', 'topic', 'pub', '--once', '/cmd_vel',
            'geometry_msgs/msg/Twist',
            '{linear: {x: 0.0}, angular: {z: 0.0}}'
        ], capture_output=True)
 
    # ── Run ──────────────────────────────────────────────────────────────────
 
    def run(self) -> bool:
        print(f"\n{'='*60}")
        print(f"  TARGET:              '{self.target}'")
        print(f"  TIMEOUT:             {self.timeout}s")
        print(f"  COMPLETION_STOP_OFF: {self.disable_completion_stop}")
        print(f"{'='*60}")
 
        if self.target not in ALLOWED_OBJECTS:
            print(f"\n  ERROR: '{self.target}' is not in ALLOWED_OBJECTS.")
            print(f"  Allowed: {ALLOWED_OBJECTS}")
            return False
 
        
 
        # Launch everything
        self._launch_sim()
        self._launch_nav_stack()
 
        # ROS monitor
        rclpy.init()
        monitor = MissionMonitor()
        spin_thread = threading.Thread(
            target=lambda: rclpy.spin(monitor), daemon=True
        )
        spin_thread.start()
 
        print(f"\n  Mission running — monitoring for up to {self.timeout}s...\n")
        start = time.time()
        last_print = 0
 
        try:
            while time.time() - start < self.timeout:
                elapsed = time.time() - start
 
                if elapsed - last_print >= 10:
                    last_print = elapsed
                    moved   = "yes" if monitor.robot_moved else "no"
                    map_info = f"{monitor.map_width}x{monitor.map_height}" \
                               if monitor.map_received else "none"
                    found   = "YES ✓" if monitor.found else "no"
                    print(f"  [{elapsed:5.0f}s] map={map_info}  "
                          f"moved={moved}  found={found}")
 
                if monitor.found and monitor.robot_stopped:
                    print(f"\n  ✓ Mission SUCCESS at {elapsed:.1f}s")
                    break
 
                time.sleep(0.5)
 
        finally:
            self._stop_robot()
            self._stop_all()
 
        elapsed = time.time() - start
        self.results = {
            'timestamp':      datetime.now().isoformat(),
            'target':         self.target,
            'timeout':        self.timeout,
            'duration_sec':   round(elapsed, 1),
            'found_signal':   monitor.found,
            'found_time_sec': round(monitor.found_time - start, 1)
                              if monitor.found_time else None,
            'robot_stopped':  monitor.robot_stopped,
            'robot_moved':    monitor.robot_moved,
            'map_received':   monitor.map_received,
            'map_size':       f"{monitor.map_width}x{monitor.map_height}",
            'timed_out':      elapsed >= self.timeout,
        }
 
        monitor.destroy_node()
        rclpy.shutdown()
 
        passed = self._evaluate()
        self.results['passed'] = passed
 
        if self.results_file:
            self._write_results()
 
        return passed
 
    # ── Evaluate ─────────────────────────────────────────────────────────────
 
    def _evaluate(self) -> bool:
        r = self.results
        passed = True
        issues = []
 
        print(f"\n{'─'*60}")
        print("  RESULTS")
        print(f"{'─'*60}")
 
        if not r['map_received']:
            issues.append("SLAM never published a map — check TF chain")
            passed = False
        else:
            print(f"  ✓ Map published ({r['map_size']})")
 
        if not r['robot_moved']:
            issues.append("Robot never moved — check nav_explorer is receiving scan/map")
            passed = False
        else:
            print(f"  ✓ Robot moved during exploration")
 
        if not r['found_signal']:
            issues.append(f"'{r['target']}' not detected within {r['duration_sec']}s")
            passed = False
        else:
            print(f"  ✓ Object found at {r['found_time_sec']}s")
 
        if r['found_signal'] and not r['robot_stopped']:
            issues.append("Found signal received but robot did not stop")
            passed = False
        elif r['found_signal'] and r['robot_stopped']:
            print(f"  ✓ Robot stopped after finding object")
 
        if r['timed_out']:
            issues.append(f"Mission timed out after {r['timeout']}s")
            passed = False
 
        print(f"{'─'*60}")
        if passed:
            print(f"  OVERALL: PASS ✓  ({r['duration_sec']}s)")
        else:
            print(f"  OVERALL: FAIL ✗")
            for issue in issues:
                print(f"    • {issue}")
        print(f"{'─'*60}\n")
 
        return passed
 
    # ── Save results ─────────────────────────────────────────────────────────
 
    def _write_results(self):
        path = Path(self.results_file)
 
        # JSON — all runs in one array
        all_results = []
        if path.exists():
            try:
                with open(path) as f:
                    all_results = json.load(f)
            except (json.JSONDecodeError, IOError):
                all_results = []
        all_results.append(self.results)
        with open(path, 'w') as f:
            json.dump(all_results, f, indent=2)
        print(f"  Results saved → {path.resolve()}")
 
        # Text summary — one line per run
        txt_path = path.with_suffix('.txt')
        r = self.results
        status = "PASS" if r['passed'] else "FAIL"
        with open(txt_path, 'a') as f:
            f.write(
                f"[{r['timestamp']}] {status} | "
                f"target={r['target']} | "
                f"found={r['found_signal']} | "
                f"found_at={r['found_time_sec']}s | "
                f"duration={r['duration_sec']}s | "
                f"map={r['map_size']} | "
                f"timed_out={r['timed_out']}\n"
            )
        print(f"  Summary appended → {txt_path.resolve()}")
 
 
# ─── Run all objects ─────────────────────────────────────────────────────────
def run_all_tests(timeout: int, disable_completion_stop: bool,
                  results_file: Path, world: str):
    print(f"\nRunning tests for all {len(ALLOWED_OBJECTS)} allowed objects...\n")
    passed_list = []
    failed_list = []
 
    for obj in ALLOWED_OBJECTS:
        test = IntegrationTest(obj, timeout, disable_completion_stop,
                               results_file, world)
        success = test.run()
        if success:
            passed_list.append(obj)
        else:
            failed_list.append(obj)
        print("  Waiting 5s before next test...\n")
        time.sleep(5)
 
    print(f"\n{'='*60}")
    print(f"  SUMMARY: {len(passed_list)}/{len(ALLOWED_OBJECTS)} passed")
    if passed_list:
        print(f"  Passed: {passed_list}")
    if failed_list:
        print(f"  Failed: {failed_list}")
    print(f"{'='*60}\n")
 
    return len(failed_list) == 0
 
 
# ─── Entry point ─────────────────────────────────────────────────────────────
def main():
    parser = argparse.ArgumentParser(
        description='Integration test — launches launch_sim.py + nav_stack.py automatically.'
    )
    parser.add_argument('--target', type=str,
                        help='Object to search for')
    parser.add_argument('--test-all', action='store_true',
                        help='Test every object in ALLOWED_OBJECTS')
    parser.add_argument('--timeout', type=int, default=DEFAULT_TIMEOUT,
                        help=f'Mission timeout in seconds (default: {DEFAULT_TIMEOUT})')
    parser.add_argument('--disable-completion-stop', action='store_true', default=False,
                        help='Keep exploring after map complete (default: True)')
    parser.add_argument('--results-file', type=Path, default=None,
                        help='Path to save results (e.g. results.json)')
    parser.add_argument('--world', type=str, default='RG1',
                        help='Gazebo world name for reset (default: RG1)')
    parser.add_argument('--list-objects', action='store_true',
                        help='Print allowed objects and exit')
    args = parser.parse_args()
 
    if args.list_objects:
        print("Allowed objects:")
        for obj in ALLOWED_OBJECTS:
            print(f"  - {obj}")
        sys.exit(0)
 
    if args.test_all:
        success = run_all_tests(
            args.timeout, args.disable_completion_stop,
            args.results_file, args.world
        )
        sys.exit(0 if success else 1)
 
    if not args.target:
        parser.error("Provide --target <object> or --test-all")
 
    test = IntegrationTest(
        args.target, args.timeout,
        args.disable_completion_stop, args.results_file, args.world
    )
    success = test.run()
    sys.exit(0 if success else 1)
 
 
if __name__ == '__main__':
    main()
 
#!/usr/bin/env python3
"""
test_integration.py

Full system integration test for the object-finding robot.
Launches launch_sim.py and nav_stack.py automatically.

Usage:
    # Gazebo must be running on host first, then:
    python3 test_integration.py --target "person"
    python3 test_integration.py --target "chair" --results-file results.json
    python3 test_integration.py --test-all --results-file results.json
    python3 test_integration.py --list-objects
"""

import argparse
import json
import subprocess
import sys
import time
import threading
from datetime import datetime
from pathlib import Path

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from nav_msgs.msg import Odometry
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Twist


# ─── Allowed objects ────────────────────────────────────────────────────────
ALLOWED_OBJECTS = [
    "backpack",
    "bicycle",
    "refrigerator",
    "teddy bear",
    # Add more as needed
]

# ─── Paths ──────────────────────────────────────────────────────────────────
NAV_STACK    = Path("/DEV_WS/ros2_ws/src/nav/nav_stack.py")
LAUNCH_PKG   = "my_robot_description"
LAUNCH_FILE  = "launch_sim.py"

# ─── Timing ─────────────────────────────────────────────────────────────────
DEFAULT_TIMEOUT        = 300   # seconds for mission
LAUNCH_SIM_WAIT_SEC    = 8     # wait after launch_sim.py starts
NAV_STACK_WAIT_SEC     = 6     # wait after nav_stack.py starts
ROBOT_STOP_TOLERANCE   = 0.02  # m/s below this = stopped
ROBOT_STOP_DURATION    = 3.0   # seconds must stay stopped to confirm


# ─── Gazebo readiness check ──────────────────────────────────────────────────
def wait_for_gazebo(timeout_sec=30) -> bool:
    """Return True if Gazebo /clock is publishing within timeout."""
    print("  Checking Gazebo is running...")
    deadline = time.time() + timeout_sec
    while time.time() < deadline:
        result = subprocess.run(
            ['ros2', 'topic', 'hz', '/clock', '--window', '5'],
            capture_output=True, text=True, timeout=6
        )
        if 'average rate' in result.stdout:
            print("  ✓ Gazebo is running")
            return True
        time.sleep(2)
    print("  ✗ Gazebo not detected — is it running on the host?")
    return False


# ─── Mission monitor node ────────────────────────────────────────────────────
class MissionMonitor(Node):
    def __init__(self):
        super().__init__('integration_test_monitor')

        self.found = False
        self.found_time = None
        self.map_received = False
        self.map_width = 0
        self.map_height = 0
        self.robot_stopped = False
        self.robot_stop_start = None
        self.odom_positions = []

        self.create_subscription(Bool, '/detections/found', self._on_found, 10)
        self.create_subscription(OccupancyGrid, '/map', self._on_map, 10)
        self.create_subscription(Odometry, '/odom', self._on_odom, 10)
        self.create_subscription(Twist, '/cmd_vel', self._on_cmd_vel, 10)

    def _on_found(self, msg):
        if msg.data and not self.found:
            self.found = True
            self.found_time = time.time()
            self.get_logger().info('✓ /detections/found = True')

    def _on_map(self, msg):
        self.map_received = True
        self.map_width = msg.info.width
        self.map_height = msg.info.height

    def _on_odom(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        self.odom_positions.append((x, y))

    def _on_cmd_vel(self, msg):
        moving = (abs(msg.linear.x) > ROBOT_STOP_TOLERANCE or
                  abs(msg.angular.z) > ROBOT_STOP_TOLERANCE)
        if not moving:
            if self.robot_stop_start is None:
                self.robot_stop_start = time.time()
            elif time.time() - self.robot_stop_start >= ROBOT_STOP_DURATION:
                self.robot_stopped = True
        else:
            self.robot_stop_start = None
            self.robot_stopped = False

    @property
    def robot_moved(self):
        if len(self.odom_positions) < 2:
            return False
        xs = [p[0] for p in self.odom_positions]
        ys = [p[1] for p in self.odom_positions]
        dist = ((max(xs)-min(xs))**2 + (max(ys)-min(ys))**2) ** 0.5
        return dist > 0.1


# ─── Integration test ────────────────────────────────────────────────────────
class IntegrationTest:
    def __init__(self, target: str, timeout: int,
                 disable_completion_stop: bool = True,
                 results_file: Path = None,
                 world: str = 'RG1'):
        self.target = target
        self.timeout = timeout
        self.disable_completion_stop = disable_completion_stop
        self.results_file = results_file
        self.world = world
        self.launch_sim_proc = None
        self.nav_proc = None
        self.results = {}

    # ── Process management ───────────────────────────────────────────────────

    def _launch_sim(self):
        """Start launch_sim.py with the target object as user_input."""
        print(f"  Starting launch_sim.py with user_input={self.target}...")
        self.launch_sim_proc = subprocess.Popen([
            'ros2', 'launch', LAUNCH_PKG, LAUNCH_FILE,
            f'user_input:={self.target}',
        ])
        time.sleep(LAUNCH_SIM_WAIT_SEC)
        print("  ✓ launch_sim.py started")

    def _launch_nav_stack(self):
        """Start nav_stack.py."""
        print("  Starting nav_stack.py...")
        cmd = ['python3', str(NAV_STACK), '--no-stop-on-exit']
        if self.disable_completion_stop:
            cmd.append('--disable-completion-stop')
        self.nav_proc = subprocess.Popen(cmd)
        time.sleep(NAV_STACK_WAIT_SEC)
        print("  ✓ nav_stack.py started")

    def _stop_all(self):
        """Terminate both processes cleanly."""
        for name, proc in [('nav_stack', self.nav_proc),
                            ('launch_sim', self.launch_sim_proc)]:
            if proc and proc.poll() is None:
                print(f"  Stopping {name}...")
                proc.terminate()
                try:
                    proc.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    proc.kill()

    def _stop_robot(self):
        subprocess.run([
            'ros2', 'topic', 'pub', '--once', '/cmd_vel',
            'geometry_msgs/msg/Twist',
            '{linear: {x: 0.0}, angular: {z: 0.0}}'
        ], capture_output=True)

    # ── Run ──────────────────────────────────────────────────────────────────

    def run(self) -> bool:
        print(f"\n{'='*60}")
        print(f"  TARGET:              '{self.target}'")
        print(f"  TIMEOUT:             {self.timeout}s")
        print(f"  COMPLETION_STOP_OFF: {self.disable_completion_stop}")
        print(f"{'='*60}")

        if self.target not in ALLOWED_OBJECTS:
            print(f"\n  ERROR: '{self.target}' is not in ALLOWED_OBJECTS.")
            print(f"  Allowed: {ALLOWED_OBJECTS}")
            return False

        # Check Gazebo
        if not wait_for_gazebo():
            return False

        # Launch everything
        self._launch_sim()
        self._launch_nav_stack()

        # ROS monitor
        rclpy.init()
        monitor = MissionMonitor()
        spin_thread = threading.Thread(
            target=lambda: rclpy.spin(monitor), daemon=True
        )
        spin_thread.start()

        print(f"\n  Mission running — monitoring for up to {self.timeout}s...\n")
        start = time.time()
        last_print = 0

        try:
            while time.time() - start < self.timeout:
                elapsed = time.time() - start

                if elapsed - last_print >= 10:
                    last_print = elapsed
                    moved   = "yes" if monitor.robot_moved else "no"
                    map_info = f"{monitor.map_width}x{monitor.map_height}" \
                               if monitor.map_received else "none"
                    found   = "YES ✓" if monitor.found else "no"
                    print(f"  [{elapsed:5.0f}s] map={map_info}  "
                          f"moved={moved}  found={found}")

                if monitor.found and monitor.robot_stopped:
                    print(f"\n  ✓ Mission SUCCESS at {elapsed:.1f}s")
                    break

                time.sleep(0.5)

        finally:
            self._stop_robot()
            self._stop_all()

        elapsed = time.time() - start
        self.results = {
            'timestamp':      datetime.now().isoformat(),
            'target':         self.target,
            'timeout':        self.timeout,
            'duration_sec':   round(elapsed, 1),
            'found_signal':   monitor.found,
            'found_time_sec': round(monitor.found_time - start, 1)
                              if monitor.found_time else None,
            'robot_stopped':  monitor.robot_stopped,
            'robot_moved':    monitor.robot_moved,
            'map_received':   monitor.map_received,
            'map_size':       f"{monitor.map_width}x{monitor.map_height}",
            'timed_out':      elapsed >= self.timeout,
        }

        monitor.destroy_node()
        rclpy.shutdown()

        passed = self._evaluate()
        self.results['passed'] = passed

        if self.results_file:
            self._write_results()

        return passed

    # ── Evaluate ─────────────────────────────────────────────────────────────

    def _evaluate(self) -> bool:
        r = self.results
        passed = True
        issues = []

        print(f"\n{'─'*60}")
        print("  RESULTS")
        print(f"{'─'*60}")

        if not r['map_received']:
            issues.append("SLAM never published a map — check TF chain")
            passed = False
        else:
            print(f"  ✓ Map published ({r['map_size']})")

        if not r['robot_moved']:
            issues.append("Robot never moved — check nav_explorer is receiving scan/map")
            passed = False
        else:
            print(f"  ✓ Robot moved during exploration")

        if not r['found_signal']:
            issues.append(f"'{r['target']}' not detected within {r['duration_sec']}s")
            passed = False
        else:
            print(f"  ✓ Object found at {r['found_time_sec']}s")

        if r['found_signal'] and not r['robot_stopped']:
            issues.append("Found signal received but robot did not stop")
            passed = False
        elif r['found_signal'] and r['robot_stopped']:
            print(f"  ✓ Robot stopped after finding object")

        if r['timed_out']:
            issues.append(f"Mission timed out after {r['timeout']}s")
            passed = False

        print(f"{'─'*60}")
        if passed:
            print(f"  OVERALL: PASS ✓  ({r['duration_sec']}s)")
        else:
            print(f"  OVERALL: FAIL ✗")
            for issue in issues:
                print(f"    • {issue}")
        print(f"{'─'*60}\n")

        return passed

    # ── Save results ─────────────────────────────────────────────────────────

    def _write_results(self):
        path = Path(self.results_file)

        # JSON — all runs in one array
        all_results = []
        if path.exists():
            try:
                with open(path) as f:
                    all_results = json.load(f)
            except (json.JSONDecodeError, IOError):
                all_results = []
        all_results.append(self.results)
        with open(path, 'w') as f:
            json.dump(all_results, f, indent=2)
        print(f"  Results saved → {path.resolve()}")

        # Text summary — one line per run
        txt_path = path.with_suffix('.txt')
        r = self.results
        status = "PASS" if r['passed'] else "FAIL"
        with open(txt_path, 'a') as f:
            f.write(
                f"[{r['timestamp']}] {status} | "
                f"target={r['target']} | "
                f"found={r['found_signal']} | "
                f"found_at={r['found_time_sec']}s | "
                f"duration={r['duration_sec']}s | "
                f"map={r['map_size']} | "
                f"timed_out={r['timed_out']}\n"
            )
        print(f"  Summary appended → {txt_path.resolve()}")


# ─── Run all objects ─────────────────────────────────────────────────────────
def run_all_tests(timeout: int, disable_completion_stop: bool,
                  results_file: Path, world: str):
    print(f"\nRunning tests for all {len(ALLOWED_OBJECTS)} allowed objects...\n")
    passed_list = []
    failed_list = []

    for obj in ALLOWED_OBJECTS:
        test = IntegrationTest(obj, timeout, disable_completion_stop,
                               results_file, world)
        success = test.run()
        if success:
            passed_list.append(obj)
        else:
            failed_list.append(obj)
        print("  Waiting 5s before next test...\n")
        time.sleep(5)

    print(f"\n{'='*60}")
    print(f"  SUMMARY: {len(passed_list)}/{len(ALLOWED_OBJECTS)} passed")
    if passed_list:
        print(f"  Passed: {passed_list}")
    if failed_list:
        print(f"  Failed: {failed_list}")
    print(f"{'='*60}\n")

    return len(failed_list) == 0


# ─── Entry point ─────────────────────────────────────────────────────────────
def main():
    parser = argparse.ArgumentParser(
        description='Integration test — launches launch_sim.py + nav_stack.py automatically.'
    )
    parser.add_argument('--target', type=str,
                        help='Object to search for')
    parser.add_argument('--test-all', action='store_true',
                        help='Test every object in ALLOWED_OBJECTS')
    parser.add_argument('--timeout', type=int, default=DEFAULT_TIMEOUT,
                        help=f'Mission timeout in seconds (default: {DEFAULT_TIMEOUT})')
    parser.add_argument('--disable-completion-stop', action='store_true', default=True,
                        help='Keep exploring after map complete (default: True)')
    parser.add_argument('--results-file', type=Path, default=None,
                        help='Path to save results (e.g. results.json)')
    parser.add_argument('--world', type=str, default='RG1',
                        help='Gazebo world name for reset (default: RG1)')
    parser.add_argument('--list-objects', action='store_true',
                        help='Print allowed objects and exit')
    args = parser.parse_args()

    if args.list_objects:
        print("Allowed objects:")
        for obj in ALLOWED_OBJECTS:
            print(f"  - {obj}")
        sys.exit(0)

    if args.test_all:
        success = run_all_tests(
            args.timeout, args.disable_completion_stop,
            args.results_file, args.world
        )
        sys.exit(0 if success else 1)

    if not args.target:
        parser.error("Provide --target <object> or --test-all")

    test = IntegrationTest(
        args.target, args.timeout,
        args.disable_completion_stop, args.results_file, args.world
    )
    success = test.run()
    sys.exit(0 if success else 1)


if __name__ == '__main__':
    main()