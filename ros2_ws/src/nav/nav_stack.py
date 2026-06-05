#!/usr/bin/env python3

"""
nav_stack.py

One-command bringup for the navigation side of the vehicle_blue stack.
This does NOT start Gazebo. Start/play the simulation separately, then run this.

Default current setup:
python3 nav_stack.py

Common custom topic example:
python3 nav_stack.py \
  --scan-topic /lidar2 \
  --map-topic /map \
  --cmd-vel-topic /model/vehicle_blue/cmd_vel \
  --odom-topic /model/vehicle_blue/odometry

Run only the navigator against already-existing ROS topics:
python3 nav_stack.py --no-bridge --no-slam --no-odom-tf --no-static-tf
"""

import argparse
import os
import signal
import subprocess
import sys
import time
from pathlib import Path


PROCS = []
SHUTTING_DOWN = False


def script_dir():
    return Path(__file__).resolve().parent


def start(name, cmd, delay=0.0):
    print(f'\n=== starting {name} ===')
    print(' '.join(cmd))
    proc = subprocess.Popen(cmd)
    PROCS.append((name, proc))
    if delay > 0.0:
        time.sleep(delay)
    return proc


def run_once(name, cmd):
    print(f'\n=== running {name} ===')
    print(' '.join(cmd))
    try:
        subprocess.run(cmd, check=False)
    except FileNotFoundError:
        print(f'WARNING: could not run {name}; command not found: {cmd[0]}')


def shutdown(signum=None, frame=None):
    
    global SHUTTING_DOWN
    if SHUTTING_DOWN:
        return
    SHUTTING_DOWN = True
    print('\nShutting down nav stack...')

    # Try to stop the robot first. This may fail if ROS is already down; okay.
    if ARGS is not None and not ARGS.no_stop_on_exit:
        run_once('stop robot', [
            'ros2', 'topic', 'pub', '--once', ARGS.cmd_vel_topic,
            'geometry_msgs/msg/Twist',
            '{linear: {x: 0.0}, angular: {z: 0.0}}'
        ])

    for name, proc in reversed(PROCS):
        if proc.poll() is None:
            print(f'terminating {name}')
            try:
                proc.terminate()
            except ProcessLookupError:
                pass

    deadline = time.time() + 5.0
    for name, proc in reversed(PROCS):
        remaining = max(0.0, deadline - time.time())
        if proc.poll() is None:
            try:
                proc.wait(timeout=remaining)
            except subprocess.TimeoutExpired:
                print(f'killing {name}')
                try:
                    proc.kill()
                except ProcessLookupError:
                    pass

    sys.exit(0)
    
    


def parse_args():
    p = argparse.ArgumentParser(description='One-command navigation stack bringup. Gazebo is separate.')

    p.add_argument('--scan-topic', default='/lidar2')
    p.add_argument('--map-topic', default='/map')
    p.add_argument('--cmd-vel-topic', default='/cmd_vel')
    p.add_argument('--odom-topic', default='/odom')
    p.add_argument('--clock-topic', default='/clock')

    p.add_argument('--map-frame', default='map')
    p.add_argument('--odom-frame', default='odom')
    p.add_argument('--base-frame', default='base_link')
    p.add_argument('--lidar-frame', default='lidar_link')

    p.add_argument('--slam-params-file', default=str(Path.home() / 'mapper_params_vehicle_blue.yaml'))

    p.add_argument('--no-bridge', action='store_true', help='Do not start ros_gz_bridge bridges.')
    p.add_argument('--no-slam', action='store_true', help='Do not start slam_toolbox.')
    p.add_argument('--no-odom-tf', action='store_true', help='Do not start odom_to_tf.py.')
    p.add_argument('--no-static-tf', action='store_true', help='Do not publish static base->lidar TF.')
    p.add_argument('--no-stop-on-start', action='store_true')
    p.add_argument('--no-stop-on-exit', action='store_true')

    p.add_argument('--use-sim-time', default='true', choices=['true', 'false'])
    p.add_argument('--disable-completion-stop', action='store_true')
    p.add_argument('--found-topic', default='/detections/found')
    p.add_argument('--no-stop-on-found', action='store_true')

    # Bridge message type knobs. Defaults match the current Gazebo/ROS setup.
    p.add_argument('--scan-gz-type', default='gz.msgs.LaserScan')
    p.add_argument('--odom-gz-type', default='gz.msgs.Odometry')
    p.add_argument('--twist-gz-type', default='gz.msgs.Twist')

    return p.parse_args()


def main():
    global ARGS
    ARGS = parse_args()

    signal.signal(signal.SIGINT, shutdown)
    signal.signal(signal.SIGTERM, shutdown)

    sd = script_dir()
    nav_explorer = sd / 'nav_explorer.py'
    odom_to_tf = sd / 'odom_to_tf.py'

    if not nav_explorer.exists():
        print(f'ERROR: missing {nav_explorer}')
        sys.exit(1)

    # 1) Bridges. Assumes Gazebo is already running and playing.
    if not ARGS.no_bridge:
        start('clock/lidar/odom bridge', [
            'ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
            f'{ARGS.clock_topic}@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            f'{ARGS.scan_topic}@sensor_msgs/msg/LaserScan[{ARGS.scan_gz_type}',
            f'{ARGS.odom_topic}@nav_msgs/msg/Odometry[{ARGS.odom_gz_type}',
        ], delay=1.0)

        start('cmd_vel bridge', [
            'ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
            f'{ARGS.cmd_vel_topic}@geometry_msgs/msg/Twist]{ARGS.twist_gz_type}',
        ], delay=1.0)

    # 2) Dynamic odom TF.
    if not ARGS.no_odom_tf:
        if not odom_to_tf.exists():
            print(f'ERROR: missing {odom_to_tf}')
            shutdown()
        start('odom_to_tf', [
            'python3', str(odom_to_tf),
            '--ros-args',
            '-p', f'use_sim_time:={ARGS.use_sim_time}',
            '-p', f'odom_topic:={ARGS.odom_topic}',
            '-p', f'odom_frame:={ARGS.odom_frame}',
            '-p', f'base_frame:={ARGS.base_frame}',
        ], delay=1.0)

    # 3) Static base->lidar TF.
    if not ARGS.no_static_tf:
        start('static base->lidar TF', [
            'ros2', 'run', 'tf2_ros', 'static_transform_publisher',
            '0', '0', '0', '0', '0', '0',
            ARGS.base_frame,
            ARGS.lidar_frame,
        ], delay=1.0)

    # 4) SLAM Toolbox.
    if not ARGS.no_slam:
        start('slam_toolbox', [
            'ros2', 'launch', 'slam_toolbox', 'online_async_launch.py',
            f'use_sim_time:={ARGS.use_sim_time}',
            f'slam_params_file:={ARGS.slam_params_file}',
        ], delay=3.0)

    # 5) Stop robot once before navigation starts.
    if not ARGS.no_stop_on_start:
        run_once('stop robot before nav', [
            'ros2', 'topic', 'pub', '--once', ARGS.cmd_vel_topic,
            'geometry_msgs/msg/Twist',
            '{linear: {x: 0.0}, angular: {z: 0.0}}'
        ])

    # 6) Navigation node.
    nav_cmd = [
        'python3', str(nav_explorer),
        '--ros-args',
        '-p', f'use_sim_time:={ARGS.use_sim_time}',
        '-p', f'scan_topic:={ARGS.scan_topic}',
        '-p', f'map_topic:={ARGS.map_topic}',
        '-p', f'cmd_vel_topic:={ARGS.cmd_vel_topic}',
        '-p', f'map_frame:={ARGS.map_frame}',
        '-p', f'base_frame:={ARGS.base_frame}',
        '-p', f'found_topic:={ARGS.found_topic}',
        '-p', f'stop_on_found:={str(not ARGS.no_stop_on_found).lower()}',
    ]

    if ARGS.disable_completion_stop:
        nav_cmd += ['-p', 'disable_completion_stop:=true']

    start('nav_explorer', nav_cmd)

    print('\nNavigation stack is running. Press Ctrl-C to stop everything started by this script.\n')
    reported_exits = set()
    while True:
        time.sleep(1.0)
        for name, proc in PROCS:
            returncode = proc.poll()
            if returncode is None:
                continue

            if name == 'nav_explorer':
                print(f'nav_explorer exited with returncode={returncode}. Shutting down nav stack.')
                continue

            if name not in reported_exits:
                print(f'WARNING: process exited: {name} returncode={returncode}')
                reported_exits.add(name)


ARGS = None


if __name__ == '__main__':
    main()
