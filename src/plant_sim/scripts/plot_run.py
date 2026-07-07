#!/usr/bin/env python3
"""Record the integration test and save the evidence figure.

Spawns the integration test (which launches the simulation and drives the
acquire -> lock -> blockage -> coast -> re-lock story), records the true
pointing error and the mode transitions while it runs, and renders a PNG.
The figure therefore shows exactly the run that the test suite asserts.
"""
import argparse
import os
import subprocess
import time

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from pat_interfaces.msg import AxisState, ModeState

MODE_NAMES = {0: 'IDLE', 1: 'ACQUIRE', 2: 'HANDOFF', 3: 'LOCK', 4: 'COAST', 5: 'SAFE'}
BAND_TINTS = {
    'IDLE': '#e8e8e8', 'ACQUIRE': '#f6e3c5', 'HANDOFF': '#dce9f5',
    'LOCK': '#d9eedd', 'COAST': '#f5dede', 'SAFE': '#eadff2',
}
SERIES_COLORS = {'azimuth': '#0072B2', 'elevation': '#E69F00'}  # Okabe-Ito pair


class Recorder:

    def __init__(self, node):
        self.node = node
        self.start = time.monotonic()
        self.times, self.azimuth, self.elevation = [], [], []
        self.mode_changes = []  # (t, mode name)
        latched = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        node.create_subscription(AxisState, 'true_error', self.on_error, 100)
        node.create_subscription(ModeState, 'mode', self.on_mode, latched)

    def elapsed(self):
        return time.monotonic() - self.start

    def on_error(self, msg):
        self.times.append(self.elapsed())
        self.azimuth.append(msg.azimuth)
        self.elevation.append(msg.elevation)

    def on_mode(self, msg):
        name = MODE_NAMES.get(msg.mode, '?')
        if not self.mode_changes or self.mode_changes[-1][1] != name:
            self.mode_changes.append((self.elapsed(), name))


def record_integration_test(node, recorder):
    test_path = os.path.join(
        os.path.dirname(os.path.abspath(__file__)), '..', 'test', 'test_integration.py')
    test = subprocess.Popen(['launch_test', test_path])
    while test.poll() is None:
        rclpy.spin_once(node, timeout_sec=0.02)
    if test.returncode != 0:
        raise RuntimeError(f'integration test failed with code {test.returncode}')


def draw_bands(axis, changes, end, label):
    for i, (start, name) in enumerate(changes):
        stop = changes[i + 1][0] if i + 1 < len(changes) else end
        axis.axvspan(start, stop, color=BAND_TINTS.get(name, '#eeeeee'), zorder=0)
        if label and stop - start > 0.5:
            axis.text((start + stop) / 2, 0.97, name, transform=axis.get_xaxis_transform(),
                      ha='center', va='top', fontsize=7, color='#555555')


def plot(recorder, output):
    fig, (full, zoom) = plt.subplots(2, 1, sharex=True, figsize=(10, 6))
    to_mrad = [e * 1e3 for e in recorder.azimuth], [e * 1e3 for e in recorder.elevation]
    to_urad = [e * 1e6 for e in recorder.azimuth], [e * 1e6 for e in recorder.elevation]
    end = recorder.times[-1]

    for axis, (az, el), unit in ((full, to_mrad, 'mrad'), (zoom, to_urad, 'urad')):
        draw_bands(axis, recorder.mode_changes, end, label=(axis is full))
        axis.plot(recorder.times, az, color=SERIES_COLORS['azimuth'],
                  linewidth=1.2, label='azimuth')
        axis.plot(recorder.times, el, color=SERIES_COLORS['elevation'],
                  linewidth=1.2, label='elevation')
        axis.set_ylabel(f'true error [{unit}]')
        axis.grid(True, alpha=0.25, linewidth=0.5)

    zoom.set_ylim(-150, 150)
    zoom.set_xlabel('time [s]')
    full.legend(loc='upper right', fontsize=8)
    full.set_title('PAT demo: acquisition, lock, blockage, coast, re-lock')
    zoom.set_title('same run at lock scale', fontsize=9)

    os.makedirs(os.path.dirname(output) or '.', exist_ok=True)
    fig.tight_layout()
    fig.savefig(output, dpi=150)
    print(f'saved {output}')


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', default='docs/pat_integration_test.png')
    args = parser.parse_args()

    rclpy.init()
    node = rclpy.create_node('plot_recorder')
    recorder = Recorder(node)
    try:
        record_integration_test(node, recorder)
    finally:
        node.destroy_node()
        rclpy.shutdown()

    plot(recorder, args.output)


if __name__ == '__main__':
    main()
