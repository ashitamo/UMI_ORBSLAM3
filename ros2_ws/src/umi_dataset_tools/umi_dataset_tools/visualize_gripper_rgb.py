#!/usr/bin/env python3
"""Interactively visualize gripper opening and synchronized color images."""

import argparse
from bisect import bisect_left
import csv
from functools import lru_cache
from pathlib import Path

import cv2
import matplotlib.pyplot as plt
from matplotlib.widgets import Button, Slider
import numpy as np


def nearest_index(sorted_values, target):
    insertion = bisect_left(sorted_values, target)
    candidates = []
    if insertion < len(sorted_values):
        candidates.append(insertion)
    if insertion > 0:
        candidates.append(insertion - 1)
    return min(candidates, key=lambda index: abs(sorted_values[index] - target))


def read_gripper_csv(path, value_column):
    records = []
    with path.open(newline='') as stream:
        reader = csv.DictReader(stream)
        required = {'timestamp_ns', value_column}
        missing = required.difference(reader.fieldnames or [])
        if missing:
            raise RuntimeError(f'{path} is missing columns: {sorted(missing)}')
        for row in reader:
            if not row[value_column]:
                continue
            timestamp_ns = int(row['timestamp_ns'])
            records.append((timestamp_ns, float(row[value_column])))
    if not records:
        raise RuntimeError(f'No valid {value_column} records found in {path}')
    records.sort(key=lambda record: record[0])
    return records


def associate_images(records, color_dir, max_image_dt_sec):
    image_paths = sorted(
        (path for path in color_dir.glob('*.png') if path.stem.isdigit()),
        key=lambda path: int(path.stem),
    )
    if not image_paths:
        raise RuntimeError(f'No timestamp-named PNG images found in {color_dir}')
    image_timestamps = [int(path.stem) for path in image_paths]
    max_dt_ns = int(max_image_dt_sec * 1e9)
    associations = []
    for timestamp_ns, _ in records:
        image_index = nearest_index(image_timestamps, timestamp_ns)
        image_timestamp = image_timestamps[image_index]
        if abs(image_timestamp - timestamp_ns) <= max_dt_ns:
            associations.append((image_paths[image_index], image_timestamp))
        else:
            associations.append((None, None))
    return associations


def lowpass_filter(values, times_sec, cutoff_hz):
    if len(values) < 2:
        return values.copy()
    time_constant = 1.0 / (2.0 * np.pi * cutoff_hz)

    def filter_once(samples, sample_times):
        filtered = np.empty_like(samples, dtype=float)
        filtered[0] = samples[0]
        for index in range(1, len(samples)):
            dt = abs(float(sample_times[index] - sample_times[index - 1]))
            alpha = dt / (time_constant + dt) if dt > 0 else 0.0
            filtered[index] = filtered[index - 1] + alpha * (samples[index] - filtered[index - 1])
        return filtered

    forward = filter_once(values, times_sec)
    return filter_once(forward[::-1], times_sec[::-1])[::-1]


@lru_cache(maxsize=32)
def load_rgb_image(path):
    image = cv2.imread(path, cv2.IMREAD_COLOR)
    if image is None:
        raise RuntimeError(f'Unable to read image: {path}')
    return cv2.cvtColor(image, cv2.COLOR_BGR2RGB)


class GripperRgbViewer:
    def __init__(self, dataset_dir, value_column, fps, playback_rate, max_image_dt_sec, lowpass_cutoff_hz):
        self.dataset_dir = dataset_dir
        self.value_column = value_column
        self.fps = fps
        self.playback_rate = playback_rate
        self.lowpass_cutoff_hz = lowpass_cutoff_hz
        self.playing = False
        self.index = 0

        gripper_path = dataset_dir / 'gripper.csv'
        color_dir = dataset_dir / 'color'
        if not gripper_path.is_file():
            raise RuntimeError(f'Gripper CSV not found: {gripper_path}')
        if not color_dir.is_dir():
            raise RuntimeError(f'Color image directory not found: {color_dir}')

        records = read_gripper_csv(gripper_path, value_column)
        self.timestamps_ns = [record[0] for record in records]
        self.values = np.asarray([record[1] for record in records], dtype=float)
        self.times_sec = (np.asarray(self.timestamps_ns, dtype=np.int64) - self.timestamps_ns[0]) / 1e9
        self.filtered_values = lowpass_filter(self.values, self.times_sec, lowpass_cutoff_hz)
        self.image_associations = associate_images(records, color_dir, max_image_dt_sec)

        self.figure = plt.figure(figsize=(14, 9))
        grid = self.figure.add_gridspec(2, 1, height_ratios=(3, 2))
        self.image_axis = self.figure.add_subplot(grid[0])
        self.wave_axis = self.figure.add_subplot(grid[1])
        self.figure.subplots_adjust(left=0.08, right=0.97, top=0.95, bottom=0.16, hspace=0.25)

        first_image = self._image_for_index(0)
        self.image_artist = self.image_axis.imshow(first_image)
        self.image_axis.axis('off')

        self.wave_axis.plot(
            self.times_sec, self.values, color='tab:blue', linewidth=1.0, alpha=0.55, label='Raw'
        )
        self.wave_axis.plot(
            self.times_sec,
            self.filtered_values,
            color='tab:orange',
            linewidth=2.0,
            label=f'Low-pass ({lowpass_cutoff_hz:g} Hz)',
        )
        self.cursor = self.wave_axis.axvline(self.times_sec[0], color='tab:red', linewidth=1.5)
        self.raw_marker, = self.wave_axis.plot(
            [self.times_sec[0]], [self.values[0]], marker='o', color='tab:blue', markersize=6
        )
        self.filtered_marker, = self.wave_axis.plot(
            [self.times_sec[0]], [self.filtered_values[0]], marker='o', color='tab:orange', markersize=7
        )
        self.wave_axis.set_xlabel('Time from first gripper measurement [s]')
        self.wave_axis.set_ylabel(self._value_label())
        self.wave_axis.grid(True, alpha=0.3)
        self.wave_axis.margins(x=0.01, y=0.1)
        self.wave_axis.legend(loc='best')

        slider_axis = self.figure.add_axes((0.18, 0.075, 0.62, 0.025))
        self.slider = Slider(
            slider_axis,
            'Frame',
            0,
            len(self.timestamps_ns) - 1,
            valinit=0,
            valstep=1,
        )
        previous_axis = self.figure.add_axes((0.18, 0.02, 0.09, 0.04))
        play_axis = self.figure.add_axes((0.285, 0.02, 0.12, 0.04))
        next_axis = self.figure.add_axes((0.42, 0.02, 0.09, 0.04))
        slower_axis = self.figure.add_axes((0.60, 0.02, 0.09, 0.04))
        faster_axis = self.figure.add_axes((0.705, 0.02, 0.09, 0.04))
        self.previous_button = Button(previous_axis, 'Previous')
        self.play_button = Button(play_axis, 'Play')
        self.next_button = Button(next_axis, 'Next')
        self.slower_button = Button(slower_axis, 'Slower')
        self.faster_button = Button(faster_axis, 'Faster')

        self.slider.on_changed(self._on_slider)
        self.previous_button.on_clicked(lambda _event: self._step(-1))
        self.play_button.on_clicked(lambda _event: self._toggle_play())
        self.next_button.on_clicked(lambda _event: self._step(1))
        self.slower_button.on_clicked(lambda _event: self._change_rate(0.5))
        self.faster_button.on_clicked(lambda _event: self._change_rate(2.0))
        self.figure.canvas.mpl_connect('key_press_event', self._on_key)
        self.figure.canvas.mpl_connect('button_press_event', self._on_wave_click)
        self.figure.canvas.mpl_connect('close_event', lambda _event: self.timer.stop())

        self.timer = self.figure.canvas.new_timer(interval=max(1, int(round(1000 / fps))))
        self.timer.add_callback(self._on_timer)
        self.timer.start()
        self._render(0)

    def _value_label(self):
        if self.value_column == 'opening_m':
            return 'Gripper opening [m]'
        return 'Gripper opening [%]'

    def _image_for_index(self, index):
        image_path, _ = self.image_associations[index]
        if image_path is None:
            return np.zeros((480, 640, 3), dtype=np.uint8)
        return load_rgb_image(str(image_path))

    def _render(self, index):
        self.index = int(np.clip(index, 0, len(self.timestamps_ns) - 1))
        timestamp_ns = self.timestamps_ns[self.index]
        relative_time = self.times_sec[self.index]
        value = self.values[self.index]
        filtered_value = self.filtered_values[self.index]
        image_path, image_timestamp = self.image_associations[self.index]

        self.image_artist.set_data(self._image_for_index(self.index))
        if image_path is None:
            image_status = 'no image within tolerance'
        else:
            image_dt_ms = (image_timestamp - timestamp_ns) / 1e6
            image_status = f'image={image_path.name}, image-gripper={image_dt_ms:+.2f} ms'
        self.image_axis.set_title(
            f'raw={value:.6f}, low-pass={filtered_value:.6f} | gripper={timestamp_ns} | '
            f'{image_status} | rate={self.playback_rate:g}x'
        )
        self.cursor.set_xdata([relative_time, relative_time])
        self.raw_marker.set_data([relative_time], [value])
        self.filtered_marker.set_data([relative_time], [filtered_value])
        self.figure.canvas.draw_idle()

    def _set_index(self, index):
        index = int(np.clip(index, 0, len(self.timestamps_ns) - 1))
        if int(self.slider.val) == index:
            self._render(index)
        else:
            self.slider.set_val(index)

    def _on_slider(self, value):
        self._render(int(value))

    def _step(self, amount):
        self.playing = False
        self.play_button.label.set_text('Play')
        self._set_index(self.index + amount)

    def _toggle_play(self):
        if self.index >= len(self.timestamps_ns) - 1:
            self._set_index(0)
        self.playing = not self.playing
        self.play_button.label.set_text('Pause' if self.playing else 'Play')

    def _change_rate(self, multiplier):
        self.playback_rate = float(np.clip(self.playback_rate * multiplier, 0.125, 16.0))
        self._render(self.index)

    def _on_timer(self):
        if not self.playing:
            return
        if self.index >= len(self.timestamps_ns) - 1:
            self.playing = False
            self.play_button.label.set_text('Play')
            return
        target_timestamp = self.timestamps_ns[self.index] + int(self.playback_rate * 1e9 / self.fps)
        next_index = bisect_left(self.timestamps_ns, target_timestamp, lo=self.index + 1)
        self._set_index(min(next_index, len(self.timestamps_ns) - 1))

    def _on_key(self, event):
        if event.key == ' ':
            self._toggle_play()
        elif event.key in ('right', 'd'):
            self._step(1)
        elif event.key in ('left', 'a'):
            self._step(-1)
        elif event.key == 'home':
            self._set_index(0)
        elif event.key == 'end':
            self._set_index(len(self.timestamps_ns) - 1)
        elif event.key in ('up', '+'):
            self._change_rate(2.0)
        elif event.key in ('down', '-'):
            self._change_rate(0.5)
        elif event.key in ('escape', 'q'):
            plt.close(self.figure)

    def _on_wave_click(self, event):
        if event.inaxes is not self.wave_axis or event.xdata is None:
            return
        target_timestamp = self.timestamps_ns[0] + int(event.xdata * 1e9)
        self._set_index(nearest_index(self.timestamps_ns, target_timestamp))

    def show(self):
        plt.show()


def main():
    parser = argparse.ArgumentParser(description='Visualize gripper opening waveform with synchronized RGB images.')
    parser.add_argument('dataset', type=Path, help='Processed dataset directory containing gripper.csv and color/')
    parser.add_argument('--value', choices=['opening_m', 'opening_percent'], default='opening_m')
    parser.add_argument('--fps', type=float, default=30.0, help='UI refresh rate, default: 30')
    parser.add_argument('--playback-rate', type=float, default=1.0, help='Initial real-time playback rate, default: 1')
    parser.add_argument('--max-image-dt', type=float, default=0.05, help='Maximum image/gripper time gap in seconds')
    parser.add_argument('--lowpass-cutoff', type=float, default=2.0, help='Low-pass cutoff frequency in Hz, default: 2')
    args = parser.parse_args()
    if args.fps <= 0 or args.playback_rate <= 0 or args.lowpass_cutoff <= 0 or args.max_image_dt < 0:
        parser.error('fps, playback-rate, and lowpass-cutoff must be positive; max-image-dt cannot be negative')

    viewer = GripperRgbViewer(
        args.dataset.expanduser().resolve(),
        args.value,
        args.fps,
        args.playback_rate,
        args.max_image_dt,
        args.lowpass_cutoff,
    )
    viewer.show()


if __name__ == '__main__':
    main()
