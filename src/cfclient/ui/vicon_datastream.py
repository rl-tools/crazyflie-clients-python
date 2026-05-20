#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
#  Copyright (C) 2026
#
#  Crazyflie Nano Quadcopter Client

"""
Vicon DataStream forwarding support.
"""

from dataclasses import dataclass
import logging
import math
import os
import time

from PyQt6.QtCore import QThread
from PyQt6.QtCore import pyqtSignal


logger = logging.getLogger(__name__)

DEFAULT_VICON_DATASTREAM_HOST = '192.154.4.123'
DEFAULT_VICON_OBJECT_NAME = 'crazyflie'
DEFAULT_VICON_POSITION_SCALE = 0.001
DEFAULT_VICON_FORWARD_RATE_HZ = 100.0


@dataclass(frozen=True)
class ViconDataStreamConfig:
    host: str = DEFAULT_VICON_DATASTREAM_HOST
    object_name: str = DEFAULT_VICON_OBJECT_NAME
    segment_name: str | None = None
    position_scale: float = DEFAULT_VICON_POSITION_SCALE
    forward_rate_hz: float = DEFAULT_VICON_FORWARD_RATE_HZ

    @property
    def forward_period(self):
        return 1.0 / self.forward_rate_hz


def _env_bool(value, default=False):
    if value is None:
        return default
    return value.lower() in ('1', 'true', 'yes', 'on')


def _env_float(env, name, default):
    value = env.get(name)
    if value is None:
        return default

    try:
        parsed = float(value)
    except ValueError:
        logger.warning('Invalid %s=%s, using %s', name, value, default)
        return default

    if parsed <= 0:
        logger.warning('Invalid %s=%s, using %s', name, value, default)
        return default
    return parsed


def vicon_datastream_enabled(env=os.environ):
    explicit = env.get('VICON_DATASTREAM')
    if explicit is not None:
        return _env_bool(explicit)

    return 'VICON_DATASTREAM_HOST' in env


def vicon_datastream_config_from_env(env=os.environ):
    object_name = env.get('VICON_OBJECT_NAME')
    if object_name is None:
        object_name = env.get('VICON_SUBJECT_NAME', DEFAULT_VICON_OBJECT_NAME)

    segment_name = env.get('VICON_SEGMENT_NAME') or None

    return ViconDataStreamConfig(
        host=env.get('VICON_DATASTREAM_HOST', DEFAULT_VICON_DATASTREAM_HOST),
        object_name=object_name,
        segment_name=segment_name,
        position_scale=_env_float(
            env, 'VICON_POSITION_SCALE', DEFAULT_VICON_POSITION_SCALE),
        forward_rate_hz=_env_float(
            env, 'VICON_FORWARD_RATE_HZ', DEFAULT_VICON_FORWARD_RATE_HZ),
    )


class ViconDataStreamWorker(QThread):
    vicon_data_signal = pyqtSignal(dict)
    status_signal = pyqtSignal(str)
    error_signal = pyqtSignal(str)

    def __init__(self, config):
        super().__init__()
        self._config = config
        self._running = True
        self._segment_name = config.segment_name
        self._client = None
        self._last_pose_print = 0

    def run(self):
        client = None

        try:
            import pyvicon_datastream as pv

            client = pv.PyViconDatastream()
            self._client = client
            result = client.connect(self._config.host)
            if result != pv.Result.Success:
                self._report_error(
                    'Connection to Vicon DataStream host {} failed: {}'
                    .format(self._config.host, result))
                return

            self._report_status(
                'Connected to Vicon DataStream host {}'.format(
                    self._config.host))
            self._configure_client(client, pv)
            self._read_frames(client, pv)
        except Exception as e:
            self._report_error('Vicon DataStream error: {}'.format(e))
        finally:
            self._disconnect(client)
            self._client = None

    def stop(self):
        self._running = False
        self._disconnect(self._client)
        if not self.wait(2000):
            logger.warning('Timed out waiting for Vicon DataStream worker')

    def _configure_client(self, client, pv):
        actions = (
            ('enable segment data', client.enable_segment_data),
            ('set stream mode',
             lambda: client.set_stream_mode(pv.StreamMode.ServerPush)),
            ('set axis mapping',
             lambda: client.set_axis_mapping(
                 pv.Direction.Forward, pv.Direction.Left, pv.Direction.Up)),
        )

        for action, function in actions:
            result = function()
            if result != pv.Result.Success:
                raise RuntimeError(
                    'Could not {}: {}'.format(action, result))

    def _read_frames(self, client, pv):
        last_frame_error = 0
        last_segment_error = 0

        while self._running:
            loop_start = time.time()
            frame_result = client.get_frame()
            if frame_result != pv.Result.Success:
                last_frame_error = self._warn_periodically(
                    last_frame_error,
                    'Could not get Vicon frame: {}'.format(frame_result))
                self._sleep_until_next_period(loop_start)
                continue

            try:
                if self._segment_name is None:
                    self._segment_name = self._resolve_segment_name(client)
            except RuntimeError as e:
                last_segment_error = self._warn_periodically(
                    last_segment_error, str(e))
                self._sleep_until_next_period(loop_start)
                continue

            pose = self._read_pose(client)
            if pose is not None:
                self.vicon_data_signal.emit(pose)
                self._print_pose_periodically(pose)

            self._sleep_until_next_period(loop_start)

    def _resolve_segment_name(self, client):
        root_segment = client.get_subject_root_segment_name(
            self._config.object_name)
        if root_segment:
            self._report_status(
                'Using Vicon segment {} for object {}'.format(
                    root_segment, self._config.object_name))
            return root_segment

        segment_count = client.get_segment_count(self._config.object_name)
        if segment_count:
            segment_name = client.get_segment_name(self._config.object_name, 0)
            if segment_name:
                self._report_status(
                    'Using Vicon segment {} for object {}'.format(
                        segment_name, self._config.object_name))
                return segment_name

        subjects = self._subject_names(client)
        if subjects:
            raise RuntimeError(
                'Could not find Vicon object {}. Available subjects: {}'
                .format(self._config.object_name, ', '.join(subjects)))

        raise RuntimeError(
            'Could not find Vicon object {}. No subjects available.'
            .format(self._config.object_name))

    def _subject_names(self, client):
        subject_count = client.get_subject_count()
        subjects = []
        for index in range(subject_count):
            name = client.get_subject_name(index)
            if name:
                subjects.append(name)
        return subjects

    def _read_pose(self, client):
        translation = client.get_segment_global_translation(
            self._config.object_name, self._segment_name)
        quaternion = client.get_segment_global_quaternion(
            self._config.object_name, self._segment_name)

        if translation is None or quaternion is None:
            return None

        position = [float(value) * self._config.position_scale
                    for value in translation]
        orientation = self._normalize_quaternion(quaternion)
        if orientation is None:
            return None

        return {
            'object': self._config.object_name,
            'segment': self._segment_name,
            'pose': {
                'position': {
                    'x': position[0],
                    'y': position[1],
                    'z': position[2],
                },
                'orientation': {
                    'x': orientation[0],
                    'y': orientation[1],
                    'z': orientation[2],
                    'w': orientation[3],
                },
            },
        }

    def _normalize_quaternion(self, quaternion):
        # pyvicon-datastream returns w, x, y, z; Crazyflie expects x, y, z, w.
        qw, qx, qy, qz = [float(value) for value in quaternion]
        norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
        if norm == 0:
            logger.warning('Skipping Vicon pose with zero quaternion')
            return None

        return qx / norm, qy / norm, qz / norm, qw / norm

    def _sleep_until_next_period(self, loop_start):
        elapsed = time.time() - loop_start
        sleep_time = self._config.forward_period - elapsed
        if sleep_time > 0:
            time.sleep(sleep_time)

    def _warn_periodically(self, last_warning, message):
        now = time.time()
        if now - last_warning >= 1.0:
            self._report_error(message)
            return now
        return last_warning

    def _print_pose_periodically(self, message):
        now = time.time()
        if now - self._last_pose_print < 1.0:
            return

        self._last_pose_print = now
        pose = message['pose']
        position = pose['position']
        orientation = pose['orientation']
        print(
            '[vicon] received pose from {} object={} segment={} '
            'pos=({:.3f}, {:.3f}, {:.3f})m '
            'quat=({:.3f}, {:.3f}, {:.3f}, {:.3f})'.format(
                self._config.host,
                message['object'],
                message['segment'],
                position['x'],
                position['y'],
                position['z'],
                orientation['x'],
                orientation['y'],
                orientation['z'],
                orientation['w']),
            flush=True)

    def _report_status(self, message):
        logger.info(message)
        print('[vicon] {}'.format(message), flush=True)
        self.status_signal.emit(message)

    def _report_error(self, message):
        logger.warning(message)
        print('[vicon] {}'.format(message), flush=True)
        self.error_signal.emit(message)

    def _disconnect(self, client):
        if client is None:
            return

        try:
            if client.is_connected():
                client.disconnect()
                self._report_status('Disconnected from Vicon DataStream host')
        except Exception as e:
            logger.warning('Error disconnecting from Vicon DataStream: %s', e)
