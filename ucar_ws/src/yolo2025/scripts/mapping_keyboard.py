#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""Manual mapping keyboard controller for the UCar ROS Melodic workspace.

This program is intentionally run in its own terminal while mapping.launch is
running.  It publishes only /cmd_vel and provides two map actions:
  s: save gmapping's current /map to /tmp
  t: replace the active task map with that saved map, then delete the /tmp map
"""

from __future__ import print_function

import os
import re
import select
import shutil
import subprocess
import sys
import termios
import time
import tty

import rospy
from geometry_msgs.msg import Twist


DEFAULT_TASK_MAP_DIR = '/home/ucar/ucar_ws/src/ucar_nav/maps'
DEFAULT_TASK_MAP_NAME = 'iflysse_2026_direct'
DEFAULT_STAGING_PREFIX = '/tmp/ucar_mapping_current'


class MappingKeyboard(object):
    def __init__(self):
        rospy.init_node('mapping_keyboard')
        self.linear_speed = float(rospy.get_param('~linear_speed', 0.12))
        self.angular_speed = float(rospy.get_param('~angular_speed', 0.45))
        self.linear_speed_step = float(rospy.get_param('~linear_speed_step', 0.02))
        self.angular_speed_step = float(rospy.get_param('~angular_speed_step', 0.05))
        self.min_linear_speed = float(rospy.get_param('~min_linear_speed', 0.02))
        self.max_linear_speed = float(rospy.get_param('~max_linear_speed', 0.50))
        self.min_angular_speed = float(rospy.get_param('~min_angular_speed', 0.10))
        self.max_angular_speed = float(rospy.get_param('~max_angular_speed', 1.50))
        self.command_timeout = float(rospy.get_param('~command_timeout', 0.35))
        self.task_map_dir = rospy.get_param('~task_map_dir', DEFAULT_TASK_MAP_DIR)
        self.task_map_name = rospy.get_param('~task_map_name', DEFAULT_TASK_MAP_NAME)
        self.staging_prefix = rospy.get_param('~staging_prefix', DEFAULT_STAGING_PREFIX)

        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.last_motion_time = 0.0
        self.moving = False
        self.saved_this_session = False
        self.watchdog = rospy.Timer(rospy.Duration(0.05), self._watchdog)
        rospy.on_shutdown(self.stop)

    def stop(self):
        self.cmd_pub.publish(Twist())
        self.moving = False

    def _watchdog(self, _event):
        if self.moving and time.time() - self.last_motion_time > self.command_timeout:
            self.stop()

    def command(self, linear_x=0.0, angular_z=0.0):
        cmd = Twist()
        cmd.linear.x = linear_x
        cmd.angular.z = angular_z
        self.cmd_pub.publish(cmd)
        self.last_motion_time = time.time()
        self.moving = abs(linear_x) > 0.0 or abs(angular_z) > 0.0

    def show_speeds(self):
        print('Current mapping speeds: linear=%.2f m/s, angular=%.2f rad/s' %
              (self.linear_speed, self.angular_speed))

    def adjust_linear_speed(self, direction):
        self.linear_speed = min(
            self.max_linear_speed,
            max(self.min_linear_speed, self.linear_speed + direction * self.linear_speed_step))
        self.show_speeds()

    def adjust_angular_speed(self, direction):
        self.angular_speed = min(
            self.max_angular_speed,
            max(self.min_angular_speed, self.angular_speed + direction * self.angular_speed_step))
        self.show_speeds()

    def _staging_paths(self):
        return self.staging_prefix + '.pgm', self.staging_prefix + '.yaml'

    def save_map(self):
        self.stop()
        pgm_path, yaml_path = self._staging_paths()
        for path in (pgm_path, yaml_path):
            if os.path.exists(path):
                os.remove(path)

        rospy.loginfo('Saving current gmapping /map ...')
        try:
            result = subprocess.call(
                ['rosrun', 'map_server', 'map_saver', '-f', self.staging_prefix])
        except OSError as exc:
            rospy.logerr('Could not start map_saver: %s', exc)
            return

        if result != 0 or not os.path.isfile(pgm_path) or not os.path.isfile(yaml_path):
            rospy.logerr('Map save failed; keep driving and confirm gmapping publishes /map.')
            return

        self.saved_this_session = True
        rospy.loginfo('Map saved temporarily. Press t to replace task map, or keep mapping and press s again.')

    @staticmethod
    def _yaml_for_task(source_yaml, task_pgm_name):
        lines = source_yaml.splitlines()
        rewritten = []
        found_image = False
        for line in lines:
            if re.match(r'^\s*image\s*:', line):
                rewritten.append('image: ' + task_pgm_name)
                found_image = True
            else:
                rewritten.append(line)
        if not found_image:
            rewritten.insert(0, 'image: ' + task_pgm_name)
        return '\n'.join(rewritten) + '\n'

    def install_task_map(self):
        self.stop()
        staged_pgm, staged_yaml = self._staging_paths()
        if not self.saved_this_session or not os.path.isfile(staged_pgm) or not os.path.isfile(staged_yaml):
            rospy.logerr('No map saved in this session. Press s after gmapping has built the map first.')
            return

        target_pgm = os.path.join(self.task_map_dir, self.task_map_name + '.pgm')
        target_yaml = os.path.join(self.task_map_dir, self.task_map_name + '.yaml')
        temp_pgm = target_pgm + '.mapping_tmp'
        temp_yaml = target_yaml + '.mapping_tmp'
        try:
            if not os.path.isdir(self.task_map_dir):
                raise IOError('task map directory does not exist: ' + self.task_map_dir)
            with open(staged_yaml, 'r') as handle:
                source_yaml = handle.read()
            shutil.copyfile(staged_pgm, temp_pgm)
            with open(temp_yaml, 'w') as handle:
                handle.write(self._yaml_for_task(source_yaml, os.path.basename(target_pgm)))
            os.rename(temp_pgm, target_pgm)
            os.rename(temp_yaml, target_yaml)
            os.remove(staged_pgm)
            os.remove(staged_yaml)
        except (IOError, OSError) as exc:
            for path in (temp_pgm, temp_yaml):
                if os.path.exists(path):
                    os.remove(path)
            rospy.logerr('Task map was not replaced: %s', exc)
            return

        self.saved_this_session = False
        rospy.loginfo('Task map replaced: %s. Quit mapping, then restart 2026.launch to load it.', target_yaml)

    def handle_key(self, key):
        if key in ('w', '\x1b[A'):
            self.command(linear_x=self.linear_speed)
        elif key in ('x', '\x1b[B'):
            self.command(linear_x=-self.linear_speed)
        elif key in ('a', '\x1b[D'):
            self.command(angular_z=self.angular_speed)
        elif key in ('d', '\x1b[C'):
            self.command(angular_z=-self.angular_speed)
        elif key == ' ':
            self.stop()
        elif key == 's':
            self.save_map()
        elif key == 't':
            self.install_task_map()
        elif key in ('+', '='):
            self.adjust_linear_speed(1)
        elif key == '-':
            self.adjust_linear_speed(-1)
        elif key == ']':
            self.adjust_angular_speed(1)
        elif key == '[':
            self.adjust_angular_speed(-1)
        elif key == 'v':
            self.show_speeds()
        elif key == 'q':
            return False
        return True

    @staticmethod
    def read_key():
        key = sys.stdin.read(1)
        if key == '\x1b' and select.select([sys.stdin], [], [], 0.02)[0]:
            key += sys.stdin.read(1)
            if select.select([sys.stdin], [], [], 0.02)[0]:
                key += sys.stdin.read(1)
        return key

    def run(self):
        if not sys.stdin.isatty():
            rospy.logfatal('mapping_keyboard needs an interactive terminal; run it with rosrun, not inside roslaunch.')
            return
        print('Manual mapping keys: W/up forward, X/down reverse, A/left turn left, D/right turn right')
        print('space stop | s save current map | t replace task map | q quit')
        print('+/= linear faster, - linear slower | ] angular faster, [ angular slower | v show speeds')
        print('Motion stops automatically %.2f s after key repeat stops.' % self.command_timeout)
        self.show_speeds()

        settings = termios.tcgetattr(sys.stdin)
        try:
            tty.setraw(sys.stdin.fileno())
            while not rospy.is_shutdown():
                ready, _, _ = select.select([sys.stdin], [], [], 0.10)
                if ready and not self.handle_key(self.read_key()):
                    break
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
            self.stop()


if __name__ == '__main__':
    try:
        if not sys.stdin.isatty():
            sys.stderr.write('mapping_keyboard needs an interactive terminal; run it with rosrun, not inside roslaunch.\n')
            sys.exit(2)
        MappingKeyboard().run()
    except rospy.ROSInterruptException:
        pass
