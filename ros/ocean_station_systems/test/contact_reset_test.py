#!/usr/bin/env python3
"""Exercise real named contact messages through two complete Gazebo resets."""

import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import threading
import time
import uuid

from gz.msgs10.clock_pb2 import Clock
from gz.msgs10.contacts_pb2 import Contacts
from gz.transport13 import Node


WORLD = '''<sdf version="1.9"><world name="contact_reset">
  <physics name="default" type="ignored">
    <max_step_size>0.005</max_step_size><real_time_factor>1</real_time_factor>
  </physics>
  <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
  CONTACT_PLUGIN
  <model name="ground"><static>true</static><link name="link">
    <collision name="collision"><geometry><plane><normal>0 0 1</normal>
      <size>10 10</size></plane></geometry></collision>
  </link></model>
  <model name="box"><pose>0 0 1.5 0 0 0</pose><link name="link">
    <inertial><mass>1</mass><inertia><ixx>0.1667</ixx><iyy>0.1667</iyy>
      <izz>0.1667</izz></inertia></inertial>
    <collision name="collision"><geometry><box><size>1 1 1</size></box></geometry></collision>
    <sensor name="native_name" type="contact"><contact>
      <collision>collision</collision></contact></sensor>
    <sensor name="override_name" type="contact"><topic>/outer_topic_unused</topic>
      <contact><collision>collision</collision><topic>/contact_reset/custom</topic></contact>
    </sensor>
  </link></model>
</world></sdf>'''


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--plugin-dir', required=True)
    parser.add_argument('--output', required=True)
    parser.add_argument('--native', action='store_true',
                        help='Reproduce native Harmonic contact loss after reset')
    args = parser.parse_args()
    output = Path(args.output).resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    os.environ['GZ_PARTITION'] = 'contact-reset-' + uuid.uuid4().hex
    os.environ['GZ_SIM_SYSTEM_PLUGIN_PATH'] = (
        args.plugin_dir + os.pathsep + os.environ.get('GZ_SIM_SYSTEM_PLUGIN_PATH', ''))
    plugin = ('<plugin filename="gz-sim-contact-system" name="gz::sim::systems::Contact"/>'
              if args.native else
              '<plugin filename="libocean-replay-contact-system.so" '
              'name="ocean_station_systems::ReplayContact"/>')
    topics = ('/world/contact_reset/model/box/link/link/sensor/native_name/contact',
              '/contact_reset/custom')
    lock = threading.Condition()
    counts = {topic: 0 for topic in topics}
    names = {topic: set() for topic in topics}
    clock = {'time': None, 'sequence': 0}
    report = {'passed': False, 'native': args.native, 'phases': []}

    def clock_callback(message):
        with lock:
            clock['time'] = message.sim.sec + message.sim.nsec * 1e-9
            clock['sequence'] += 1
            lock.notify_all()

    def contact_callback(topic):
        def received(message):
            stamp = message.header.stamp.sec + message.header.stamp.nsec * 1e-9
            with lock:
                if clock['time'] is not None and stamp > clock['time'] + 0.10:
                    return
                for contact in message.contact:
                    first, second = contact.collision1.name, contact.collision2.name
                    pair = (first, second)
                    if (any(item.endswith('box::link::collision') for item in pair) and
                            any(item.endswith('ground::link::collision') for item in pair)):
                        counts[topic] += 1
                        names[topic].add(pair)
                lock.notify_all()
        return received

    def wait_until(predicate, timeout, reason):
        deadline = time.monotonic() + timeout
        with lock:
            while not predicate():
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    raise AssertionError(reason)
                lock.wait(min(remaining, 0.1))

    node = Node()
    node.subscribe(Clock, '/world/contact_reset/clock', clock_callback)
    for topic in topics:
        node.subscribe(Contacts, topic, contact_callback(topic))

    def control(reset=False):
        # The subprocess leaves Python callbacks free to receive /clock while
        # Gazebo performs its synchronous transport service response.
        request = 'pause: true reset: {all: true}' if reset else 'pause: false'
        result = subprocess.run(
            ['gz', 'service', '-s', '/world/contact_reset/control',
             '--reqtype', 'gz.msgs.WorldControl', '--reptype', 'gz.msgs.Boolean',
             '--timeout', '5000', '--req', request],
            capture_output=True, text=True, timeout=8)
        assert result.returncode == 0 and 'data: true' in result.stdout, (
            'Gazebo rejected world control: ' + result.stdout + result.stderr)

    with tempfile.TemporaryDirectory(prefix='ocean-contact-reset-') as temp:
        world_path = Path(temp) / 'world.sdf'
        world_path.write_text(WORLD.replace('CONTACT_PLUGIN', plugin))
        with output.with_suffix('.log').open('w') as log:
            server = subprocess.Popen(['gz', 'sim', '-s', '-r', '-v', '3', str(world_path)],
                                      stdout=log, stderr=subprocess.STDOUT,
                                      start_new_session=True)
            try:
                for phase in range(3):
                    wait_until(lambda: all(value >= 5 for value in counts.values()),
                               10, f'No real named contact messages in phase {phase}')
                    with lock:
                        record = {'phase': phase, 'sim_time': clock['time'],
                                  'messages': dict(counts),
                                  'collision_pairs': {key: sorted(value)
                                                      for key, value in names.items()}}
                    report['phases'].append(record)
                    print(f'Phase {phase}: named contacts received on both topics', flush=True)
                    if phase == 2:
                        break
                    with lock:
                        before_sequence = clock['sequence']
                    control(reset=True)
                    wait_until(lambda: clock['sequence'] > before_sequence and
                               clock['time'] is not None and clock['time'] <= 0.01,
                               5, 'World did not rewind to paused time zero')
                    with lock:
                        counts.update({topic: 0 for topic in topics})
                        names.update({topic: set() for topic in topics})
                    control(reset=False)
                report['passed'] = True
            except Exception as error:
                report['error'] = str(error)
                with lock:
                    report['failed_phase'] = {
                        'phase': phase, 'sim_time': clock['time'],
                        'messages': dict(counts),
                        'collision_pairs': {key: sorted(value)
                                            for key, value in names.items()},
                    }
                raise
            finally:
                os.killpg(server.pid, signal.SIGTERM)
                try:
                    server.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    os.killpg(server.pid, signal.SIGKILL)
                    server.wait()
                output.write_text(json.dumps(report, indent=2) + '\n')
    print('Named contact sensing passed initial run and two full world resets.')


if __name__ == '__main__':
    main()
