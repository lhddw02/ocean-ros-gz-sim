"""Generate a calibrated AprilTag and an isolated, configurable landing scene."""
import argparse
import json
import math
from pathlib import Path
import shutil
import xml.etree.ElementTree as ET

import cv2
import numpy as np
import yaml


FAMILIES = {
    'tag16h5': cv2.aruco.DICT_APRILTAG_16h5,
    'tag25h9': cv2.aruco.DICT_APRILTAG_25h9,
    'tag36h11': cv2.aruco.DICT_APRILTAG_36h11,
}


def load_configuration(share, config=None):
    path = Path(config) if config else Path(share) / 'config/landing.yaml'
    value = yaml.safe_load(path.read_text(encoding='utf-8'))
    if not isinstance(value, dict) or not isinstance(value.get('scene'), dict):
        raise ValueError('landing configuration must contain a scene mapping')
    if not isinstance(value.get('controller'), dict):
        raise ValueError('landing configuration must contain a controller mapping')
    return value


def generate_tag(path, family, tag_id, tag_size, quiet_margin):
    if family not in FAMILIES:
        raise ValueError('Unsupported tag family: ' + str(family))
    dictionary = cv2.aruco.getPredefinedDictionary(FAMILIES[family])
    if isinstance(tag_id, bool) or not isinstance(tag_id, int) or not 0 <= tag_id < len(dictionary.bytesList):
        raise ValueError(f'tag_id must be from 0 to {len(dictionary.bytesList) - 1} for {family}')
    if not math.isfinite(tag_size) or not 0.05 <= tag_size <= 1.0:
        raise ValueError('tag_size must be between 0.05 and 1.0 metres')
    if not math.isfinite(quiet_margin) or not 0.05 <= quiet_margin <= 0.3:
        raise ValueError('quiet_margin must be between 0.05 and 0.3 metres')
    # Integer cells preserve the dictionary bits exactly. A white quiet zone is
    # part of the board texture, but not part of the calibrated tag_size.
    cells = dictionary.markerSize + 2
    pixels = cells * 100
    marker = cv2.aruco.drawMarker(dictionary, tag_id, pixels, borderBits=1)
    margin = round(pixels * quiet_margin / tag_size)
    board = np.full((pixels + margin * 2, pixels + margin * 2), 255, np.uint8)
    board[margin:margin + pixels, margin:margin + pixels] = marker
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    if not cv2.imwrite(str(path), board):
        raise RuntimeError('Unable to write marker image: ' + str(path))
    # Verify the real generated bitmap rather than accepting an arbitrary image.
    _, identifiers, _ = cv2.aruco.detectMarkers(board, dictionary)
    if identifiers is None or tag_id not in identifiers.flatten():
        raise RuntimeError('Generated AprilTag failed its own dictionary decode')
    return {'family': family, 'id': tag_id, 'black_border_size_m': tag_size,
            'board_size_m': tag_size + 2 * quiet_margin,
            'texture_size_px': int(board.shape[0])}


def build_scene(share, output, scene):
    share, output = Path(share), Path(output)
    output.mkdir(parents=True, exist_ok=True)
    station = output / 'models/floating_station'
    shutil.copytree(share / 'models/floating_station', station, dirs_exist_ok=True)
    info = generate_tag(station / 'materials/textures/apriltag.png',
                        scene['tag_family'], scene['tag_id'],
                        float(scene['tag_size']), float(scene['quiet_margin']))
    station_xml = ET.parse(station / 'model.sdf')
    mesh = station_xml.find(".//visual[@name='apriltag_board']/geometry/mesh")
    scale = info['board_size_m'] / 0.6
    ET.SubElement(mesh, 'scale').text = f'{scale:.9f} {scale:.9f} 1'
    ET.indent(station_xml, space='  ')
    station_xml.write(station / 'model.sdf', encoding='utf-8', xml_declaration=True)

    world_xml = ET.parse(share / 'worlds/ocean.sdf')
    world = world_xml.find('world')
    world.find('physics/max_step_size').text = '0.0025'
    ET.SubElement(world, 'plugin', filename='ocean-replay-contact-system',
                  name='ocean_station_systems::ReplayContact')
    for include in world.findall('include'):
        if include.findtext('uri') == 'model://duck':
            include.find('pose').text = '-6 5 0 0 0 0'
        if include.findtext('uri') == 'model://spherical_buoy':
            include.find('pose').text = '6 5 0 0 0 0'
    for model, prefix in [('floating_station', 'station'), ('landing_drone', 'drone')]:
        coordinates = [float(scene[prefix + '_' + axis]) for axis in 'xyz']
        if not all(math.isfinite(value) for value in coordinates):
            raise ValueError('Spawn coordinates must be finite')
        include = ET.SubElement(world, 'include')
        ET.SubElement(include, 'uri').text = 'model://' + model
        ET.SubElement(include, 'pose').text = ' '.join(map(str, coordinates)) + ' 0 0 0'
    world.find("gui/plugin[@filename='MinimalScene']/camera_pose").text = '4 -7 3.2 0 0.20 1.93'
    # The preview is the detector's annotated image, bridged back into Gazebo.
    # Keep this panel in the generated landing world only.
    preview = ET.SubElement(world.find('gui'), 'plugin',
                            filename='ImageDisplay', name='Downward camera')
    gui = ET.SubElement(preview, 'gz-gui')
    ET.SubElement(gui, 'title').text = 'Downward camera - AprilTag'
    for kind, key, value in [
            ('string', 'state', 'floating'), ('bool', 'showTitleBar', 'true'),
            ('bool', 'resizable', 'true'), ('double', 'width', '480'),
            ('double', 'height', '405'), ('double', 'z', '2')]:
        ET.SubElement(gui, 'property', type=kind, key=key).text = value
    anchors = ET.SubElement(gui, 'anchors', target='3D View')
    ET.SubElement(anchors, 'line', own='right', target='right')
    ET.SubElement(anchors, 'line', own='top', target='top')
    ET.SubElement(preview, 'topic').text = '/landing/debug_image'
    ET.SubElement(preview, 'topic_picker').text = 'false'
    replay = ET.SubElement(world.find('gui'), 'plugin',
                           filename='LandingReplay', name='Landing replay')
    gui = ET.SubElement(replay, 'gz-gui')
    ET.SubElement(gui, 'title').text = 'Landing demo'
    for kind, key, value in [
            ('string', 'state', 'floating'), ('bool', 'showTitleBar', 'true'),
            ('bool', 'resizable', 'false'), ('double', 'width', '350'),
            ('double', 'height', '210'), ('double', 'z', '3')]:
        ET.SubElement(gui, 'property', type=kind, key=key).text = value
    anchors = ET.SubElement(gui, 'anchors', target='3D View')
    ET.SubElement(anchors, 'line', own='left', target='left')
    ET.SubElement(anchors, 'line', own='top', target='top')
    ET.indent(world_xml, space='  ')
    world_path = output / 'landing.sdf'
    world_xml.write(world_path, encoding='utf-8', xml_declaration=True)
    (output / 'scene.json').write_text(json.dumps({'marker': info, 'scene': scene}, indent=2) + '\n')
    return world_path, output / 'models'


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--share', required=True)
    parser.add_argument('--output', required=True)
    parser.add_argument('--config')
    args = parser.parse_args()
    config = load_configuration(args.share, args.config)
    world, resources = build_scene(args.share, args.output, config['scene'])
    print(f'Landing scene: {world}; resources: {resources}')


if __name__ == '__main__':
    main()
