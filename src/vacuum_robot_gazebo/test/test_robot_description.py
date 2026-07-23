# Copyright 2026 Adaptive Pivot-G2 Research Team
# Licensed under the Apache License, Version 2.0

"""Regression contract for the two-wheel CAD, Gazebo, Nav2, and hardware profile."""

import math
from pathlib import Path
import xml.etree.ElementTree as ET

import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
WHEEL_RADIUS = 0.0425
WHEEL_SEPARATION = 0.2548
SIM_EFFECTIVE_WHEEL_SEPARATION = 0.2809
WHEEL_TREAD_WIDTH = 0.0300


def floats(text):
    """Parse a whitespace-separated vector."""
    return [float(value) for value in text.split()]


def named(elements, name):
    """Select one XML element by its name attribute."""
    return next(element for element in elements if element.get('name') == name)


def assert_positive_definite_inertia(inertia):
    """Check the three Sylvester criteria for a symmetric 3x3 inertia matrix."""
    ixx = float(inertia.get('ixx') or inertia.findtext('ixx'))
    iyy = float(inertia.get('iyy') or inertia.findtext('iyy'))
    izz = float(inertia.get('izz') or inertia.findtext('izz'))
    ixy = float(inertia.get('ixy') or inertia.findtext('ixy'))
    ixz = float(inertia.get('ixz') or inertia.findtext('ixz'))
    iyz = float(inertia.get('iyz') or inertia.findtext('iyz'))
    determinant = (
        ixx * iyy * izz + 2.0 * ixy * ixz * iyz
        - ixx * iyz * iyz - iyy * ixz * ixz - izz * ixy * ixy
    )
    assert ixx > 0.0
    assert ixx * iyy - ixy * ixy > 0.0
    assert determinant > 0.0


class TestRobotDescriptionContract:
    """Keep all duplicated runtime formats on the same measured geometry."""

    @classmethod
    def setup_class(cls):
        cls.urdf = ET.parse(PACKAGE_ROOT / 'urdf' / 'vacuum_robot.urdf').getroot()
        sdf_root = ET.parse(
            PACKAGE_ROOT / 'models' / 'vacuum_robot' / 'model.sdf'
        ).getroot()
        cls.sdf = sdf_root.find('model')
        with (PACKAGE_ROOT / 'config' / 'real_robot_profile.yaml').open() as stream:
            cls.profile = yaml.safe_load(stream)
        with (PACKAGE_ROOT / 'config' / 'nav2_params.yaml').open() as stream:
            cls.nav2 = yaml.safe_load(stream)
        with (PACKAGE_ROOT / 'config' / 'bridge.yaml').open() as stream:
            cls.bridge = yaml.safe_load(stream)

    def test_urdf_tree_and_sensor_frames(self):
        links = {link.get('name') for link in self.urdf.findall('link')}
        assert links == {
            'base_link', 'base_footprint', 'laser', 'imu_link',
            'left_wheel', 'right_wheel',
        }
        parents = {}
        for joint in self.urdf.findall('joint'):
            child = joint.find('child').get('link')
            assert child not in parents
            parents[child] = joint.find('parent').get('link')
        assert set(parents) == links - {'base_link'}

        joints = {joint.get('name'): joint for joint in self.urdf.findall('joint')}
        assert floats(joints['base_to_laser'].find('origin').get('xyz')) == [0.0, 0.0, 0.10892]
        assert floats(joints['base_to_imu'].find('origin').get('xyz')) == [0.0, 0.0, -0.0128]
        assert floats(joints['base_footprint_joint'].find('origin').get('xyz')) == [
            0.0, 0.0, -WHEEL_RADIUS,
        ]

    def test_urdf_physics_matches_rolling_treads(self):
        links = {link.get('name'): link for link in self.urdf.findall('link')}
        # base_link inertia is intentionally kept only in SDF because KDL does
        # not support inertia on the root link used by robot_state_publisher.
        assert links['base_link'].find('inertial') is None
        for link_name in ('left_wheel', 'right_wheel'):
            inertial = links[link_name].find('inertial')
            assert inertial is not None
            assert float(inertial.find('mass').get('value')) > 0.0
            assert_positive_definite_inertia(inertial.find('inertia'))

        joints = {joint.get('name'): joint for joint in self.urdf.findall('joint')}
        left_y = floats(joints['left_wheel_joint'].find('origin').get('xyz'))[1]
        right_y = floats(joints['right_wheel_joint'].find('origin').get('xyz'))[1]
        assert math.isclose(left_y - right_y, WHEEL_SEPARATION, abs_tol=1.0e-12)
        for joint_name in ('left_wheel_joint', 'right_wheel_joint'):
            limit = joints[joint_name].find('limit')
            assert float(limit.get('effort')) > 0.0
            assert float(limit.get('velocity')) > 0.0

        for link_name in ('left_wheel', 'right_wheel'):
            cylinder = links[link_name].find('collision/geometry/cylinder')
            assert math.isclose(float(cylinder.get('radius')), WHEEL_RADIUS)
            assert math.isclose(float(cylinder.get('length')), WHEEL_TREAD_WIDTH)

        base_collisions = {
            collision.get('name'): collision
            for collision in links['base_link'].findall('collision')
        }
        expected_boxes = {
            'lower_center_collision': [0.44, 0.20, 0.10],
            'front_cross_collision': [0.12, 0.34, 0.10],
            'rear_cross_collision': [0.12, 0.34, 0.10],
            'upper_deck_collision': [0.44, 0.34, 0.04],
        }
        for collision_name, size in expected_boxes.items():
            assert floats(
                base_collisions[collision_name].find('geometry/box').get('size')
            ) == size
        for corner in ('front_left', 'front_right', 'rear_left', 'rear_right'):
            sphere = base_collisions[f'{corner}_ball_collision'].find('geometry/sphere')
            assert math.isclose(float(sphere.get('radius')), 0.006)

    def test_sdf_plugins_sensors_and_geometry_are_consistent(self):
        links = {link.get('name'): link for link in self.sdf.findall('link')}
        left_y = floats(links['left_wheel'].findtext('pose'))[1]
        right_y = floats(links['right_wheel'].findtext('pose'))[1]
        assert math.isclose(left_y - right_y, WHEEL_SEPARATION, abs_tol=1.0e-12)

        for link_name in ('left_wheel', 'right_wheel'):
            cylinder = links[link_name].find('collision/geometry/cylinder')
            assert math.isclose(float(cylinder.findtext('radius')), WHEEL_RADIUS)
            assert math.isclose(float(cylinder.findtext('length')), WHEEL_TREAD_WIDTH)
            assert_positive_definite_inertia(links[link_name].find('inertial/inertia'))
        assert_positive_definite_inertia(links['base_link'].find('inertial/inertia'))

        collisions = {
            collision.get('name'): collision
            for collision in links['base_link'].findall('collision')
        }
        assert {
            'lower_center_collision', 'front_cross_collision',
            'rear_cross_collision', 'upper_deck_collision',
        }.issubset(collisions)

        sensors = {
            sensor.get('name'): sensor for sensor in links['base_link'].findall('sensor')
        }
        assert sensors['rplidar_a1m8'].findtext('topic') == '/scan'
        assert sensors['rplidar_a1m8'].findtext('gz_frame_id') == 'laser'
        assert floats(sensors['rplidar_a1m8'].findtext('pose'))[2] == 0.10892
        assert sensors['bno055_imu'].findtext('topic') == '/imu/data'
        assert sensors['bno055_imu'].findtext('gz_frame_id') == 'imu_link'
        assert floats(sensors['bno055_imu'].findtext('pose'))[2] == -0.0128

        plugins = {plugin.get('name'): plugin for plugin in self.sdf.findall('plugin')}
        drive = plugins['gz::sim::systems::DiffDrive']
        # Gazebo's multi-contact support needs a calibrated effective track;
        # the physical separation remains WHEEL_SEPARATION everywhere else.
        assert math.isclose(
            float(drive.findtext('wheel_separation')),
            SIM_EFFECTIVE_WHEEL_SEPARATION,
        )
        assert math.isclose(float(drive.findtext('wheel_radius')), WHEEL_RADIUS)
        assert drive.findtext('topic') == '/cmd_vel'
        assert drive.findtext('odom_topic') == '/odom'
        assert drive.findtext('tf_topic') == '/tf'
        assert 'gz::sim::systems::JointStatePublisher' in plugins
        ground_truth = plugins['gz::sim::systems::OdometryPublisher']
        assert ground_truth.findtext('odom_frame') == 'world'
        assert ground_truth.findtext('robot_base_frame') == 'base_link'
        assert ground_truth.findtext('odom_topic') == '/ground_truth/odom'

    def test_nav2_hardware_profile_and_bridge_use_the_same_contract(self):
        robot = self.profile['robot']
        assert math.isclose(robot['wheel_diameter_m'] / 2.0, WHEEL_RADIUS)
        assert math.isclose(robot['wheel_tread_width_m'], WHEEL_TREAD_WIDTH)
        assert math.isclose(robot['wheel_separation_m'], WHEEL_SEPARATION)
        encoder = self.profile['drive']['encoder']
        expected_radians = 2.0 * math.pi / encoder['encoder_ticks_per_rev']
        expected_metres = math.pi * robot['wheel_diameter_m'] / encoder['encoder_ticks_per_rev']
        assert math.isclose(encoder['radians_per_tick'], expected_radians, rel_tol=1.0e-8)
        assert math.isclose(encoder['metres_per_tick'], expected_metres, rel_tol=1.0e-8)

        smoother = self.nav2['smoother_server']['ros__parameters']['pivot_g2']
        assert math.isclose(smoother['wheel_separation'], WHEEL_SEPARATION)
        for costmap_name in ('local_costmap', 'global_costmap'):
            parameters = self.nav2[costmap_name][costmap_name]['ros__parameters']
            footprint = yaml.safe_load(parameters['footprint'])
            xs = [point[0] for point in footprint]
            ys = [point[1] for point in footprint]
            assert math.isclose(max(xs) - min(xs), robot['body_length_m'])
            assert math.isclose(max(ys) - min(ys), robot['body_width_m'])

        bridge = {entry['ros_topic_name']: entry for entry in self.bridge}
        assert set(bridge) == {
            '/clock', '/joint_states', '/odom', '/tf', '/scan', '/imu/data', '/cmd_vel',
            '/ground_truth/odom',
        }
        assert bridge['/cmd_vel']['direction'] == 'ROS_TO_GZ'
        for topic in (
            '/clock', '/joint_states', '/odom', '/tf', '/scan', '/imu/data',
            '/ground_truth/odom',
        ):
            assert bridge[topic]['direction'] == 'GZ_TO_ROS'

    def test_mesh_assets_are_present(self):
        mesh_dir = PACKAGE_ROOT / 'models' / 'vacuum_robot' / 'meshes'
        for filename in ('base_link.stl', 'left_wheel_link_1.stl', 'right_wheel_link_1.stl'):
            mesh = mesh_dir / filename
            assert mesh.is_file()
            assert mesh.stat().st_size > 84
