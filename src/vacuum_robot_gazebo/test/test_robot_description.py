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
SIM_EFFECTIVE_WHEEL_SEPARATION = 0.2834
WHEEL_TREAD_WIDTH = 0.0300
MOTOR_GEAR_RATIO = 45.0
MOTOR_NO_LOAD_RPM = 130.0
MOTOR_RATED_RPM = 100.0
MOTOR_NO_LOAD_RADPS = MOTOR_NO_LOAD_RPM * 2.0 * math.pi / 60.0
MOTOR_STALL_TORQUE_NM = 3.6 * 0.0980665
MOTOR_BODY_LENGTH = 0.068
MOTOR_BODY_RADIUS = 0.0125


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
            'left_motor', 'right_motor', 'left_wheel', 'right_wheel',
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
        assert floats(
            joints['base_to_left_motor'].find('origin').get('xyz')
        ) == [0.0, 0.0934, 0.0]
        assert floats(
            joints['base_to_right_motor'].find('origin').get('xyz')
        ) == [0.0, -0.0934, 0.0]
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
            assert math.isclose(
                float(limit.get('effort')),
                MOTOR_STALL_TORQUE_NM,
                rel_tol=1.0e-8,
            )
            assert math.isclose(
                float(limit.get('velocity')),
                MOTOR_NO_LOAD_RADPS,
                rel_tol=1.0e-10,
            )

        for link_name in ('left_wheel', 'right_wheel'):
            cylinder = links[link_name].find('collision/geometry/cylinder')
            assert math.isclose(float(cylinder.get('radius')), WHEEL_RADIUS)
            assert math.isclose(float(cylinder.get('length')), WHEEL_TREAD_WIDTH)

        for link_name in ('left_motor', 'right_motor'):
            motor = links[link_name]
            assert motor.find('collision') is None
            assert motor.find('inertial') is None
            cylinder = motor.find('visual/geometry/cylinder')
            assert math.isclose(
                float(cylinder.get('radius')),
                MOTOR_BODY_RADIUS,
            )
            assert math.isclose(
                float(cylinder.get('length')),
                MOTOR_BODY_LENGTH,
            )

        transmissions = {
            transmission.get('name'): transmission
            for transmission in self.urdf.findall('transmission')
        }
        assert set(transmissions) == {
            'left_ga25_transmission',
            'right_ga25_transmission',
        }
        for side in ('left', 'right'):
            transmission = transmissions[f'{side}_ga25_transmission']
            assert (
                transmission.findtext('type')
                == 'transmission_interface/SimpleTransmission'
            )
            assert (
                transmission.find('joint').get('name')
                == f'{side}_wheel_joint'
            )
            assert math.isclose(
                float(transmission.findtext('actuator/mechanicalReduction')),
                MOTOR_GEAR_RATIO,
            )

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

        base_visuals = {
            visual.get('name'): visual
            for visual in links['base_link'].findall('visual')
        }
        for side, expected_y in (('left', 0.0934), ('right', -0.0934)):
            visual = base_visuals[f'{side}_ga25_motor_visual']
            pose = floats(visual.findtext('pose'))
            assert math.isclose(pose[1], expected_y)
            cylinder = visual.find('geometry/cylinder')
            assert math.isclose(
                float(cylinder.findtext('radius')),
                MOTOR_BODY_RADIUS,
            )
            assert math.isclose(
                float(cylinder.findtext('length')),
                MOTOR_BODY_LENGTH,
            )

        joints = {
            joint.get('name'): joint for joint in self.sdf.findall('joint')
        }
        for joint_name in ('left_wheel_joint', 'right_wheel_joint'):
            limit = joints[joint_name].find('axis/limit')
            assert math.isclose(
                float(limit.findtext('effort')),
                MOTOR_STALL_TORQUE_NM,
                rel_tol=1.0e-8,
            )
            assert math.isclose(
                float(limit.findtext('velocity')),
                MOTOR_NO_LOAD_RADPS,
                rel_tol=1.0e-10,
            )

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
        simulation = self.profile['simulation_calibration']
        assert math.isclose(
            simulation['effective_wheel_separation_m'],
            SIM_EFFECTIVE_WHEEL_SEPARATION,
        )
        assert math.isclose(
            simulation['wheel_separation_multiplier'],
            SIM_EFFECTIVE_WHEEL_SEPARATION / WHEEL_SEPARATION,
            rel_tol=1.0e-10,
        )
        drive = self.profile['drive']
        assert drive['motor_model'] == 'GA25_encoder_130rpm'
        assert drive['motor_count'] == 2
        assert drive['rated_voltage_v'] == 12.0
        assert drive['rotation_direction'] == 'CW_CCW'
        assert drive['gearbox_ratio'] == MOTOR_GEAR_RATIO
        assert drive['armature_speed_rpm'] == 6000.0
        assert drive['nominal_output_rpm'] == MOTOR_NO_LOAD_RPM
        assert drive['rated_load_output_rpm'] == MOTOR_RATED_RPM
        assert drive['no_load_current_per_motor_a'] == 0.060
        assert drive['rated_load_current_per_motor_a'] == 0.300
        assert drive['stall_current_per_motor_a'] == 1.3
        assert math.isclose(
            drive['rated_output_torque_nm'],
            drive['rated_output_torque_kgf_cm'] * 0.0980665,
        )
        assert math.isclose(
            drive['stall_output_torque_nm'],
            drive['stall_output_torque_kgf_cm'] * 0.0980665,
        )
        assert math.isclose(
            drive['no_load_output_angular_speed_radps'],
            MOTOR_NO_LOAD_RADPS,
            rel_tol=1.0e-10,
        )
        assert math.isclose(
            drive['rated_load_output_angular_speed_radps'],
            MOTOR_RATED_RPM * 2.0 * math.pi / 60.0,
            rel_tol=1.0e-10,
        )
        assert math.isclose(
            drive['theoretical_no_load_linear_speed_mps'],
            MOTOR_NO_LOAD_RADPS * WHEEL_RADIUS,
            rel_tol=1.0e-10,
        )
        assert math.isclose(
            drive['theoretical_rated_load_linear_speed_mps'],
            MOTOR_RATED_RPM * 2.0 * math.pi / 60.0 * WHEEL_RADIUS,
            rel_tol=1.0e-10,
        )
        assert math.isclose(
            drive['maximum_joint_effort_nm'],
            MOTOR_STALL_TORQUE_NM,
            rel_tol=1.0e-8,
        )
        gearbox_prediction = (
            drive['armature_speed_rpm'] / drive['gearbox_ratio']
        )
        tolerance = (
            drive['nominal_output_rpm']
            * drive['no_load_output_speed_tolerance_percent']
            / 100.0
        )
        assert abs(gearbox_prediction - drive['nominal_output_rpm']) <= tolerance

        # Encoder tick geometry is intentionally unavailable; populating these
        # fields without a PPR and decode mode would create false odometry.
        encoder = drive['encoder']
        assert all(
            encoder[key] is None for key in (
                'location',
                'pulses_per_revolution',
                'quadrature_decode',
                'encoder_ticks_per_rev',
                'radians_per_tick',
                'metres_per_tick',
            )
        )

        power = self.profile['power']
        assert power['topology'] == '4S4P'
        assert power['total_cell_count'] == 16
        assert (
            power['series_cell_count'] * power['parallel_cell_count']
            == power['total_cell_count']
        )
        cell = power['cell']
        pack = power['pack']
        assert math.isclose(
            cell['capacity_ah'] * cell['discharge_rate_c'],
            cell['stated_discharge_current_a'],
        )
        assert math.isclose(
            pack['nominal_voltage_assumption_v'],
            power['series_cell_count']
            * cell['nominal_voltage_assumption_v'],
        )
        assert math.isclose(
            pack['full_voltage_assumption_v'],
            power['series_cell_count']
            * cell['full_voltage_assumption_v'],
        )
        assert math.isclose(
            pack['capacity_ah'],
            power['parallel_cell_count'] * cell['capacity_ah'],
        )
        assert math.isclose(
            pack['nominal_energy_assumption_wh'],
            pack['nominal_voltage_assumption_v'] * pack['capacity_ah'],
        )
        assert math.isclose(
            pack['theoretical_continuous_discharge_current_a'],
            power['parallel_cell_count']
            * cell['stated_discharge_current_a'],
        )
        assert power['motor_rail']['regulator_required'] is True
        assert (
            power['motor_rail']['regulated_voltage_v']
            == drive['rated_voltage_v']
        )
        assert (
            pack['full_voltage_assumption_v']
            > power['motor_rail']['regulated_voltage_v']
        )

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
