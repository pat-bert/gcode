import logging
from configparser import ConfigParser
from math import pi
from typing import Optional

from src.Coordinate import Coordinate
from src.MelfaCoordinateService import MelfaCoordinateService
from src.clients.IClient import ClientError
from src.clients.TcpClientR3 import TcpClientR3
from src.gcode.GCmd import GCmd
from src.prechecks.configs import melfa_rv_4a
from src.prechecks.dataclasses import Constraints, Increments, Extrusion
from src.prechecks.exceptions import CartesianLimitViolation, ConfigurationChangesError, JointVelocityViolation, \
    NoValidPathFound, WorkspaceViolation
from src.prechecks.prechecks import check_traj
from src.protocols.R3Protocol import R3Protocol


def check_trajectory(config_f='./../config.ini', gcode_f='./../test.gcode', ip: Optional[str] = None,
                     port: Optional[int] = None):
    """
    Validate a trajectory for a given robot setup.
    :param config_f: File path for the configuration file
    :param gcode_f: File path for the input G-Code file
    :param ip: Optional host address to be used to resolve robot parameters directly
    :param port: Optional port to be used to resolve robot parameters directly
    :return:
    """
    print(f'Reading G-Code from file {gcode_f}.')
    with open(gcode_f, 'r') as f:
        cmd_raw = f.readlines()

    commands = [GCmd.read_cmd_str(cmd_str.strip()) for cmd_str in cmd_raw if not cmd_str.startswith(GCmd.COMMENT)]
    commands = [cmd for cmd in commands if cmd is not None]

    config_parser = ConfigParser()
    config_parser.read(config_f)

    read_param_from_robot = False
    home_position = None
    cartesian_limits = None
    joint_limits = None

    # Parameters that always need to be configured within the config file
    max_jnt_speed = config_parser.get('prechecks', 'max_joint_speed')
    joint_velocity_limits = [float(i) for i in max_jnt_speed.split(', ')]

    # Increments for sampling
    inc_distance_mm = float(config_parser.get('prechecks', 'ds_mm'))
    inc_angle_tool_deg = float(config_parser.get('prechecks', 'dphi_deg', fallback=2 * pi))

    urdf = config_parser.get('prechecks', 'urdf_path')
    default_acc = float(config_parser.get('prechecks', 'default_acc'))

    # Extrusion parameters
    extrusion_height = float(config_parser.get('prechecks', 'extrusion_height'))
    extrusion_width = float(config_parser.get('prechecks', 'extrusion_width'))

    # Tool Center Point offsets
    tool_offset_x = float(config_parser.get('prechecks', 'tool_offset_x', fallback=0))
    tool_offset_y = float(config_parser.get('prechecks', 'tool_offset_y', fallback=0))
    tool_offset_z = float(config_parser.get('prechecks', 'tool_offset_z', fallback=0))

    # Heat bed offsets
    x_hb = float(config_parser.get('prechecks', 'bed_origin_x', fallback=0))
    y_hb = float(config_parser.get('prechecks', 'bed_origin_y', fallback=0))
    z_hb = float(config_parser.get('prechecks', 'bed_origin_z', fallback=0))

    robot_config = melfa_rv_4a(atoff=tool_offset_z, rtoff=tool_offset_x)

    if ip is not None and port is not None:
        # Parameters can be read from the robot
        try:
            print(f'Attempting to read configuration from {ip}:{port}')
            tcp_client = TcpClientR3(host=ip, port=port)
            protocol = R3Protocol(tcp_client, MelfaCoordinateService())
            home_position = protocol.get_safe_pos().values
            cartesian_limits = protocol.get_xyz_borders()
            joint_limits = protocol.get_joint_borders()
        except ClientError as e:
            print(f'Reading parameters from robot failed due to {e}. Falling back to config file.')
        else:
            read_param_from_robot = True

    if not read_param_from_robot:
        # Parameters that need to be configured in the config file if they are not read from the robot
        home_pos_str = config_parser.get('prechecks', 'home_joints')
        home_position = [float(i) for i in home_pos_str.split(', ')]
        cartesian_limits_str = config_parser.get('prechecks', 'xyz_limits')
        cartesian_limits = [float(i) for i in cartesian_limits_str.split(', ')]
        joint_limits_str = config_parser.get('prechecks', 'joint_limits')
        joint_limits = [float(i) for i in joint_limits_str.split(', ')]

    print('\nConfiguration parameters:')
    print(f'Joint home position in rad: {home_position}')
    print(f'Cartesian limits in mm: {cartesian_limits}')
    print(f'Joint limits in rad: {joint_limits}')
    print(f'Maximum joint velocities in rad/s: {joint_velocity_limits}')
    print(f'Checking resolution in mm: {inc_distance_mm}')
    print(f'URDF filepath: {urdf}')
    print(f'Default acceleration set to {default_acc} mm/s^2')
    print('\n')

    hb_offset = Coordinate([x_hb, y_hb, z_hb], 'XYZ')

    # Create the constraints
    traj_constraint = Constraints(cartesian_limits, joint_limits, joint_velocity_limits)

    # Create the increments
    incs = Increments(inc_distance_mm, inc_angle_tool_deg / 180 * pi)

    # Create the extrusion data
    extr = Extrusion(extrusion_height, extrusion_width)

    try:
        # Check the trajectory
        check_traj(commands, robot_config, traj_constraint, home_position, incs, extr, default_acc, urdf, hb_offset)
    except (CartesianLimitViolation, WorkspaceViolation):
        logging.error('Please verify that the limits are correct and check the positioning of the part.')
        raise
    except ConfigurationChangesError:
        logging.error('Robot configuration transitions are not supported within a coherent segment.')
        raise
    except JointVelocityViolation:
        logging.error('Autoamtically reducing the speed is not yet supported.')
        raise
    except NoValidPathFound:
        logging.error('Recreating graph (partially) with different tool orientation is unsupported.')
        raise
