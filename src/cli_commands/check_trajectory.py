import logging
from configparser import ConfigParser
from math import pi
from typing import Optional

from src.Coordinate import Coordinate
from src.clients.TcpClientR3 import TcpClientR3
from src.gcode.GCmd import GCmd
from src.prechecks.configs import melfa_rv_4a
from src.prechecks.dataclasses import Constraints, Increments, Extrusion
from src.prechecks.exceptions import CartesianLimitViolation, ConfigurationChangesError, JointVelocityViolation, \
    NoValidPathFound, WorkspaceViolation
from src.prechecks.prechecks import check_traj
from src.protocols.R3Protocol import R3Protocol


def check_trajectory(config_f='./../config.ini', gcode_f='./../test.gcode', ip: Optional[str] = None,
                     port: Optional[int] = 0):
    """
    Validate a trajectory for a given robot setup.
    :param config_f: File path for the configuration file
    :param gcode_f: File path for the input G-Code file
    :param ip: Optional host address to be used to resolve robot parameters directly
    :param port: Optional port to be used to resolve robot parameters directly
    :return:
    """
    with open(gcode_f, 'r') as f:
        cmd_raw = f.readlines()

    commands = [GCmd.read_cmd_str(cmd_str.strip()) for cmd_str in cmd_raw if not cmd_str.startswith(GCmd.COMMENT)]
    commands = [cmd for cmd in commands if cmd is not None]

    config_parser = ConfigParser()
    config_parser.read(config_f)

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

    if ip is not None:
        # Parameters can be read from the robot
        tcp_client = TcpClientR3(host=ip, port=port)
        # TODO Configure setup for reading parameters from robot correctly
        protocol = R3Protocol(tcp_client)
        home_position = protocol.get_safe_pos().values
        cartesian_limits = protocol.get_xyz_borders()
        joint_limits = protocol.get_joint_borders()
    else:
        # Parameters that need to be configured in the config file if they are not read from the robot
        home_pos_str = config_parser.get('prechecks', 'home_joints')
        home_position = [float(i) for i in home_pos_str.split(', ')]
        cartesian_limits_str = config_parser.get('prechecks', 'xyz_limits')
        cartesian_limits = [float(i) for i in cartesian_limits_str.split(', ')]
        joint_limits_str = config_parser.get('prechecks', 'joint_limits')
        joint_limits = [float(i) for i in joint_limits_str.split(', ')]

    # Heat bed offset
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
    except (CartesianLimitViolation, WorkspaceViolation) as e:
        logging.exception('Fatal error occured: {}'.format("\n".join(e.args)))
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
