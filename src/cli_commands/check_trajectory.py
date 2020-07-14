import logging
from configparser import ConfigParser
from typing import Optional

from MelfaCoordinateService import MelfaCoordinateService
from src.clients.IClient import ClientError
from src.clients.TcpClientR3 import TcpClientR3
from src.gcode.GCmd import GCmd
from src.prechecks.configs import melfa_rv_4a
from src.prechecks.exceptions import CartesianLimitViolation, ConfigurationChangesError, JointVelocityViolation, \
    NoValidPathFound, WorkspaceViolation
from src.prechecks.prechecks import Constraints, check_traj
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
    robot_config = melfa_rv_4a()

    config_parser = ConfigParser()
    config_parser.read(config_f)

    read_param_from_robot = False
    home_position = None
    cartesian_limits = None
    joint_limits = None

    if ip is not None and port is not None:
        print(f'Attempting to read configuration from {ip}:{port}')

        # Parameters can be read from the robot
        try:
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

    # Parameters that always need to be configured within the config file
    max_jnt_speed = config_parser.get('prechecks', 'max_joint_speed')
    joint_velocity_limits = [float(i) for i in max_jnt_speed.split(', ')]
    inc_distance_mm = float(config_parser.get('prechecks', 'ds_mm'))
    urdf_file_path = config_parser.get('prechecks', 'urdf_path')
    default_acc = float(config_parser.get('prechecks', 'default_acc'))

    print(f'Maximum joint velocities in rad/s: {joint_velocity_limits}')
    print(f'Checking resolution in mm: {inc_distance_mm}')
    print(f'URDF filepath: {urdf_file_path}')
    print(f'Default acceleration set to {default_acc} mm/s^2')
    print('\n')

    # Create the constraints
    traj_constraint = Constraints(cartesian_limits, joint_limits, joint_velocity_limits)

    try:
        # Check the trajectory
        check_traj(commands, robot_config, traj_constraint, home_position, inc_distance_mm, default_acc, urdf_file_path)
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
