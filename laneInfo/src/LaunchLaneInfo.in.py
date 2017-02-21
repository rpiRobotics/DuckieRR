#!/usr/bin/env python
from rr_utils import (RRNodeInterface, LaunchRRNode, FormatRobdefString)
import yaml
import sys,argparse

if __name__ == '__main__':
    # Parse command line arguments
    parser = argparse.ArgumentParser(
        description='Initialize the laneInfo nodes')
    parser.add_argument('--port',type=int,default=0,
        help='TCP port to host service on' +\
        '(will auto-generate if not specified)')
    parser.add_argument('--configLD', type=open,
        help='A config file for the line detector (Otherwise use Default)')
    parser.add_argument('--configGP', type=open,
        help='A config file for the ground projection (Otherwise use Default)')
    parser.add_argument('--configLANE', type=open,
        help='A config file for the lane info (Otherwise use Default)')
    parser.add_argument('args', nargs=argparse.REMAINDER)

    args = parser.parse_args(sys.argv[1:])

    config_LD = args.configLD
    if config_LD is None:
        config_LD = '${DEFAULT_LD_PARAMS}'

    config_GP = args.configGP
    if config_GP is None:
        config_GP = '${DEFAULT_GP_PARAMS}'

    config_LANE = args.configLANE
    if config_LANE is None:
        config_LANE = '${DEFAULT_LANE_PARAMS}'

    launch_file = """\
node_name: Duckiebot.Lane

objects:
    - name: Duckiebot
      robdef: ${DUCKIEBOT_ROBDEF}

    - name: LineDetector
      robdef: ${LINEDETECTOR_ROBDEF}
      class: LineDetectorNode.LineDetectorNode
      configuration: %s 

    - name: GroundProjection
      robdef: ${GROUNDPROJECTION_ROBDEF}
      class: GroundProjectionNode.GroundProjectionNode
      configuration: %s

    - name: LaneInfo
      robdef: ${LANEINFO_ROBDEF}
      class: LaneInfoNode.LaneInfoNode
      configuration: %s 

tcp_port: %d
    """%(config_LD, config_GP, config_LANE, args.port)
    
    launch_config = yaml.load(launch_file)
    LaunchRRNode(**launch_config)