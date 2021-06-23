import argparse

'''
* @author: vladddd46
* @brief:  Registers flags of command-line arguments.
*          Returns object, which has values of flags, which user specified 
*          while running script. Flags are also have default values in order
*          user not specified them.
'''



def parseArguments():
	argparser = argparse.ArgumentParser(description='Simulation of motorbike falling')
	argparser.add_argument(
		'--sip',
		default='127.0.0.1',
		help='IP which will be used, to send sensor`s data to'
	)
	argparser.add_argument(
		'--sport',
		default=7000,
		type=int,
		help='Port which will be used, to send sensor`s data to'
	)
	argparser.add_argument(
		'--rip',
		default='127.0.0.1',
		help='IP which will be used, to receive angle of steer from'
	)
	argparser.add_argument(
		'--rport',
		default=7001,
		type=int,
		help='Port which will be used, to receive angle of steer from'
	)
	argparser.add_argument(
		'--carla_ip',
		default='127.0.0.1',
		help='ip carla will be run on'
	)
	argparser.add_argument(
		'--carla_port',
		default=5000,
		type=int,
		help='port carla will be run on'
	)
	argparser.add_argument(
		'--drawer_ip',
		default='127.0.0.1',
		help='ip, drawer sender will send data to'
	)
	argparser.add_argument(
		'--drawer_port',
		default=7002,
		type=int,
		help='port drawer sender will connect to'
	)
	argparser.add_argument(
		'--drawer_on',
		default=0,
		type=int,
		help='defines, wether drawer-sender module must be on'
	)
	argparser.add_argument(
		'-v', '--verbose',
		action='store_true',
		dest='debug',
		help='print debug information')
	argparser.add_argument(
		'-a', '--autopilot',
		action='store_true',
		help='enable autopilot')
	argparser.add_argument(
		'--res',
		metavar='WIDTHxHEIGHT',
		# default='1280x720',
		default='800x600',
		help='window resolution (default: 1280x720)')
	argparser.add_argument(
		'--filter',
		metavar='PATTERN',
		default='vehicle.kawasaki.ninja',
		help='actor filter (default: "vehicle.*")')
	argparser.add_argument(
		'--rolename',
		metavar='NAME',
		default='hero',
		help='actor role name (default: "hero")')
	argparser.add_argument(
		'--gamma',
		default=2.2,
		type=float,
		help='Gamma correction of the camera (default: 2.2)')
	return argparser.parse_args()
