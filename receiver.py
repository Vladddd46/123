import socket
import json
import carla
import random

'''
* @author: vladddd46
* @brief:  Server thread, which receives data(steer-wheel angle)
* 		   from remote client(ip=args.rip:args.rport) and sends it via
* 		   queue to simulationEntryPoint in order to change angle of steer-wheel.
'''


def setCenterOfVehicleMass(vehicle, x, y, z):
	physics_control = vehicle.get_physics_control()
	physics_control.center_of_mass = carla.Vector3D(x=x, y=y, z=z)
	vehicle.apply_physics_control(physics_control)




def ReceiveSteerWheelAngleFromRemoteServer_THREAD(parsedArgs, motorbike):
	sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
	sock.bind((parsedArgs.rip, parsedArgs.rport))
	sock.listen()
	
	# sendSocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
	# sendSocket.connect(("127.0.0.1", 7778))

	y = 0
	while True:
		connectionSocket, addr = sock.accept()
		with connectionSocket:
			while True:
				data = connectionSocket.recv(1024)
				try:
					if not data:
						break
					data = data.decode("ascii")
					parsedData = json.loads(data)
					steerWheelAngle = parsedData["angle"]
					# sendSocket.sendall(bytes(str(steerWheelAngle), 'utf-8'))
					motorbike.apply_control(carla.VehicleControl(throttle=0, steer=float(steerWheelAngle)))
					y += 0.1
					# setCenterOfVehicleMass(motorbike, 0, y, 0)
					data = ""
				except Exception as e:
					print(e)
					pass
	sock.close()
	# sendSocket.close()