import socket
import time

'''
* @author: vladddd46
* @brief:  Sender thread. Gets IMU sensor`s data from queue 
*          and sends it via socket to remote server. 
'''

def sendImuDataToRemoteServer_THREAD(q, parsedArgs):
	while True:
		try:
			sendSocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
			sendSocket.connect((parsedArgs.sip, parsedArgs.sport))
		except  Exception as e:
			print("Exception while creating and connecting socket: ", e)
			time.sleep(1)

		while True:
			try:
				dataToSend = q.get() # getting imu data in json format as str.
				sendSocket.sendall(bytes(dataToSend, 'utf-8')) # sending data to balancer.
			except Exception as e:
				break
		try:
			sendSocket.close()
		except:
			pass