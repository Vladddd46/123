import matplotlib.pyplot as plt
import time
import random
import socket 
import json
xdata = []
ydata = []

startTime = time.time()
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.bind(("127.0.0.1", 7778))
sock.listen()

xMaxLim = 60
xMinLim = 0
yMaxLim = 1
yMinLim = -1

while True:
	connectionSocket, addr = sock.accept()
	with connectionSocket:
		while True:

			plt.show()
			axes = plt.gca()
			axes.set_xlim(xMinLim, xMaxLim)
			axes.set_ylim(yMinLim, yMaxLim)
			line, = axes.plot(xdata, ydata, 'r-')
			 
			for i in range(10000000):
				try:
					data = connectionSocket.recv(1024)
					data = data.decode("utf-8")
					print("==",data)
					data = float(data)
				except Exception as e:
					print(e)
					continue
				passedTime = time.time() - startTime
				if passedTime >= xMaxLim:
					xMaxLim += 60
					axes.set_xlim(xMinLim, xMaxLim)

				xdata.append(time.time() - startTime)
				ydata.append(data)
				line.set_xdata(xdata)
				line.set_ydata(ydata)
				plt.draw()
				plt.pause(1e-17)
				time.sleep(0.1)

plt.show()
sock.close()

