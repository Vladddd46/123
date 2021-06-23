<h1>Carla Simulation client</h1>
<h4>Short description:</h4> carla client, which is used for simulation of self-balancing motorbike. In simulation motorbike is spawned. If velocity of motorbike is 0, motorbike starts falling. Motorbike has IMU(accelerometer, gyroscope, compass) sensor. Data from IMU sensor is sent to remote server(Raspberry pi) via socket. Remote server(Raspberry pi) calculates steer-angle, which should be applied to motorbike in order to prevent it from falling. Remote server(Raspberry pi) sends steer-angle to simulation via socket. Simulation receives steer-angle and apply it to motorbike. Due to these actions motorbike is getting prevented from falling.<br>

<h4>Shortly about carla:</h4> Carla simulator has client-server architecture. In our case, carla-server is running on remote server. Carla-server simulates world. Carla-clients can connect to carla-server via socket and make changes in world`s simulation(create actors, change weather, etc).<br>PS. By default gui runs on server side(it is run on UnrealEngine) but it is not an appropriate way for us. Our vision is: carla-server is running on remote server and every user can use client to interact with server and see gui on client`s side. Due to this, client was designed so, that gui starts on client side.(Pygame lib. is used for this.)
<br><br>

<h4>Files description:</h4>
1. main.py - main logic of simulation.<br>
2. argsParser.py - registration of command line flags<br>
3. receiver.py - receiver module, which receives steer-angle from remote server(Raspberry pi) and sends it via queue to simulationEntryPoint (RUNS AS NEW THREAD)<br>
4. sender.py - receives IMU sensor`s data from queue and sends it to remote server(Raspberry pi) via socket. (RUNS AS NEW THREAD)<br>
5. simulationEntryPoint.py - simulates falling of motorbike. Receives steer-angle from queue and applies it to motorbike in order to prevent it from falling.<br>
6. carla folder - contains carla lib.<br>
<br><br>

<h4>Quickstart:</h4>
python3 main.py <flags><br>
<b>Flag`s description:</b><br>
1. --sip=127.0.0.1 (by default) => send ip - ip imu sensor`s data will be sent to.<br>
2. --sport=7000 (by default) => send port - port imu sensor`s data will be sent to.<br>
3. --rip=127.0.0.1 (by default) => receive ip - ip, which receive steer-angle server will run on<br>
4. --rport=7001 (by default) => receive port - port, which receive steer-angle server will run on<br>
5. --carla_ip=127.0.0.1 (by default) => ip of carla-server.<br>
6. --carla_port=5000 (by default) => ip of carla-server.<br>
7. --filter=vehicle.kawasaki.ninja (by default) => vehiclem which will be spawned.<br>

<h4>Architecture description:</h4>
Has 3 threads (main, sender, receiver) and 2 queues. <b>Sender</b> works as client - gets data from queue_a(data in queue_a pushes from imu sensor`s callback) and sends it to remote server(Raspberry pi). <b>Receiver</b> receives data from remote server(Raspberry pi) and send it via queue_b in main thread(simulationEntryPoint). Main thread does all simulation job - simulates falling and changes steer-angle in order to prevent motorbike from falling.