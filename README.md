# NavigationPuebla
Autonomous Navegation for El Primer Foro Aeroespacial Mexicano

Pasos a seguir para utilizar el código: 
1. Correr en la jetson:
  Archivos externos al paquete que han de correrse:
    1. roslaunch bus_can_drive drive_can.cpp
    2. roslaunch ______ imu.launch
    3. roscd servos/scripts
    4. python3 lab.py
  Archivos del código que han de correrse
    1. rosrun navigationpuebla odometry.py
    2. rosrun navigationpuebla mapping.py
    3. rosrun navigationpuebla arm_controller2.py
    4. rosrun navigationpuebla deteccion7.py
2. Archvios que puedes corrrer en tu computadora teniendo el ROS_MASTER:URI= http://ip_jetson:11311
    1. rosrun navigationpuebla PyGameSim10.py (Este archivo simula el movimiento del rover, correrlo sólamente si quieres visualizarlo)
    2. rosrun navigationpuebla route6.py (Este archvio manda al rover a correr la ruta determinada en el código)
