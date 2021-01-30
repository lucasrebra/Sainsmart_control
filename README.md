# Sainsmart_control
Desarrollo de un proyecto robótico en ROS con el brazo Hobby de Sainsmart.

# Lanzamiento de brazo robótico con controlador Rosserial

Antes de realizar el lanzamiento se debe tener el arduino debidamente programado mediante 
el API de arduino. También se podría programar en todo caso con programas como el Visual Studio
Code utilizando la extensión platformio, muy útil en el caso de programar boards de todo tipo.

```
roslaunch mr_description sm_bringup_moveit.launch

sudo chmod 666 /dev/ttyACM0

rosrun rosserial_python serial_node.py /dev/ttyACM0

rosrun acciona_motor nodo_simple.py

```

Mediante estos comandos realizaremos lo siguiente:

* 1- Lanzar la simulación y el move_group para crear trayectorias para nuestro robot.

* 2- Damos permisos al puerto del arduino. Realmente esto lo deberíamos haber realizado con anterioridad para programar el board pero quiero ponerlo para recordarlo.

* 3- Corremos el nodo que envolverá a nuestro arduino y que lo tratará como un nodo más de ROS lo cual nos ayudará a interactuar correctamente con él.

* 4- Lanzamos el nodo que se encargará de recoger las posiciones angulares que son devueltas como feedback de la simulación y las clona "imprimiéndolas" en nuestro robot.





