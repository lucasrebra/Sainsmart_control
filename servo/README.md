##Esta carpeta guarda el .ino utilizado para el nodo rosserial
Funcionamiento sencillo. 
CODIGO

```
roscode
cd /dev
sudo chmod 666 ttyACMX
rosrun rosserial_python rosserial_node.py /dev/ttyACMX
```

Esto lo que realizará será lanzar el paquete rosserial para poder utilizar nuestro arduino, en mi caso un Arduino UNO como un nodo más de ROS. Esto tiene varias ventajas por la que he utilizado esta solución, y es que, Arduino es una plataforma en la que me siento bastante cómodo programando y que tengo cierta experiencia. Además, trabajar con arduino siempre nos ofrece una cantidad impresionante para poder trabajar en diversos proyectos de electrónica. En este caso, utilizar arduino nos permite tener acceso a la librería PCA9685 <Adafruit_PWMServoDriver.h>, específica para la utilización del driver del que disponemos.

