#Ya tenemos una implementacion de la kinematica en este script
#seria adecuado meterlo dentro de una funcion con nuestros datos
#en vez de aplicarlo directamente como es en este script


import math
import numpy as np

from sympy import *
from sympy.interactive import printing
printing.init_printing(use_latex = True)


theta_i = Symbol("theta_i")
alpha_i1 = Symbol("alpha_i1")
a_i1 = Symbol("a_i1")
d_i = Symbol("d_i")

a_2 = Symbol("a_2")
d_i = Symbol("d_i")
d_4 = Symbol("d_4")

theta_1 = Symbol("theta_1")
theta_2 = Symbol("theta_2")
theta_3 = Symbol("theta_3")
theta_4 = Symbol("theta_4")
theta_5 = Symbol("theta_5")
theta_6 = Symbol("theta_6")


T0_1_sim = Matrix([[cos(theta_1), 0, sin(theta_1), 40*cos(theta_1)],
          [sin(theta_1), 0, -cos(theta_1), 40*sin(theta_1)],
          [0, 1, 0, 110],
          [0, 0, 0, 1]])


T1_2_sim = Matrix([[cos(theta_2+math.pi/2), sin(theta_2+math.pi/2), 0, 127*cos(theta_2+math.pi/2)],
          [sin(theta_2+math.pi/2), -cos(theta_2+math.pi/2), 0, 127*sin(theta_2+math.pi/2)],
          [0, 0, -1, 0],
          [0,0,0,1]])


T2_3_sim = Matrix([[cos(theta_3+math.pi), 0, sin(theta_3+math.pi), -26*cos(theta_3+math.pi)],
          [sin(theta_3+math.pi),0, -cos(theta_3+math.pi), -26*sin(theta_3+math.pi)],
          [0, 1, 0, 0],
          [0,0,0,1]])


T3_4_sim = Matrix([[cos(theta_4),0 , sin(theta_4), 0],
          [sin(theta_4), 0, -cos(theta_4), 0],
          [0, 1, 0, 133],
          [0,0,0,1]])

T4_5_sim = Matrix([[cos(theta_5+math.pi), 0, sin(theta_5+ math.pi), 0],
          [sin(theta_5+math.pi), 0, -cos(theta_5+math.pi), 0],
          [0,1, 0, 0],
          [0,0,0,1]])

T5_6_sim = Matrix([[cos(theta_6+math.pi), -sin(theta_6+math.pi), 0, 0],
          [sin(theta_6+math.pi), cos(theta_6+math.pi), 0, 0],
          [0, 0, 1, 20],
          [0,0,0,1]])



T0_1_sim = T0_1_sim.subs(theta_1, 0)
T1_2_sim = T1_2_sim.subs(theta_2, 0)
T2_3_sim = T2_3_sim.subs(theta_3, 0)
T3_4_sim = T3_4_sim.subs(theta_4, 0)
T4_5_sim = T4_5_sim.subs(theta_5, 0)
T5_6_sim = T5_6_sim.subs(theta_6, 0)



TM = T0_1_sim * T1_2_sim * T2_3_sim * T3_4_sim * T4_5_sim * T5_6_sim
aux=np.array(TM)
Orientacion=aux[0:3,0:3]
Pos=aux[0:3,3]


print(TM)
print("HOLA")
print(Orientacion)
print("HOLA")
print(Pos)
