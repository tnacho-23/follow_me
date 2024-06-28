#!/usr/bin/env python

import rospy
from std_msgs.msg import Int32MultiArray, Float32MultiArray
from geometry_msgs.msg import Twist

class PositionController:
    def __init__(self):
        # Parámetros de control P
        self.kp_linear = 0.0002
        self.kp_angular = 0.001

        # Deseados
        self.target_x = 0.5  # Centro deseado en x (ajustar según tu configuración)
        self.target_y = 0.5  # Centro deseado en y (ajustar según tu configuración)
        self.target_width = 0.2  # Ancho deseado de la detección (ajustar según tu configuración)
        self.target_height = 0.2  # Alto deseado de la detección (ajustar según tu configuración)

        # Crear un suscriptor y un publicador
        self.pos_sub = rospy.Subscriber('/position', Int32MultiArray, self.pos_callback)
        self.cmd_vel_pub = rospy.Publisher('/RosAria/cmd_vel', Twist, queue_size=10)

    def pos_callback(self, data):
        
        # Leer los valores de la detección
        x1, y2, width, height = data.data

        print(width*height)
        # Calcular el centro de la detección
        detection_center_x = x1 + width / 2.0
        detection_center_y = y2 + height / 2.0

        
        if width*height > 200000:
            print('stop')
            # cmd_vel = Twist()
            # cmd_vel.linear.x = 0
            # cmd_vel.angular.z = 0
            # # Publicar el mensaje
            # self.cmd_vel_pub.publish(cmd_vel)
            error_x = 0
            error_height = 0
            alpha = 0
            # cmd_vel = Twist()
            # cmd_vel.linear.x = 0
            # cmd_vel.angular.z = 0
        # print(data.data)
        else:
            # Calcular el error en posición
            error_x = self.target_x - detection_center_x
            error_y = self.target_y - detection_center_y

            # Calcular el error en tamaño
            error_width = self.target_width - width
            error_height = self.target_height - height
            alpha = 0.4



        # Control P para velocidad lineal y angular
        linear_vel = -self.kp_linear * error_height
        angular_vel = self.kp_angular * error_x + alpha
        print(f'L={linear_vel}')
        print(f'w={angular_vel}')
        # Crear el mensaje de velocidad
        cmd_vel = Twist()
        cmd_vel.linear.x = linear_vel
        cmd_vel.angular.z = angular_vel

        # Publicar el mensaje
        self.cmd_vel_pub.publish(cmd_vel)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('position_controller')
    controller = PositionController()
    controller.run()
