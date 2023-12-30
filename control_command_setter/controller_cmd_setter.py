#! /usr/bin/env python
# -*- coding: utf-8 -*-

import time
import sys
import rclpy
import pygame
from rclpy.node import Node
from autoware_auto_control_msgs.msg import AckermannControlCommand


class Controller_cmd_setter(Node):
    def __init__(self, joystick=None):
        super().__init__('Controller_cmd_setter')

        # simple_pure_pursuit からトピックを受信
        self.create_subscription(AckermannControlCommand, "/control/command/pre_control_cmd", self.onTrigger, 1)

        # raw_vehicle_cmd_converter にトピックを送信
        self.vehicle_inputs_pub_ = self.create_publisher(AckermannControlCommand, "/control/command/control_cmd", 1)

        # 初期値を時速 20km/h とする
        self.target_vel = 20.0

        # 比例係数 (simple_pure_pursuitと同値である必要あり)
        self.speed_proportional_gain_ = 1.0

        # log を出力するかどうか
        self.log = False

        # 0.1 秒ごとにコントローラーの入力を更新
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.timer_callback)

        if joystick is None:
            print("ジョイスティックが見つかりません。")
            sys.exit()
        self.joystick = joystick

    # コントローラーからの入力を更新
    def timer_callback(self):
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                sys.exit()

        self.target_vel  =  self.target_vel + self.joystick.get_axis(3) * (-1.0)
        
        # Aボタンで速度を +10 km/s
        if self.joystick.get_button(0): 
            self.target_vel += 10.0
        
        # Aボタンで速度を -10 km/s
        if self.joystick.get_button(1): 
            self.target_vel -= 10.0

        self.target_vel = min(max( self.target_vel, 0), 150)
        self.get_logger().info("target_vel:{}".format(self.target_vel))

    # 制御コマンドを送受信
    def onTrigger(self, msg):
        # km/h ->m/s
        target_vel = self.target_vel / 3.6

        # 現在の速度を逆計算 
        current_longitudinal_vel = msg.longitudinal.speed - (msg.longitudinal.acceleration / self.speed_proportional_gain_) 

        if self.log:
            self.get_logger().info("current vel {}".format(current_longitudinal_vel * 3.6))
        
        msg.longitudinal.speed = float(target_vel)

        # accel を計算
        msg.longitudinal.acceleration = float(self.speed_proportional_gain_ * (target_vel - current_longitudinal_vel))

        # トピックを送信
        self.vehicle_inputs_pub_.publish(msg)

def main(args=None):
    print('Hi from Controller_cmd_setter.')

    # rosノードを初期化
    rclpy.init(args=args)

    # Pygameの初期化
    pygame.init()

    # pygame を初期化
    pygame.joystick.init()

    # 利用可能なジョイスティックの数を取得
    joystick_count = pygame.joystick.get_count()

    if joystick_count == 0:
        print("ジョイスティックが見つかりません。")
        sys.exit()

    # 0番のジョイスティックを取得
    joystick = pygame.joystick.Joystick(0)
    joystick.init()

    node = Controller_cmd_setter(joystick=joystick)
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()