#! /usr/bin/env python
# -*- coding: utf-8 -*-

import time
import sys
import math
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

        # ステアリングを手動にするか
        self.manual_steering = False
        # 現在のステアリング値
        self.target_steering = 0.0
        self.target_angle = 0.0
        # ホイールの長さ
        self.wheel_base = 2.9718
        # ジョイスティックの移動で何度動かすか
        self.steering_scale = 45.0
        self.button_cout = 0

        # 0.1 秒ごとにコントローラーの入力を更新
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.timer_callback)

        if joystick is None:
            print("ジョイスティックが見つかりません。")
            sys.exit()
        self.joystick = joystick

    # コントローラーからの入力を更新
    def timer_callback(self):
        self.button_cout += 1 
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                sys.exit()

        # Aボタンで速度を +10 km/s
        if self.joystick.get_button(0): 
            self.target_vel += 10.0
        
        # Xボタンで速度を -10 km/s
        if self.joystick.get_button(1): 
            self.target_vel -= 10.0

        # B ボタンでステアリングモードを切り替え
        if self.joystick.get_button(2):
            if self.button_cout > 2: #チャタリング防止
                self.manual_steering = not self.manual_steering
                self.button_cout = 0

        # Joystick の値を反映
        if self.joystick.get_hat(0):
            joystic_x, joystic_y = self.joystick.get_hat(0)
            self.target_vel  =  self.target_vel + joystic_x * 1.0

            if self.manual_steering:
                self.target_angle = self.target_angle + joystic_y * self.steering_scale
                self.target_angle = max(min(self.target_angle, 200), -200)
                self.target_steering = self.target_angle /(self.wheel_base * 360)

        # Yボタンで角度を０に戻す
        if self.joystick.get_button(3):
            if self.manual_steering:
                self.target_angle = 0.0
                self.target_steering = self.target_angle /(self.wheel_base * 360)

        if self.joystick.get_axis(3):
            self.target_vel  =  self.target_vel + self.joystick.get_axis(3) * (-1.0)
    
        self.target_vel = min(max( self.target_vel, 0), 200)
        self.get_logger().info("target_vel:{} steering: {} steering_mode: {}".format(self.target_vel,
                                                                   self.target_angle,
                                                                   self.manual_steering
                                                                   ))
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

        # 自動ステアリングモードではステアリング値を取得
        if not (self.manual_steering):
            self.target_steering = msg.lateral.steering_tire_angle
            self.target_angle = (self.target_steering * 360.0 * self.wheel_base)
        else:
            msg.lateral.steering_tire_angle = self.target_steering

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