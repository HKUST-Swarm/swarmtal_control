#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import math
import time
import argparse
import numpy as np

import rclpy
from rclpy.node import Node

# swarmtal_msgs (ROS2 版本) 中的消息
from swarmtal_msgs.msg import DroneOnboardCommand

def main():
    parser = argparse.ArgumentParser(description='A simple command tool for sending DroneOnboardCommand.')
    parser.add_argument('command_type', 
                        metavar='command_type', 
                        choices=["takeoff", "landing", "emland", "flyto", "vel", "arm", "disarm", 
                                 "joy_control", "circle", "circle_yaw", "sweep", "csv"],
                        help="Type of command to send")
    parser.add_argument("-c","--center", nargs=3, type=float, help="center for circle", default=[0, 0, 1])
    parser.add_argument("-r","--radius", type=float, help="radius for circle", default=0.5)
    parser.add_argument("-t","--cycle", type=float, help="cycle for circle or sweep", default=30)
    parser.add_argument("--fmin", type=float, help="min freq for sweep", default=0.1)
    parser.add_argument("--fmax", type=float, help="max freq for sweep", default=5)
    parser.add_argument("--count", type=int, help="sweep count number", default=3)
    parser.add_argument("-x", "--axis", type=int, help="axis for sweep", default=0)
    parser.add_argument("-A", "--amp", type=float, help="amp for sweep", default=1.0)
    parser.add_argument("-p", "--path", type=str, help="Path for CSV file", default="")
    parser.add_argument("params", nargs="*", type=float, help="parameters for command")

    args = parser.parse_args()

    print(f"Will send command {args.command_type} with params {args.params}")

    rclpy.init()

    node = rclpy.create_node('cmded')
    node.get_logger().info("Initializing drone_cmd (ROS2) node...")

    # 创建 Publisher
    pub = node.create_publisher(DroneOnboardCommand, '/drone_commander/onboard_command', 10)
    node.get_logger().info("Publisher created on /drone_commander/onboard_command")

    # 等待至少一个订阅者连接
    while rclpy.ok():
        sub_count = pub.get_subscription_count()
        if sub_count > 0:
            node.get_logger().info(f"Detected {sub_count} subscriber(s), ready to send command.")
            break
        else:
            node.get_logger().info("Waiting for subscriber connection...")
            time.sleep(0.5)

    cmd = DroneOnboardCommand()

    def send_command(c: DroneOnboardCommand):
        """发布命令并打印简单信息."""
        pub.publish(c)
        node.get_logger().info(f"Sent command_type={c.command_type}, param1={c.param1}, param2={c.param2} ...")

    # 以 50Hz 循环 => 间隔 0.02s
    sleep_dt = 0.02

    # 根据 command_type 不同，填充并发送 DroneOnboardCommand
    if args.command_type == "takeoff":
        # param: height
        if len(args.params) < 1:
            node.get_logger().warn("No height specified, default to 1.0m")
            height = 1.0
        else:
            height = args.params[0]
        cmd.command_type = DroneOnboardCommand.CTRL_TAKEOF_COMMAND
        cmd.param1 = int(height*10000)
        cmd.param2 = 5000  # 0.5m/s

        send_command(cmd)

    elif args.command_type == "landing":
        cmd.command_type = DroneOnboardCommand.CTRL_LANDING_COMMAND
        cmd.param1 = 0
        cmd.param2 = 3000
        send_command(cmd)

    elif args.command_type == "emland":
        cmd.command_type = DroneOnboardCommand.CTRL_LANDING_COMMAND
        cmd.param1 = 1
        cmd.param2 = 10000
        send_command(cmd)

    elif args.command_type == "arm":
        cmd.command_type = DroneOnboardCommand.CTRL_ARM_COMMAND
        cmd.param1 = 1
        send_command(cmd)

    elif args.command_type == "disarm":
        cmd.command_type = DroneOnboardCommand.CTRL_ARM_COMMAND
        cmd.param1 = 0
        send_command(cmd)

    elif args.command_type == "flyto":
        cmd.command_type = DroneOnboardCommand.CTRL_POS_COMMAND
        if len(args.params) < 3:
            node.get_logger().error("Must give X Y Z when using flyto")
            node.destroy_node()
            rclpy.shutdown()
            sys.exit(-1)
        else:
            cmd.param1 = int(args.params[0] * 10000)
            cmd.param2 = int(args.params[1] * 10000)
            cmd.param3 = int(args.params[2] * 10000)
            if len(args.params) == 4:
                cmd.param4 = int(args.params[3] * 10000)
            else:
                cmd.param4 = 666666  # MAGIC
            cmd.param5 = 0
            cmd.param6 = 0
            cmd.param7 = 0
            cmd.param8 = 0

        # 连续发送，直到 Ctrl+C or 节点关闭
        try:
            while rclpy.ok():
                send_command(cmd)
                time.sleep(sleep_dt)
        except KeyboardInterrupt:
            pass

    elif args.command_type == "vel":
        cmd.command_type = DroneOnboardCommand.CTRL_VEL_COMMAND
        if len(args.params) < 3:
            node.get_logger().error("Must give VX VY VZ when using vel")
            node.destroy_node()
            rclpy.shutdown()
            sys.exit(-1)
        else:
            cmd.param1 = int(args.params[0]*10000)
            cmd.param2 = int(args.params[1]*10000)
            cmd.param3 = int(args.params[2]*10000)

            if len(args.params) == 4:
                cmd.param4 = int(args.params[3]*10000)
            else:
                cmd.param4 = 666666

            cmd.param5 = 0
            cmd.param6 = 0
            cmd.param7 = 0
            cmd.param8 = 0

        try:
            while rclpy.ok():
                send_command(cmd)
                time.sleep(sleep_dt)
        except KeyboardInterrupt:
            pass

    elif args.command_type in ("circle", "circle_yaw"):
        cmd.command_type = DroneOnboardCommand.CTRL_POS_COMMAND
        node.get_logger().info(f"Will draw circle at center={args.center} r={args.radius}, cycle={args.cycle}")

        ox, oy, oz = args.center
        r = args.radius
        T = args.cycle
        cmd.param4 = 666666

        t = 0.0
        yaw = 666666.0
        try:
            while rclpy.ok():
                x = ox + math.sin(t*math.pi*2/T)*r
                y = oy + math.cos(t*math.pi*2/T)*r
                vx = math.cos(t*math.pi*2/T) * r * math.pi*2/T
                vy = -math.sin(t*math.pi*2/T) * r * math.pi*2/T
                if args.command_type == "circle_yaw":
                    yaw = t*math.pi*2/T

                ax = - math.sin(t*math.pi*2/T) * r * (math.pi*2/T)**2
                ay = - math.cos(t*math.pi*2/T) * r * (math.pi*2/T)**2

                cmd.param1 = int(x*10000)
                cmd.param2 = int(y*10000)
                cmd.param3 = int(oz*10000)
                if args.command_type == "circle_yaw":
                    cmd.param4 = int(yaw*10000)
                cmd.param5 = int(vx*10000)
                cmd.param6 = int(vy*10000)
                cmd.param7 = 0
                cmd.param8 = int(ax*10000)
                cmd.param9 = int(ay*10000)

                node.get_logger().info(
                    f"{t:.2f}s => xyz({x:.2f},{y:.2f},{oz:.2f}), yaw={yaw:.2f}, ff=({vx:.2f},{vy:.2f}), acc=({ax:.2f},{ay:.2f})"
                )
                send_command(cmd)

                t += sleep_dt
                time.sleep(sleep_dt)
        except KeyboardInterrupt:
            pass

    elif args.command_type == "csv":
        cmd.command_type = DroneOnboardCommand.CTRL_POS_COMMAND
        cmd.param4 = 666666

        if args.path == "":
            node.get_logger().info("No CSV path specified, exit.")
            node.destroy_node()
            rclpy.shutdown()
            sys.exit(-1)
        else:
            csv_path = args.path
            csv_data = np.genfromtxt(csv_path, delimiter=',')
            node.get_logger().info(f"CSV loaded, length={len(csv_data)}, total time={len(csv_data)/50:.2f}s")

        t = 0.0
        tick = 0
        try:
            while rclpy.ok():
                if tick >= len(csv_data):
                    node.get_logger().info("Reached end of CSV data, stop sending.")
                    break

                x  = csv_data[tick,0]
                y  = csv_data[tick,1]
                z  = csv_data[tick,2]
                vx = csv_data[tick,3]
                vy = csv_data[tick,4]

                cmd.param1 = int(x*10000)
                cmd.param2 = int(y*10000)
                cmd.param3 = int(z*10000)
                cmd.param5 = int(vx*10000)
                cmd.param6 = int(vy*10000)
                cmd.param7 = 0

                # 如果想减少控制台频繁输出，可以调节打印频率
                node.get_logger().info(f"{t:.2f}s => xyz({x:.2f},{y:.2f},{z:.2f}), ff=({vx:.2f},{vy:.2f})")
                send_command(cmd)

                t    += sleep_dt
                tick += 1
                time.sleep(sleep_dt)
        except KeyboardInterrupt:
            pass

    elif args.command_type == "sweep":
        cmd.command_type = DroneOnboardCommand.CTRL_POS_COMMAND
        cmd.param4 = 666666

        node.get_logger().info(f"Will sweep axis={args.axis} @ origin={args.center}, amp={args.amp}, T={args.cycle}, f={args.fmin}~{args.fmax}, count={args.count}")
        
        def generate_sweep_signal_base_func(T, omgmin=0.3, omgmax=12.0, c1=4.0, c2=0.0187):
            """仿原脚本: 生成一个随时间变化的函数, 实现 sweep 效果."""
            return lambda _t: math.sin(
                _t * omgmin + (omgmax - omgmin) * c2 * (T / c1 * (math.exp(c1 * _t / T) - 1) - _t)
            )

        func = generate_sweep_signal_base_func(
            args.cycle,
            omgmin=args.fmin*2*math.pi,
            omgmax=args.fmax*2*math.pi
        )

        t = 0.0
        count = 0
        try:
            while rclpy.ok() and count < args.count:
                x0, y0, z0 = args.center
                vx = 0.0
                vy = 0.0
                vz = 0.0
                x  = x0
                y  = y0
                z  = z0

                val = func(t)*args.amp
                if args.axis == 0:
                    vx = val
                    x  = x0 + val
                elif args.axis == 1:
                    vy = val
                    y  = y0 + val
                elif args.axis == 2:
                    vz = val
                    z  = z0 + val

                cmd.param1 = int(x*10000)
                cmd.param2 = int(y*10000)
                cmd.param3 = int(z*10000)
                cmd.param5 = int(vx*10000)
                cmd.param6 = int(vy*10000)
                cmd.param7 = int(vz*10000)

                node.get_logger().info(f"[{count}:{t:.2f}s] Sweeping => xyz({x:.2f},{y:.2f},{z:.2f}), ff=({vx:.2f},{vy:.2f},{vz:.2f})")
                send_command(cmd)

                time.sleep(sleep_dt)
                t += sleep_dt
                if t > args.cycle:
                    count += 1
                    t = 0.0
                    node.get_logger().info(f"Finish sweep {count}/{args.count}, resetting time.")
            node.get_logger().info("Sweep done or reached max count.")
        except KeyboardInterrupt:
            pass

    else:
        node.get_logger().error(f"Unknown command_type: {args.command_type}")

    node.get_logger().info("Done sending command. Shutting down...")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
