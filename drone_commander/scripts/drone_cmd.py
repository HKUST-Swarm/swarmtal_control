#!/usr/bin/env python2
from __future__ import print_function

import argparse
import rospy
from swarmtal_msgs.msg import drone_onboard_command
import sys
import math
import numpy as np

def send(cmd, args, pub):
    # pub = rospy.Publisher("/drone_commander/onboard_command", drone_onboard_command, queue_size=1)
    pub.publish(cmd)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='A easy command tool for sending command to swarm drone')
    parser.add_argument('command_type', metavar='command_type', choices=
        ["takeoff", "landing", "emland", "flyto","vel", "arm", "disarm", "joy_control", "circle", "circle_yaw", "sweep", "csv"], help="Type of command to send")
    parser.add_argument("-c","--center", nargs=3, type=float, help="center for circle", default=[0, 0, 1])
    parser.add_argument("-r","--radius", type=float, help="radius for circle", default=0.5)
    parser.add_argument("-t","--cycle", type=float, help="cycle for circle or for sweep a cycle", default=30)
    parser.add_argument("--fmin", type=float, help="min freq for sweep", default=0.1)
    parser.add_argument("--fmax", type=float, help="max freq for sweep", default=5)
    parser.add_argument("--count", type=int, help="sweep count number for sweep", default=3)
    parser.add_argument("-x", "--axis", type=int, help="axis for sweep", default=0)
    parser.add_argument("-A", "--amp", type=float, help="amp for sweep", default=1.0)
    parser.add_argument("-p", "--path", type=str, help="Path", default="")

    parser.add_argument("params",nargs="*", type=float, help="parameters for command")
    args = parser.parse_args()

    print("Will send command {} with params {}".format(args.command_type, args.params))

    try:
        rospy.get_master().getPid()
    except:
        print("roscore is offline, exit")
        sys.exit(-1)

    rospy.init_node('cmded', anonymous=True)


    print("Sending to onboard")
    pub = rospy.Publisher("/drone_commander/onboard_command", drone_onboard_command, queue_size=1)


    rate = rospy.Rate(50)  # 20hz

    while not rospy.is_shutdown():
        connections = pub.get_num_connections()
        print("Wait for pub")
        if connections > 0:
            break
        rate.sleep()
    cmd = drone_onboard_command()

    if args.command_type == "takeoff":
        if len(args.params) < 1:
            rospy.logwarn("No height specs, will fly to default 1.0m")
            height = 1.0
        else:
            height = args.params[0]
        cmd.command_type = drone_onboard_command.CTRL_TAKEOF_COMMAND
        cmd.param1 = int(height*10000)
        send(cmd, args, pub)

    elif args.command_type == "landing":
        cmd.command_type = drone_onboard_command.CTRL_LANDING_COMMAND
        cmd.param1 = 0
        cmd.param2 = 3000
        send(cmd, args, pub)

    elif args.command_type == "emland":
        cmd.command_type = drone_onboard_command.CTRL_LANDING_COMMAND
        cmd.param1 = 1
        cmd.param2 = 10000
        send(cmd, args, pub)

    elif args.command_type == "arm":
        cmd.command_type = drone_onboard_command.CTRL_ARM_COMMAND
        cmd.param1 = 1
        send(cmd, args, pub)


    elif args.command_type == "disarm":
        cmd.command_type = drone_onboard_command.CTRL_ARM_COMMAND
        cmd.param1 = 0
        send(cmd, args, pub)


    elif args.command_type == "flyto":
        cmd.command_type = drone_onboard_command.CTRL_POS_COMMAND
        if len(args.params) < 3:
            rospy.logerr("Must give xyz when using flyto")
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


        while not rospy.is_shutdown():
            try:
                send(cmd, args, pub)
                rate.sleep()
            except KeyboardInterrupt:
                exit(0)
    
    elif args.command_type == "vel":
        cmd.command_type = drone_onboard_command.CTRL_VEL_COMMAND
        if len(args.params) < 3:
            rospy.logerr("Must give xyz when using flyto")
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
        while not rospy.is_shutdown():
            try:
                send(cmd, args, pub)
                rate.sleep()
            except KeyboardInterrupt:
                exit(0)

    elif args.command_type == "circle" or args.command_type == "circle_yaw":
        cmd.command_type = drone_onboard_command.CTRL_POS_COMMAND
        print("Will draw circle @ origin {} {} {}, r {} T {}".format(
            args.center[0],
            args.center[1],
            args.center[2],
            args.radius,
            args.cycle
        ))

        ox = args.center[0]
        oy = args.center[1]
        oz = args.center[2]
        r = args.radius
        T = args.cycle


        cmd.param1 = 0
        cmd.param2 = 0
        cmd.param3 = 0
        cmd.param4 = 666666
        cmd.param5 = 0
        cmd.param6 = 0
        cmd.param7 = 0
        cmd.param8 = 0
        cmd.param9 = 0
        cmd.param10 = 0

        t = 0
        yaw = 666666
        while not rospy.is_shutdown():
            try:
                x = ox + math.sin(t*math.pi*2/T)*r
                y = oy + math.cos(t*math.pi*2/T)*r
                vx = math.cos(t*math.pi*2/T) * r * math.pi*2/T
                vy = -math.sin(t*math.pi*2/T) * r * math.pi*2/T
                if args.command_type == "circle_yaw":
                    yaw = t*math.pi*2/T
                ax = - math.sin(t*math.pi*2/T) * r * math.pi*2/T * math.pi*2/T
                ay = - math.cos(t*math.pi*2/T) * r * math.pi*2/T * math.pi*2/T

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

                rospy.loginfo("{:3.2f} xyz {:3.2f} {:3.2f} {:3.2f} Y {:3.2f} ff {:3.2f} {:3.2f} {:3.2f} {:3.2f}".format(t, x, y, oz, yaw, vx, vy, ax, ay))
                send(cmd, args, pub)
                t = t + 0.02
                rate.sleep()

            except KeyboardInterrupt:
                exit(0)

    elif args.command_type == "csv":
        cmd.command_type = drone_onboard_command.CTRL_POS_COMMAND
        cmd.param4 = 666666
        if args.path=="":
            rospy.loginfo("No csv specs, exit")
            sys.exit(-1)
        else:
            csv_path = args.path
            csv_data = np.genfromtxt(csv_path, delimiter=',')
            rospy.loginfo("CSV duration {}s len {}".format(len(csv_data), len(csv_data)/50))

        cmd.param1 = 0
        cmd.param2 = 0
        cmd.param3 = 0
        cmd.param4 = 666666
        cmd.param5 = 0
        cmd.param6 = 0
        cmd.param7 = 0
        cmd.param8 = 0
        cmd.param9 = 0
        cmd.param10 = 0

        t = 0
        tick = 0
        while not rospy.is_shutdown():
            x = csv_data[tick,0]
            y = csv_data[tick,1]
            z = csv_data[tick,2]
            vx = csv_data[tick,3]
            vy = csv_data[tick,4]

            cmd.param1 = int(x*10000)
            cmd.param2 = int(y*10000)
            cmd.param3 = int(z*10000)
            cmd.param5 = int(vx*10000)
            cmd.param6 = int(vy*10000)
            cmd.param7 = 0


            rospy.loginfo_throttle(0.1, "{:3.2f} xyz {:3.2f} {:3.2f} {:3.2f} ff {:3.2f} {:3.2f}".format(t, x, y, z, vx, vy))
            send(cmd, args, pub)
            t = t + 0.02
            tick = tick + 1
            rate.sleep()

    elif args.command_type == "sweep":
        cmd.command_type = drone_onboard_command.CTRL_POS_COMMAND
        cmd.param4 = 666666

        print("Will sweep axis {} @ origin {} {} {}, amp {} T {} freq {}:{}/s by{} times".format(
            args.axis,
            args.center[0],
            args.center[1],
            args.center[2],
            args.amp,
            args.cycle,
            args.fmin,
            args.fmax,
            args.count
        ))

        def generate_sweep_signal_base_func(T, omgmin=0.3, omgmax=12, c1=4.0, c2=0.0187):
            return lambda t: math.sin(t * omgmin + (omgmax - omgmin) * c2 * (T / c1 * (math.exp(c1 * t / T) - 1) - t))
        func  = generate_sweep_signal_base_func(args.cycle, omgmin=args.fmin*6.28, omgmax=args.fmax*6.28)

        t = 0
        count = 0

        while not rospy.is_shutdown() and count < args.count:
            try:
                x0 = args.center[0]
                y0 = args.center[1]
                z0 = args.center[2]

                vx = 0
                vy = 0
                vz = 0

                x = x0
                y = y0
                z = z0

                if args.axis == 0:
                    vx = func(t) * args.amp
                    x = func(t) * args.amp + x0
                elif args.axis == 1:
                    vy = func(t) * args.amp
                    y = func(t) * args.amp + y0
                elif args.axis == 2:
                    vz = func(t) * args.amp
                    z = func(t) * args.amp + z0

                cmd.param1 = int(x*10000)
                cmd.param2 = int(y*10000)
                cmd.param3 = int(z*10000)
                cmd.param5 = int(vx*10000)
                cmd.param6 = int(vy*10000)
                cmd.param7 = int(vz*10000)

                print("[{}:{:3.2f}] Sweeping.... xyz {:3.2f} {:3.2f} {:3.2f} ff {:3.2f} {:3.2f} {:3.2f}".format(count, t, x, y, z, vx, vy, vz))
                send(cmd, args, pub)
                t = t + 0.02
                if t > args.cycle:
                    count = count + 1
                    t = 0
                rate.sleep()
                print("Finish sweep, stop")
            except KeyboardInterrupt:
                exit(0) 