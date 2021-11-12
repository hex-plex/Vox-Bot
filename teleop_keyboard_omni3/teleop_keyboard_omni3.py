#!/usr/bin/env python

from __future__ import print_function

import rospy

from std_msgs.msg import Float64

import sys, select, termios, tty

msg = """
Reading from the keyboard !
---------------------------
Moving around:
   u    i    o
   j    k    l
   m    ,    .

For Holonomic mode (strafing), hold down the shift key:
---------------------------
   U    I    O
   J    K    L
   M    <    >


anything else : stop

q/z : increase/decrease max speeds by 10%

CTRL-C to quit
"""

moveBindings = {
        'i':(1,0,0),
        'o':(0,0,1),
        'j':(0,-1,0),
        'l':(0,1,0),
        'u':(0,0,-1),
        ',':(-1,0,0),
        '.':(0,1,-1),
        'm':(1,-1,0),  
        'O':(-1,1,0),
        'I':(-1,0,1),
        'J':(1,-2,1),
        'L':(-1,2,-1),
        'U':(0,-1,1),
        '<':(1,0,-1),
        '>':(0,1,-1),
        'M':(1,-1,0),  
    }

speedBindings={
        'q':(1.1,1.1),
        'z':(.9,.9),
    }

def getKey():
    tty.setraw(sys.stdin.fileno())
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def vels(speed):
    return "currently:\tspeed %s " % (speed)

if __name__=="__main__":
    settings = termios.tcgetattr(sys.stdin)

    rospy.init_node('vel_Publisher')
    pubf = rospy.Publisher('/vox/fwheel_joint_velocity_controller/command', Float64, queue_size=1)
    pubb = rospy.Publisher('/vox/bwheel_joint_velocity_controller/command', Float64, queue_size=1)
    pubr = rospy.Publisher('/vox/rwheel_joint_velocity_controller/command', Float64, queue_size=1)
    publ = rospy.Publisher('/vox/lwheel_joint_velocity_controller/command', Float64, queue_size=1)


    speed = 1.0
    x = 0
    y = 0
    z = 0
    status = 0

    try:
        print(msg)
        print(vels(speed))
        while(1):
            key = getKey()
            if key in moveBindings.keys():
                x = moveBindings[key][0]
                y = moveBindings[key][1]
                z = moveBindings[key][2]
            elif key in speedBindings.keys():
                speed = speed * speedBindings[key][0]
       
                print(vels(speed))
                if (status == 14):
                    print(msg)
                status = (status + 1) % 15
            else:
                x = 0
                y = 0
                z = 0
                th = 0
                if (key == '\x03'):
                    break

            velf = Float64()
            vell = Float64()
            velb = Float64()
            velr = Float64()

            velf = y*speed + z*speed
            vell = x*speed + z*speed
            velb = -y*speed + z*speed
            velr = -x*speed + z*speed

            pubf.publish(velf)
            publ.publish(vell)
            pubb.publish(velb)
            pubr.publish(velr)

    except Exception as e:
        print(e)

    finally:
        velf = Float64()
        vell = Float64()
        velb = Float64()
        velr = Float64()
	
        velf = 0.0
        vell = 0.0
        velb = 0.0
        velr = 0.0

        pubf.publish(velf)
        pubb.publish(velb)
        publ.publish(vell)
        pubr.publish(velr)

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
