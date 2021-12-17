#!/usr/bin/env python

import rospy
import os
import rospkg
import subprocess
import roslaunch

p = []
tf_p = []
def spawn_bot(name, x, y, z, yaw, pub_tf=False, fx=None, fy=None, fY=None, fname=None):
    global p, tf_p
    package = 'vox_gazebo'
    include_name = "robot_load_gazebo.launch"
    command = "roslaunch {0} {1} namesp:={2} x:={3} y:={4} z:={5} yaw:={6}".format(package, 
                                                                                   include_name,
                                                                                   name,
                                                                                   x,
                                                                                   y,
                                                                                   z,
                                                                                   yaw)
    if pub_tf:                                                                                   
        tf_command = "rosrun tf static_transform_publisher {0} {1} 0 0 0 {2} {3}/map {4}/map 100".format(x-fx, y-fy, yaw-fY, fname, name)                                                                              
        tf_p_in = subprocess.Popen(tf_command, shell=True)
        tf_p.append(tf_p_in)

    p_in = subprocess.Popen(command, shell=True)
    p.append(p_in)
    state = p_in.poll()
    if state is None:
        rospy.loginfo("process is running fine")
    elif state < 0:
        rospy.loginfo("Process terminated with error")
    elif state > 0:
        rospy.loginfo("Process terminated without error")

def main():
    rospy.init_node("util_helper")
    n_b = rospy.get_param("num_bot")
    fx = 0
    fy = 0
    fY = 0
    fname="world"
    for i in range(1, int(n_b)+1):
                   
        bot_x = rospy.get_param("/vox_{0}/map_merge/init_pose_x".format(i))
        bot_y = rospy.get_param("/vox_{0}/map_merge/init_pose_y".format(i))
        bot_z = rospy.get_param("/vox_{0}/map_merge/init_pose_z".format(i))
        bot_Y = rospy.get_param("/vox_{0}/map_merge/init_pose_yaw".format(i))
        bot_name = "vox_{0}".format(i)
        spawn_bot(bot_name, bot_x, bot_y, bot_z, bot_Y, True, fx, fy, fY, fname)
        if i == 1:
            fx = bot_x
            fy = bot_y
            fY = bot_Y
            fname = bot_name

    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        for tf_in in tf_p:
            tf_in.kill()
        for p_in in p:
            p_in.kill()