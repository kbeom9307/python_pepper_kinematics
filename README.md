# pepper_kinematics

## Description 

Originally, Pepper does not provide any inverse kinematics function. This provides simple inverse kinematics funciton.

## Example

    """
    Move work_pose first, then move 5 centimeters toward left (y axis positive side).
    """

    import time
    import numpy as np
    import naoqi as n
    import sys 
    import python_pepper_kinematics.pepper_kinematics as pk

    nao_ip = "192.168.0.38"
    port = 9559

    m = n.ALProxy("ALMotion", nao_ip, port)
    m.setAngles(pk.left_arm_tags, pk.left_arm_work_pose, 0.05) #friction speed

    time.sleep(1.0)

    current_angles = []
    for i in range(len(pk.left_arm_tags)): # ALMotion.getanlges arg - str 로 변환
        current_angles.append(m.getAngles(pk.left_arm_tags[i],0))

    current_angles = [i[0] for i in current_angles] # list 벗겨내고
    current_position, current_orientation = pk.left_arm_get_position(current_angles)

    target_position = current_position
    target_position[1] = target_position[1] + 0.05 # 5 cm toward left
    target_orientation = current_orientation # This is not supported yet

    target_angles = pk.left_arm_set_position(current_angles, target_position, target_orientation)

    #m.setAngles(pk.left_arm_tags, list(np.round(target_angles, decimals=2)), 0.005)
    m.setAngles(pk.left_arm_tags, list(target_angles), 0.05)


## Copyright
* author: Yuki Suga
* copyright: Yuki Suga @ ssr.tokyo
* license: GPLv3

