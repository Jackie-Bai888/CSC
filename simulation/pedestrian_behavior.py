import random
import carla
import time

def set_pedestrian(npc_pedestrian, npc_initial_speed, npc_ego_yaw, npc_ini_wp, npc_end_wp):
    # 设置行人的朝向角度
    transform = npc_pedestrian.get_transform()
    pe_ro = carla.Rotation(0, transform.rotation.yaw + npc_ego_yaw, 0)
    transform.rotation = pe_ro
    npc_pedestrian.set_transform(transform)
    # 设置行人的速度
    control = npc_pedestrian.get_control()
    control.speed = npc_initial_speed
    control.direction = npc_end_wp.transform.location - npc_ini_wp.transform.location
    control.direction.z = 3
    npc_pedestrian.apply_control(control)

    return npc_pedestrian
