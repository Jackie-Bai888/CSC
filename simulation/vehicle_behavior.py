import time
import math
import carla
import random
import numpy as np

def create_ego(world, ego_ini_wp):
    # 获取所有车辆的列表
    vehicle_list = world.get_actors().filter('vehicle.*')
    if len(vehicle_list) != 0:
        ego_vehicle = vehicle_list[0]
        # ego_vehicle.set_location(ego_ini_wp.transform.location)
        tf = ego_ini_wp.transform
        # tf.rotation.yaw = 0
        ego_vehicle.set_transform(tf)
        return ego_vehicle

    # # 获取ego车辆
    # for ve in vehicle_list:
    #     if ve.attributes['role_name'] == 'ego_vehicle':
    #         ego_vehicle = ve
    # ego_vehicle.set_autopilot(True)
    # ego_bp = world.get_blueprint_library().find('vehicle.lincoln.mkz_2020')
    # ego_bp.set_attribute('role_name', 'hero')
    # ego_vehicle = world.spawn_actor(ego_bp, start_points)

def get_ego(world):
    # 获取所有车辆的列表
    vehicle_list = world.get_actors().filter('vehicle.*')
    # 删除npc车辆
    for actor in vehicle_list:
        if "autopilot" in actor.attributes['role_name']:  # 检查是否为NPC车辆
            actor.destroy()
    # 获取所有车辆的列表
    vehicle_list = world.get_actors().filter('vehicle.*')
    if len(vehicle_list) != 0:
        ego_vehicle = vehicle_list[0]
    return ego_vehicle

def get_ego_waypoint(world, ego_vehicle, carla_map, start_points):


    # 获取当前车辆的位置
    ego_location = ego_vehicle.get_location()
    # 获取车辆当前所在的道路点
    ego_waypoint = carla_map.get_waypoint(ego_location)

    # # 设置终点位置
    # destination = carla.Location(x=100, y=200, z=0)  # 替换为你想要的终点坐标
    # end_waypoint = carla_map.get_waypoint(destination)
    # route = ego_waypoint.compute_shortest_path(end_waypoint)
    # for waypoint in route:
    #     ego_vehicle.set_transform(waypoint.transform)
    #     time.sleep(0.1)
    # print("Reached destination")

    return ego_waypoint

def generate_npc(world, carla_map, ego_waypoint):
    # 创建一个NPC车辆
    blueprint_library = world.get_blueprint_library()
    npc_blueprint = blueprint_library.filter('vehicle.*')[0]
    #pitch ( float - degrees ) Y-axis rotation angle. yaw ( float - degrees ) Z-axis rotation angle. roll ( float - degrees ) X-axis rotation angle.
    # 获取左侧车道的Waypoint对象
    # left_lane_waypoint = ego_waypoint.get_left_lane()
    # 获取右侧车道的Waypoint对象
    # right_lane_waypoint = ego_waypoint.get_right_lane()
    # 获取车辆可以运行的下一个路点
    ego_next_waypoints = list(ego_waypoint.next(10))
    npc_vehicle = world.spawn_actor(npc_blueprint, ego_next_waypoints[0].transform)
    # 可以使用waypoint.lane_type来判断是否是驾驶车道，lane_type的值包括driving(行车道)，sidewalk(人行道)，shoulder(路肩)，parking(停车位)，bikelane（自行车道），restricted(受限制的车道)
    # npc_vehicle = world.spawn_actor(npc_blueprint, left_lane_waypoint.transform)
    time.sleep(1)
    # 获取车辆当前位置
    npc_location = npc_vehicle.get_location()
    # 获取车辆当前所在的道路点
    npc_waypoint = carla_map.get_waypoint(npc_location)
    # 获取车辆可以运行的下一个路点
    npc_next_waypoints1 = list(npc_waypoint.next(5))[0]  # 获取车辆前方一定距离内的路点
    # 获取车辆可以运行的下一个路点
    npc_next_waypoints2 = list(npc_waypoint.next(10))[0]  # 获取车辆前方一定距离内的路点
    # 设置车辆的自动驾驶模式
    npc_vehicle.set_autopilot(False)

    # 计算两个waypoints之间的距离和方向
    direction = npc_next_waypoints1.transform.location - npc_waypoint.transform.location
    distance = direction.length()
    direction = direction / distance  # 归一化方向向量
    velocity = 5.0  # 设置匀速行驶的速度，单位为m/s

    # 计算匀速运动所需的时间
    time_to_reach = distance / velocity

    # 设置车辆的线速度和角速度
    npc_vehicle.set_target_velocity(carla.Vector3D(velocity * direction.x, velocity * direction.y, 0))
    npc_vehicle.set_target_angular_velocity(carla.Vector3D(0, 0, 0))

    # 等待车辆行驶到目标waypoint
    time.sleep(time_to_reach)

    # 停止车辆
    npc_vehicle.set_target_velocity(carla.Vector3D(0, 0, 0))
    npc_vehicle.set_target_angular_velocity(carla.Vector3D(0, 0, 0))

    # 等待一段时间
    time.sleep(1)

    # 销毁车辆
    npc_vehicle.destroy()

def calculate_direction(ego_ve, npc_ve, map):
    # find subject vehicle and preceding vehicle by calculating the angle between ego_ve and npc_ve
    npc_loc, ego_loc = ego_ve.get_location(), npc_ve.get_location()
    ego_wp = map.get_waypoint(ego_loc)
    next_wp = ego_wp.next(10)
    a_x = next_wp.transform.location.x - ego_ve.transform.location.x
    a_y = next_wp.transform.location.y - ego_ve.transform.location.y
    b_x = npc_loc.x - ego_loc.x
    b_y = npc_loc.y - ego_loc.y
    a_len = np.sqrt(a_x ** 2 + a_y ** 2)
    b_len = np.sqrt(b_x ** 2 + b_y ** 2)
    cos_theta = (a_x * b_x + a_y * b_y) / (a_len * b_len)
    if cos_theta >= 0:
        return npc_ve, ego_ve
    else:
        return ego_ve, npc_ve


# 获取车辆的油门
def get_throttle(vehicle):
    current_control = vehicle.get_control()
    throttle_value = current_control.throttle
    return throttle_value

# 获取车辆的刹车
def get_brake(vehicle):
    current_control = vehicle.get_control()
    brake_value = current_control.brake
    return brake_value

# 获取车辆的车头角度
def get_heading(vehicle):
    transfrom = vehicle.get_transform()
    rot = transfrom.rotation
    return rot

def set_velocity(vehicle, speed):
    # speed 单位为km/h
    speed_ms = speed * 1000 / 3600  # 结果为 m/s
    # vehicle_transform = vehicle.get_transform()
    # vehicle_forward = vehicle_transform.get_forward_vector()
    speed_vector = carla.Vector3D(speed_ms,
                                  0,
                                  0.0)
    # vehicle.set_target_velocity(speed_vector)
    vehicle.enable_constant_velocity(speed_vector)

def set_throttle(vehicle, throttle):
    # 取消NPC车辆的自动驾驶模式
    vehicle.set_autopilot(False)
    control = vehicle.get_control()
    control.throttle = throttle  # 范围为0-1
    control.brake = 0
    vehicle.apply_control(control)

def set_brake(vehicle, brake_num):
    # 取消NPC车辆的自动驾驶模式
    vehicle.set_autopilot(False)
    vehicle.apply_control(carla.VehicleControl(throttle=0, brake=brake_num))

def calculate_mttc(ego_ve, npc_ve):
    # this function is to calculate mttc
    p_ve, s_ve = calculate_direction(ego_ve, npc_ve)
    p_ve, p_acc = p_ve.get_velocity().length(), p_ve.get_acceleration().length()
    s_ve, s_acc = s_ve.get_velocity().length(), s_ve.get_acceleration().length()

    # 通过next来判断
    diff_ve = s_ve - p_ve
    diff_acc = s_acc - p_acc
    npc_location = npc_ve.get_location()
    npc_location.z = 0
    ego_location = ego_ve.get_location()
    ego_location.z = 0
    distance = npc_location.distance(ego_location)
    t_1 = (-diff_ve - np.sqrt(diff_ve**2+2*diff_acc*distance))/ diff_acc
    t_2 = (-diff_ve + np.sqrt(diff_ve**2+2*diff_acc*distance))/ diff_acc
    if diff_acc != 0:
        if t_1 > 0 and t_2 > 0:
            if t_1 >= t_2:
                return t_2
            else:
                return t_1
        elif t_1 > 0 and t_2 <= 0:
            return t_1
        elif t_1 <= 0 and t_2 > 0:
            return t_2
    else:
        if diff_ve >0:
            return distance/diff_ve


def fit_fuction(ego, npc):
    # canculate the distance between ego vehicle and npc vehicle
    # ego_location = ego.get_location()
    # npc_location = npc.get_location()
    # print('npc_location', npc_location)
    # distance = ego_location.distance(npc_location)
    # return distance
    # 计算两个物体对应的bounding box顶点的最小距离
    bbox_ego = ego.bounding_box
    bbox_npc = npc.bounding_box
    transform_a = ego.get_transform()
    transform_b = npc.get_transform()
    vertice_ego = bbox_ego.get_world_vertices(transform_a)
    vertice_npc = bbox_npc.get_world_vertices(transform_b)

    # 计算两组顶点之间所有可能对的最小距离
    min_distance = float('inf')
    for vertex_a in vertice_ego:
        for vertex_b in vertice_npc:
            distance = vertex_a.distance(vertex_b)
            if distance < min_distance:
                min_distance = distance
    round_min_distance = round(min_distance, 2)
    return round_min_distance
def create_npc(world, match_result):
    # 获取所有车辆的列表 have_sensor表示是否第一次创建ego，如果是第一次创建，则需要加sensor
    vehicle_list = world.get_actors().filter('vehicle.*')
    # # 删除npc车辆
    for actor in vehicle_list:
        if "autopilot" in actor.attributes['role_name']:  # 检查是否为NPC车辆
            actor.destroy()
    pedestrian_list = world.get_actors().filter('walker.pedestrian.*')
    # 删除行人
    for actor in pedestrian_list:
        actor.destroy()
    # 创建一个NPC车辆
    # npc_postion_ego_l_r 是指npc车在ego车左边还是右边
    blueprint_library = world.get_blueprint_library()
    #'vehicle.*truck" 'walker.pedestrian.*'
    object_type, behavior, position, _ = match_result
    if object_type == 'car':
        npc_blueprint = blueprint_library.filter('vehicle.*')[0]
    elif object_type == 'truck':
        npc_blueprint = blueprint_library.filter('vehicle.*truck')[0]
    else:
        npc_blueprint = blueprint_library.filter('walker.pedestrian.*')[0]
    map = world.get_map()
    spawn_points = map.get_spawn_points()
    npc_position = random.choice(spawn_points)
    npc_actor = world.spawn_actor(npc_blueprint, npc_position)
    return npc_actor

def mutate_npc(npc_vehicle, npc_initial_speed, npc_throttle):
    # # 设置NPC车辆的初始速度
    set_velocity(npc_vehicle, npc_initial_speed)
    # 设置NPC车辆的油门
    set_throttle(npc_vehicle, npc_throttle)
    return npc_vehicle

def set_autopilot(vehicle, is_auto):
    # 设置车辆的自动驾驶模式
    vehicle.set_autopilot(is_auto)
    time.sleep(1)

def set_insert(client, npc_vehicle, direction):
    tm = client.get_trafficmanager(8000)
    lane_change_dir = None
    if 'left' in direction or 'right' in direction:
        lane_change_dir = True
    # time.sleep(sleep_time)
    tm.force_lane_change(npc_vehicle, lane_change_dir)


def set_accelerate(npc_vehicle, ego_vehicle):
    # 如果后方npc车辆大于前方ego车辆，则不需要加速。否则，需要加速
    ego_vel = ego_vehicle.get_velocity()
    ego_vel_sqa = math.sqrt(ego_vel.x**2 + ego_vel.y**2 + ego_vel.z**2)
    npc_vel = npc_vehicle.get_velocity()
    npc_vel_sqa = math.sqrt(npc_vel.x ** 2 + npc_vel.y ** 2 + npc_vel.z ** 2)
    if npc_vel_sqa <= ego_vel_sqa:
        npc_vehicle.apply_control(carla.VehicleControl(throttle=1, brake=0))

def set_come(npc_vehicle):
    print('----进入come函数----')
    # npc_vehicle.set_wheel_steer_direction(carla.VehicleWheelLocation.FL_Wheel, 180)
    # npc_vehicle.set_wheel_steer_direction(carla.VehicleWheelLocation.FR_Wheel, 180)
    # 获取当前的变换信息
    current_transform = npc_vehicle.get_transform()

    # 计算新的变换信息，旋转180度
    new_transform = carla.Transform(
        location=current_transform.location,
        rotation=carla.Rotation(
            pitch=current_transform.rotation.pitch,
            yaw=current_transform.rotation.yaw + 180,  # 旋转180度
            roll=current_transform.rotation.roll
        )
    )

    # 应用新的变换
    npc_vehicle.set_transform(new_transform)


def set_reverse(npc_vehicle):
    # 获取车辆的控制对象
    control = npc_vehicle.get_control()

    # 设置控制参数为倒车
    control.reverse = True
    # control.throttle = 1.0  # 设置油门为最大值

    # 应用控制参数到车辆上
    npc_vehicle.apply_control(control)


def set_turn(npc_vehicle, direction, sleep_time):
    # 获取车辆的控制对象
    control = npc_vehicle.get_control()
    # 设置车辆左转
    if direction == 'left':
        control.steer = -1.0  # 左转方向盘角度为-1（负值表示左转）
    else:
        control.steer = 1.0  # 右转方向盘角度为1（正值表示右转）
    control.throttle = 0.5  # 油门踏板位置为0.5（范围从0到1）

    # 应用控制到车辆
    npc_vehicle.apply_control(control)
    # while True:
    #     current_location = npc_vehicle.get_location()
    #     distance = destination.transform.location.distance(current_location)
    #     print(distance)
    #     if distance < 2:
    #         break
    #     else:
    #         time.sleep(sleep_time)
    time.sleep(sleep_time)
    # 销毁车辆
    npc_vehicle.destroy()

def set_cross(client, npc_vehicle, start_wp, end_wp, distance,sleep_time):
    # 获取车辆的控制对象
    # control = npc_vehicle.get_control()

    # # 设置控制参数为倒车
    # control.reverse = True
    # control.throttle = 1.0  # 设置油门为最大值

    traffic_manager = client.get_trafficmanager()
    # traffic_manager.set_synchronous_mode(True)

    # npc_vehicle.apply_control(control)
    routing_ls = []
    destination = end_wp.next(ego_driving_dis)[0]
    while True:
        current_location = start_wp.transform.location
        distance = end_wp.transform.location.distance(current_location)
        print(distance)
        if distance < 1:
            routing_ls.append(end_wp.transform.location)
            break
        else:
            routing_ls.append(current_location)
            start_wp = start_wp.next(1)[0]
        # 销毁车辆
    npc_vehicle.set_autopilot(True)  # Give TM control over vehicle

    # Set parameters of TM vehicle control, we don't want lane changes
    # traffic_manager.ignore_lights_percentage(npc_vehicle, 100)
    traffic_manager.update_vehicle_lights(npc_vehicle, True)
    # traffic_manager.random_left_lanechange_percentage(npc_vehicle, 0)
    # traffic_manager.random_right_lanechange_percentage(npc_vehicle, 0)
    traffic_manager.auto_lane_change(npc_vehicle, False)
    traffic_manager.set_path(npc_vehicle, routing_ls)
    # npc_vehicle.set_autopilot(True)  # Give TM control over vehicle
    # time.sleep(5)
    # while True:
    #     current_location = npc_vehicle.get_location()
    #     distance = end_wp.transform.location.distance(current_location)
    #     print(distance)
    #     if distance < 2:
    #         break
    #     else:
    #         time.sleep(sleep_time)
    # # 销毁车辆
    # npc_vehicle.destroy()

if __name__ == '__main__':
    import simulator as sim
    import dreamview as dm
    import weather
    client, world, carla_map = sim.inital_carla()
    ego = get_ego(world)
    '''
    while True:
        ve = get_velocity(ego)
        acc = get_acceleration(ego)
        rotation = get_heading(ego)
        print(ve, ve.length(), acc, rotation.get_forward_vector(),rotation.yaw)
    '''
    throt = get_throttle(ego)
    brake = get_brake(ego)
    print(throt, brake)
    # set_velocity(ego, 100)
    i = 0
    set_brake(ego, 0)
    while i < 2:
        set_throttle(ego, 1)
        throt = get_throttle(ego)
        brake = get_brake(ego)
        print(throt, brake)
        i += 1
    set_throttle(ego, 0)
    set_brake(ego, 1)
    throt = get_throttle(ego)
    brake = get_brake(ego)
    print(throt, brake)

