import random
import time

from llm import llm
from simulation import simulator as sim
from simulation import dreamview as dm
from ui import ui
import PySimpleGUI as sg
from simulation import vehicle_behavior as vb
from simulation import pedestrian_behavior as pb
from simulation import weather as wt
import carla
import math
import csv
import json
scena_id = 0
old_crash_scena_id = 0
def extract_danger_behavior(danger_content):
    # 逆行 Retrograde motion
    obj_type = ['pedestrian', 'car', 'train'] #'truck',
    behavior = ['brake', 'cross', 'turn right', 'turn left', 'reverse', 'cut in', 'accelerate', 'come']
    position = ['left front', 'front', 'right front', 'left', 'right', 'left rear', 'rear', 'right rear']
    weather = ['sunny', 'cloudy', 'rainy', 'windy', 'foggy', 'night']

    # extract danger information
    danger_info = llm.extract_information(danger_content)
    danger_info = eval(danger_info)
    print('danger_info', danger_info)
    danger_subject, danger_behavior, danger_position, danger_weather = danger_info["subject of dangerous action"], danger_info["dangerous behavior"], danger_info["position of subject relative to ego vehicle"], danger_info["weather"]
    match_result = llm.find_matching_behavior(danger_subject, obj_type, danger_behavior, behavior, danger_position, position, danger_weather, weather)
    print('match_result', match_result)
    return match_result



def set_ego_routing(start, s_e_distance = 15):
    # start_location = ego_waypoint.transform.location
    start_location = start.transform.location
    # ego_next_waypoints = list(start_transform.next(10))[0]
    end_location = start.next(s_e_distance)[0].transform.location
    ws = dm.connnect_dreaview()
    dm.send_info(ws, start_location, end_location)

def set_npc_ve_behavior(npc_vehicle, npc_start_waypoint, npc_initial_speed, npc_throttle, npc_ego_yaw, match_result):
    #set npc vehicle type, location, velocity, throttle, row
    vb.set_autopilot(npc_vehicle, False)
    if match_result[1] == 'come':
        vb.set_come(npc_vehicle)
    if match_result[1] == 'reverse':
        npc_initial_speed = -npc_initial_speed
    npc_ve = vb.mutate_npc(npc_vehicle, npc_initial_speed, npc_throttle)
    # calculate_mttc(ego_vehicle, )
    # calculate_mttc(ego_vehicle, npc_ve)
    # set npc behavior
    if match_result[1] == 'brake':
        # 这里的npc_start_waypoint的值就是ego的初始位置，所以在设置npc初始位置时需要将坐标前移
        # vb.set_autopilot(npc_ve, True)
        sign = random.choice([-1, 1])
        steer_angle = sign * npc_ego_yaw
        npc_ve.apply_control(carla.VehicleControl(steer=steer_angle))
        # # 设置随眠时间是为了让npc先行驶一段时间后才刹车
        # time.sleep(2)
        vb.set_velocity(npc_ve,0)
    elif match_result[1] == 'cut in':
        if 'left' in match_result[2]:
            steer_angle = npc_ego_yaw
        elif 'right' in match_result[2]:
            steer_angle = -npc_ego_yaw
        npc_ve.apply_control(carla.VehicleControl(steer=steer_angle))
        # vb.set_insert(client, npc_ve, match_result[2])
    elif match_result[1] == 'accelerate':
        # 随机生成车辆轮胎转向角的方向
        # npc_destination_wp = npc_start_waypoint.next(distance)[0]
        vb.set_autopilot(npc_ve, False)
        sign = random.choice([-1, 1])
        steer_angle = sign * npc_ego_yaw
        npc_ve.apply_control(carla.VehicleControl(steer=steer_angle))
        vb.set_accelerate(npc_ve, ego_vehicle)
    # elif match_result[1] == 'cross':
    #     vb.set_autopilot(npc_ve, False)
    #     vb.set_cross(npc_ve, npc_start_waypoint, npd_end_waypoint, 1)
    # sim.set_behavior(npc_ve, match_result[1])
    return npc_ve

def set_npc_pe_behavior(npc_pedestrian, npc_ini_wp, npc_initial_speed, npc_ego_yaw, danger_analysis_result):
    if 'right' in danger_analysis_result[2]:
        npc_ego_yaw = -npc_ego_yaw
        npc_end_wp = npc_ini_wp.get_left_lane().get_left_lane()
    else:
        npc_end_wp = npc_ini_wp.get_right_lane().get_right_lane()
    npc_pedestrian = pb.set_pedestrian(npc_pedestrian, npc_initial_speed, npc_ego_yaw, npc_ini_wp, npc_end_wp)
    return npc_pedestrian

def mutate_ve_npc(npc_ini_wp_ls, ego_waypoint, driving_distance, npc_behavior_distance, npc_initial_speed, npc_throttle, npc_ego_yaw, danger_analysis_result):
    '''
    npc_behavior_distance  npc车辆发生行为的位置与ego车辆的距离
    npc_initial_speed    npc车辆的初始速度,单位为km/h，（0，200）
    npc_throttle     npc的初始油门值,范围为（0，1）
    npc_ego_yaw     npc车辆与ego车辆之间的初始角度，范围为（0,90）
    '''
    sample_num_ls = [0, 1, 2, 3, 4]
    # npc_behavir_distance要根据位置来确定改变的范围
    if npc_behavior_distance == 4:
        sample_num_ls.remove(0)
    if npc_initial_speed == 200:
        sample_num_ls.remove(1)
    if npc_throttle == 1:
        sample_num_ls.remove(2)
    if npc_ego_yaw == 1:
        sample_num_ls.remove(3)
    npc_waypoint = npc_ini_wp_ls[-1]
    #random.randint(0, 4)会包含0和4
    change_num = random.choice(sample_num_ls)#random.randint(0, 4)
    print(change_num)
    if change_num == 0:
        npc_behavior_distance -= 1  # 当在左边和右边时每次减0.1;当在左右前方和后方每次减少1
        if npc_behavior_distance <= 4:
            npc_behavior_distance = 4
    elif change_num == 1:
        npc_initial_speed += 10 # 每次加10
        if npc_initial_speed >= 200:
            npc_initial_speed = 200
    elif change_num == 2:
        npc_throttle += 0.1 #每次加0.1
        if npc_throttle >= 1:
            npc_throttle = 1
    elif change_num == 3:
        # 在这里将的yaw表示方向盘的角度，也就是前车轮的角度
        if 'left' in danger_analysis_result[2] or 'right' in danger_analysis_result[2]:
            npc_ego_yaw += 0.1  # npc车辆与ego车辆之间的初始角度，范围为（0,1）
            if npc_ego_yaw >= 1:
                npc_ego_yaw = 1
    else:
        while True:
            if danger_analysis_result[1] == 'cut in':
                judge_param = True
            else:
                judge_param = False
            position = danger_analysis_result[2]
            npc_waypoint = sim.get_ve_start_wp(ego_waypoint, position, driving_distance, judge_param)
            if npc_waypoint not in npc_ini_wp_ls:
                break
    return npc_waypoint, npc_behavior_distance, npc_initial_speed, npc_throttle, npc_ego_yaw

def mutate_pe_npc(npc_ini_wp_ls, ego_waypoint, driving_distance, npc_behavior_distance, npc_initial_speed, npc_ego_yaw, danger_analysis_result, carla_map):
    '''
    npc_behavior_distance  npc车辆发生行为的位置与ego车辆的距离
    npc_initial_speed    npc车辆的初始速度,单位为km/h，（0，200）
    npc_ego_yaw     npc车辆与ego车辆之间的初始角度，范围为（90,180）
    '''
    sample_num_ls = [0, 1, 2, 3]
    # 因为行人和ego车辆之间的距离可以比npc车辆和ego之间的距离小，所以最小值设置为0
    if npc_behavior_distance == 0:
        sample_num_ls.remove(0)
    if npc_initial_speed == 1:
        sample_num_ls.remove(1)
    if npc_ego_yaw == 180:
        sample_num_ls.remove(2)
    npc_waypoint = npc_ini_wp_ls[-1]
    #random.randint(0, 4)会包含0和4
    change_num = random.choice(sample_num_ls)
    print(change_num)
    if change_num == 0:
        npc_behavior_distance -= 1
        if npc_behavior_distance <= 0:
            npc_behavior_distance = 0
    elif change_num == 1:
        npc_initial_speed -= 0.1 # 每次减1
        if npc_initial_speed <= 0.5:
            npc_initial_speed = 0.5
    elif change_num == 2:
        npc_ego_yaw += 1  # npc行人与ego车辆之间的初始角度，范围为（90,180）
        if npc_ego_yaw >= 180:
            npc_ego_yaw = 180
    else:
        while True:
            position = danger_analysis_result[2]
            npc_waypoint = sim.get_pedestrian_start_wp(carla_map, ego_waypoint, position, driving_distance)
            if npc_waypoint not in npc_ini_wp_ls:
                break
    return npc_waypoint, npc_behavior_distance, npc_initial_speed, npc_ego_yaw

def mutate_weather(degress):
    # mutate weather[0，100]
    if degress <= 0:
        return 0
    if degress >= 100:
        return 100
    degress += 5
    return degress
def get_danger_scenario_input():
    input_window = sg.Window('Customize Dangerous Driving Scenarios').Layout(ui.get_input_layout())
    # Display and interact with the Window
    event, values = input_window.read()  # Part 4 - Event loop or Window.read call
    return event, values

def handle_collision(event, collision_ls):
    # 在这里处理碰撞事件
    # event.other_actor 即是与ego车辆发生碰撞的参与者
    if event.other_actor.type_id.startswith('vehicle'):
        print(f"Ego vehicle collided with vehicle {event.other_actor.id}")
        collision_ls[0] = True

def initial_set_vehicle(npc_vehicle, npc_ini_wp):
    npc_vehicle.set_autopilot(False)
    npc_vehicle.set_transform(npc_ini_wp.transform)
    vb.set_velocity(npc_vehicle, 0)
    vb.set_throttle(npc_vehicle, 0)
    return npc_vehicle

def initial_set_pedestrian(npc_pedestrian, npc_ini_wp):
    npc_pedestrian.set_transform(npc_ini_wp.transform)
    control = npc_pedestrian.get_control()
    control.speed = 0
    npc_pedestrian.apply_control(control)
    return npc_pedestrian
def detect_collision(event, save_file_name):
    """
    碰撞函数
    :param event: 碰撞事件对象
    """
    global scena_id
    global old_crash_scena_id
    if old_crash_scena_id != scena_id:
        old_crash_scena_id = scena_id
        impulse = event.normal_impulse
        intensity = math.sqrt(impulse.x ** 2 + impulse.y ** 2 + impulse.z ** 2)
        print('collision happen',intensity)
        with open('result/'+save_file_name+'_'+'crash.txt', 'a') as f:
            f.write(str(scena_id)+' '+'1\n')



# 角度归一化函数，确保角度在-pi到pi之间
def normalize_angle(angle):
    while angle > math.pi:
        angle -= 2 * math.pi
    while angle < -math.pi:
        angle += 2 * math.pi
    return angle

def judge_forward(tr_a, tr_b):
    """
       判断wp_a是否位于wp_b的左方。

       :param wp_a: 第一个waypoint对象
       :param wp_b: 第二个waypoint对象
       :return: 如果wp_a在wp_b的左前方则返回True，否则False
       """

    # 计算wp_a相对于wp_b的位置向量
    relative_pos = tr_a.location - tr_b.location

    # 计算wp_b的朝向角（从东向南为正值，转换为弧度）
    yaw_b_rad = math.radians(tr_b.rotation.yaw)

    # 计算相对位置向量与x轴的夹角（朝向北为0度）
    angle_with_x_axis = math.atan2(relative_pos.y, relative_pos.x)

    # 计算相对位置向量与wp_b前进方向的夹角
    angle_diff = normalize_angle(angle_with_x_axis - yaw_b_rad)

    # 判断是否在左前方：相对位置在左侧（负角度）且在前方（角度绝对值小于90度）
    # is_left = angle_diff < 0
    is_forward = abs(angle_diff) < math.pi / 2  # 90度转换为弧度
    print('angle diff', angle_diff)
    return is_forward

# def is_forward(wp_a, wp_b, carla_map):
#     # 计算wp_a相对于wp_b的位置向量
#     next_wp_b = carla_map.get_waypoint(wp_b.get_location(), project_to_road=True)
#     relative_pos_ego = next_wp_b.next(1)[0].transform.location-wp_b.get_location()
#     relative_pos_ego_npc = wp_a.transform.location - wp_b.get_location()
#
#     # 计算相对位置向量与x轴的夹角（朝向北为0度）
#     angle_with_x_axis_ego_self = math.atan2(relative_pos_ego.y, relative_pos_ego.x)
#     angle_with_x_axis_ego_npc = math.atan2(relative_pos_ego_npc.y, relative_pos_ego_npc.x)
#     # 计算相对位置向量与wp_b前进方向的夹角
#     angle_diff = normalize_angle(angle_with_x_axis_ego_npc - angle_with_x_axis_ego_self)
#     print('is_forward', angle_diff)
#     if abs(angle_diff) < math.pi/2:
#         return True
#     else:
#         return False



if __name__ == '__main__':
    # The vehicle in front of left insert.
    # danger_content_ls = ['The vehicle ahead suddenly brakes.', 'The vehicle in front of the left suddenly inserted.',
    #                      'The vehicle in front of the right suddenly inserted.', 'A pedestrian in front of the right cross the road.',
    #                      'The vehicle accelerated from behind.', 'The vehicle in front is approaching.']
    # danger_content = danger_content_ls[5]

    event, danger_scenario = get_danger_scenario_input()
    if event == 'Generate Scenario':
        danger_analysis_result = extract_danger_behavior(danger_scenario['scenario'])

        # 获取当前时间的时间戳
        timestamp = time.time()

        # 将时间戳转换为格式化的字符串，类似于"2023-04-01_10-30-45"
        current_time_str = time.strftime("%Y-%m-%d_%H-%M-%S", time.localtime(timestamp))
        save_file_name = current_time_str
        for dr in danger_analysis_result:
            save_file_name += '_'+dr
        client, world, carla_map = sim.inital_carla()
        # 设置天气
        set_weather = False
        if len(danger_analysis_result[-1]) > 1:
            # wt.set_weather(world, danger_analysis_result[-1], degree)
            set_weather = True
            weather = danger_analysis_result[-1]
        else:
            wt.set_weather(world, 'sunny', 0)
        # 设置允许循环运行的总时间（单位：秒）
        critical_scenario_thre = 3 #安全关键场景阈值，当ego车辆和其他车辆之间距离小于该值时，认为安全关键场景
        max_execution_time = 600 #10分钟
        # global scena_id
        # 创建npc，并且随机放置npc,创建车辆或者行人
        npc_actor = vb.create_npc(world, danger_analysis_result)
        ego_ini_wp_ls = []
        # 定制场景是否正确列表
        custom_scena_ls = [['scena_id', 'start_position_left_right','start_position_front','end_behavior_correct', 'critical_scenario'],]
        # 记录开始时间
        start_time = time.time()
        while (time.time()-start_time) < max_execution_time:
            weather_degress = 5
            ego_driving_dis = random.randint(20, 100)  # ego车辆行驶的距离
            if danger_analysis_result[0] == 'car':
                ego_ini_wp, npc_ini_wp = sim.get_driving_location(world, danger_analysis_result, ego_driving_dis,
                                                                  ego_ini_wp_ls)
                # 设置车辆的转向角
                if 'left' in danger_analysis_result[2] or 'right' in danger_analysis_result[2]:
                    npc_ego_yaw = 0.5  # npc车辆的初始角度，范围为（0,1）
                else:
                    npc_ego_yaw = 0
                npc_initial_speed = 10  # npc车辆的初始速度,单位为km/h，（0，200）
                npc_behavior_distance = 10  # npc车辆发生行为的位置与ego车辆的距离
                npc_throttle = 0.1  # npc的初始油门值,范围为（0，1）
            else:
                ego_ini_wp, npc_ini_wp = sim.get_walk_location(world, danger_analysis_result, ego_driving_dis,
                                                                  ego_ini_wp_ls)
                # 设置行人的初始角度应该垂直于ego
                npc_ego_yaw = 90
                npc_initial_speed = 1.5  # npc行人的初始速度,单位为m/s，（1，2）
                npc_behavior_distance = 6  # npc行人发生行为的位置与ego车辆的距离
            npc_ini_wp_ls = [npc_ini_wp]
            ego_vehicle = vb.create_ego(world, ego_ini_wp)

            blueprint_library = world.get_blueprint_library()

            collision_sensor = world.spawn_actor(blueprint_library.find('sensor.other.collision'),
                                                 carla.Transform(), attach_to=ego_vehicle)

            collision_sensor.listen(lambda event: detect_collision(event, save_file_name))

            ego_ini_wp_ls.append(ego_ini_wp)
            pre_min_score = npc_behavior_distance+1
            while True:
                if (time.time()-start_time) >= max_execution_time:
                    break
                # 设置天气
                if set_weather == True:
                    wt.set_weather(world, weather, weather_degress)
                custom_scena = [scena_id,]
                # 设置npc的初始位置
                tf = ego_ini_wp.transform
                # 设置ego的起点和驾驶距离
                ego_vehicle.set_transform(tf)
                set_ego_routing(ego_ini_wp, ego_driving_dis)
                # 如果是刹车，则首先给初始的速度
                if danger_analysis_result[1] == 'brake':
                    npc_actor = initial_set_vehicle(npc_actor, npc_ini_wp)
                    vb.set_velocity(npc_actor, npc_initial_speed)
                elif danger_analysis_result[1] not in ('accelerate'):
                    if danger_analysis_result[0] == 'car':
                        npc_actor = initial_set_vehicle(npc_actor, npc_ini_wp)
                    else:
                        npc_actor = initial_set_pedestrian(npc_actor, npc_ini_wp)
                ego_waypoint = carla_map.get_waypoint(ego_vehicle.get_transform().location,
                                                      project_to_road=True,
                                                      lane_type=carla.LaneType.Driving)
                # # # 判断npc的初始位置是否在最终位置的左边和右边，左边的land_id大于右边
                # print(npc_ini_wp.lane_id, ego_waypoint.lane_id)
                if npc_ini_wp.lane_id == ego_waypoint.lane_id:
                    custom_scena.append(True)
                else:
                    custom_scena.append(False)
                # forward_result = judge_forward(npc_actor.get_transform(), ego_waypoint.transform)
                # custom_scena.append(forward_result)
                print('***npc的初始位置***', custom_scena)
                # 判断npc的初始位置是否在最终位置的前方
                min_score = npc_behavior_distance  # 保留上一次ego和npc之间的fit score
                # 当ego车辆行驶到npc车辆的发生危险行为的位置时，开始设置npc车辆的行为
                threshold = 10 # ego停车的次数是10
                ego_stop_time = 0 #用来记录ego车辆停止的次数
                old_ego_npc_dis = -1
                tag_ego_stop = False
                ego_npc_parallel = False
                if 'rear' in danger_analysis_result[2]:
                    npc_behavior_distance = -npc_behavior_distance + 0.1
                while True:
                    ego_loc = ego_vehicle.get_location()
                    ego_npc_dis = ego_loc.distance(npc_ini_wp.transform.location)
                    # 如果后方的车辆变道，则需要比较ego是否在npc前方，如果npc接近前方ego，它两之间的距离>=npc_behavior_distance，则变道
                    if ('rear' in danger_analysis_result[2]) and (ego_npc_dis <= 4):
                        ego_npc_parallel = True
                    if ego_npc_parallel == True:
                        ego_npc_dis = -ego_npc_dis
                    print(ego_npc_dis, npc_behavior_distance)
                    if (ego_npc_parallel == True) or ('rear' not in danger_analysis_result[2]):
                        if ego_npc_dis < npc_behavior_distance:
                            if 'rear' in danger_analysis_result[2]:
                                npc_behavior_distance = min_score
                            break
                    if danger_analysis_result[1] == 'brake':
                        npc_actor.set_transform(npc_ini_wp.transform)
                    print('------')
                    time.sleep(0.5) # 等待0.5s，让车辆走一会
                    # if ego_stop_time == 0:
                    #     old_ego_npc_dis = int(ego_npc_dis)
                    if old_ego_npc_dis == int(ego_npc_dis):
                        ego_stop_time += 1
                    else:
                        old_ego_npc_dis = int(ego_npc_dis)
                        ego_stop_time = 0
                    if ego_stop_time > threshold:
                        npc_behavior_distance = min_score
                        tag_ego_stop = True
                        break
                # 如果ego车辆停止，则跳出循环，从 新设置ego和npc
                if tag_ego_stop == True:
                    collision_sensor.destroy()
                    break
                if danger_analysis_result[1] in ('accelerate'):
                    if danger_analysis_result[0] == 'car':
                        npc_actor = initial_set_vehicle(npc_actor, npc_ini_wp)
                    else:
                        npc_actor = initial_set_pedestrian(npc_actor, npc_ini_wp)

                ego_waypoint = carla_map.get_waypoint(ego_vehicle.get_transform().location,
                                                      project_to_road=True,
                                                      lane_type=carla.LaneType.Driving)
                forward_result = judge_forward(npc_ini_wp.transform, ego_waypoint.transform)
                custom_scena.append(forward_result)

                if danger_analysis_result[0] == 'car':
                    npc_actor = set_npc_ve_behavior(npc_actor, npc_ini_wp, npc_initial_speed, npc_throttle,
                        npc_ego_yaw, danger_analysis_result)
                else:
                    npc_actor = set_npc_pe_behavior(npc_actor, npc_ini_wp, npc_initial_speed, npc_ego_yaw, danger_analysis_result)
                # ego_route_wp = get_ego_route_waypoint(ego_vehicle, carla_map)
                # ego_route_wp_ls.append(ego_route_wp)
                # npc_route_wp = get_npc_route_waypoint(npc_vehicle, carla_map)
                # npc_route_wp_ls.append(npc_route_wp)
                # ego_vehicle
                # print('npc', npc_vehicle)
                #查看ego车辆和NPC车辆的位置是否逐渐变小，如果变大，则重新开始
                while True:
                    # # 看ego车辆和npc车辆的拟合函数的位置是否减小
                    # print('--score before---')
                    score = vb.fit_fuction(ego_vehicle, npc_actor)
                    print('---两个车之间的距离---',score)
                    collision_ls = [False]
                    # print('--list before---')
                    # print(score)
                    if score < min_score:
                        # print('--min score before---')
                        min_score = score
                        # print('--min score after--')
                        time.sleep(1)
                        # print('--sleep--')
                        # 记录npc车辆的路线
                    else:
                        # print('--score add--')
                        break
                # ego_npc_route_wp_ls.append({'ego_route': ego_route_wp, 'npc_route': npc_route_wp})
                # ego_route_wp_ls.append(ego_route_wp)
                # npc_route_wp_ls.append(npc_route_wp)
                # reset npc for not influence the ego vehicle
                # 左边车辆插入本车道：判断npc的初始位置是否在最终位置的左边
                ego_waypoint = carla_map.get_waypoint(ego_vehicle.get_transform().location, project_to_road=True,
                                                      lane_type=carla.LaneType.Driving)
                npc_waypoint = carla_map.get_waypoint(npc_actor.get_transform().location, project_to_road=True,
                                                      lane_type=carla.LaneType.Driving)
                # print('----ego and npc lane_id----')
                # print(ego_waypoint.lane_id, npc_waypoint.lane_id)
                # 判断是否生成定制场景
                if ego_waypoint.lane_id != npc_waypoint.lane_id:
                    control = npc_actor.get_control()
                    current_steer = control.steer  # 正值代表右转，负值代表左转
                    # print('---npc changes lane---', current_steer)
                    if current_steer < 0:
                        custom_scena.append(True)
                    else:
                        custom_scena.append(False)
                else:
                    custom_scena.append(True)
                # 判断生成的定制场景是否是安全关键场景
                print('---判断场景是否是安全关键场景,最小距离，判定条件---', min_score, critical_scenario_thre)
                # npc_vel = npc_vehicle.get_velocity()
                # npc_vel_sqa = math.sqrt(npc_vel.x ** 2 + npc_vel.y ** 2 + npc_vel.z ** 2)
                # if int(npc_vel_sqa) == 0:
                #     custom_scena.append(True)
                # else:
                #     custom_scena.append(False)
                # if min_score <= critical_scenario_thre:
                #     custom_scena.append(True)
                # else:
                #     custom_scena.append(False)
                custom_scena.append(min_score)
                # is_change_lane = is_left_forward(npc_vehicle.get_transform(), ego_vehicle.get_transform())
                custom_scena_ls.append(custom_scena)
                scena_id += 1
                # 一个场景结束后，将npc放到ego的行驶路线之外，防止影响下次场景生成
                if danger_analysis_result[0] == 'car':
                    npc_actor = initial_set_vehicle(npc_actor, ego_ini_wp.next(110)[0])
                    npc_ini_wp, npc_behavior_distance, npc_initial_speed, npc_throttle, npc_ego_yaw \
                        = mutate_ve_npc(npc_ini_wp_ls, ego_ini_wp, ego_driving_dis, npc_behavior_distance,
                                     npc_initial_speed, npc_throttle, npc_ego_yaw, danger_analysis_result)
                else:
                    npc_actor = initial_set_pedestrian(npc_actor, ego_ini_wp.next(110)[0])
                    npc_ini_wp, npc_behavior_distance, npc_initial_speed, npc_ego_yaw \
                        = mutate_pe_npc(npc_ini_wp_ls, ego_ini_wp, ego_driving_dis, npc_behavior_distance,
                                     npc_initial_speed, npc_ego_yaw, danger_analysis_result, carla_map)
                if set_weather == True:
                    weather_degress = mutate_weather(weather_degress)
                    print('--天气的值--', weather_degress)
                npc_ini_wp_ls.append(npc_ini_wp)
                print('--mutation--')
                print(npc_behavior_distance, npc_initial_speed, npc_ego_yaw)

                # 如果本次场景ego车辆和npc之间的拟合函数值没有减小，则重新设置ego车辆
                if pre_min_score <= min_score:
                    print('---pre<min,重新修改和ego相关的场景变量---')
                    collision_sensor.destroy()
                    break
                else:
                    print('---pre>min---')
                    pre_min_score = min_score

        #将是否生成定制场景的结果保存到csv文件中
        with open('result/'+save_file_name+'_'+'scena.csv', 'w', newline='') as file:
            writer = csv.writer(file)
            writer.writerows(custom_scena_ls)
        # elif danger_analysis_result[0] == 'pedestrian':
        #     is_driving = sim.get_walk_location(world, danger_analysis_result, 8)
        #     if is_driving:
        #         ego_ini_wp, pes_ini_tf, pes_end_wp = is_driving
        #     ego_vehicle = vb.set_ego(world, ego_ini_wp)
        #     set_ego_routing(ego_ini_wp, None,20)
        #     time.sleep(1)
        #     pb.set_pedestrian(world, pes_ini_tf, pes_end_wp)
                # if is_driving:
                #     ego_ini_wp, npc_ini_wp = is_driving
                #     break
                # else:
                #     world, carla_map = sim.reload_map(client, 'Town0'+str(i+1))