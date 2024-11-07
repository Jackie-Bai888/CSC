import carla
# 主要进行变化的天气是雨天、雾天和晚上
def set_rainy(degree):
    # 雨天
    rain = carla.WeatherParameters(
        cloudiness=80.0,  # 云量 (0.0 - 100.0)
        precipitation=degree,  # 降水量 (0.0 - 100.0), 一般70是大雨
        precipitation_deposits=degree,  # 降水沉积 (0.0 - 100.0)
        wind_intensity=10.0,  # 风力强度 (0.0 - 100.0)
        sun_azimuth_angle=180.0,  # 太阳方位角 (0.0 - 360.0)
        sun_altitude_angle=3.0, # Altitude angle of the sun. Values range from -90 to 90 corresponding to midnight and midday each.
        scattering_intensity=0
    )
    return rain

def set_cloudy(degree):
    # 阴天
    cloud = carla.WeatherParameters(
        cloudiness=50.0,  # 云量 (0.0 - 100.0)
        precipitation=0.0,  # 降水量 (0.0 - 100.0)
        precipitation_deposits=0.0,  # 降水沉积 (0.0 - 100.0)
        wind_intensity=0.0,  # 风力强度 (0.0 - 100.0)
        sun_azimuth_angle=10, # The azimuth angle of the sun. Values range from 0 to 360. Zero is an origin point in a sphere determined by Unreal Engine.
        sun_altitude_angle=90, # Altitude angle of the sun. Values range from -90 to 90 corresponding to midnight and midday each.
    )
    return cloud

def set_sunny(degree):
    # 晴天
    sun = carla.WeatherParameters(
        cloudiness=0.0,  # 云量 (0.0 - 100.0)
        precipitation=0.0,  # 降水量 (0.0 - 100.0)
        precipitation_deposits=0.0,  # 降水沉积 (0.0 - 100.0)
        wind_intensity=0.0,  # 风力强度 (0.0 - 100.0)
        sun_azimuth_angle=80.0,  # 太阳方位角 (0.0 - 360.0) 0.0表示太阳在正东方
        sun_altitude_angle = 80.0  # 太阳高度角，90.0表示太阳在天空最高点
    )
    return sun

def set_foggy(degress):
    # 雾天
    fog = carla.WeatherParameters(
        cloudiness= 70.0,  # 云量 (0.0 - 100.0)
        precipitation=0.0,  # 降水量 (0.0 - 100.0)
        precipitation_deposits=0.0,  # 降水沉积 (0.0 - 100.0)
        wind_intensity = 0.0,  # 风力强度 (0.0 - 100.0)
        sun_azimuth_angle = 20.0,  # The azimuth angle of the sun. Values range from 0 to 360. Zero is an origin point in a sphere determined by Unreal Engine.
        sun_altitude_angle = 5.0,  # Altitude angle of the sun. Values range from -90 to 90 corresponding to midnight and midday each.
        scattering_intensity = 0.5,
        fog_density = degress,  #Fog concentration or thickness. It only affects the RGB camera sensor. Values range from 0 to 100.
        fog_distance = 10,  #Fog start distance. Values range from 0 to infinite.
        fog_falloff = 1 #A value of 1 is approximately as dense as the air, and reaches normal-sized buildings.
    )
    return fog

def set_windy():
    # 有风
    dust_storm = carla.WeatherParameters(
        cloudiness=0.0,  # 云量 (0.0 - 100.0)
        precipitation=0.0,  # 降水量 (0.0 - 100.0)
        precipitation_deposits=0.0,  # 降水沉积 (0.0 - 100.0)
        wind_intensity=100.0,  # 风力强度 (0.0 - 100.0)
        sun_azimuth_angle=145.0,  # 太阳方位角 (0.0 - 360.0) 0.0表示太阳在正东方
        sun_altitude_angle=145.0  # 太阳高度角，90.0表示太阳在天空最高点
    )
    return dust_storm

def set_night(degress):
    # 夜景
    new_degress = 10-degress
    dust_storm = carla.WeatherParameters(
        cloudiness=0.0,  # 云量 (0.0 - 100.0)
        precipitation=0.0,  # 降水量 (0.0 - 100.0)
        precipitation_deposits=0.0,  # 降水沉积 (0.0 - 100.0)
        sun_azimuth_angle=0.0,  # 太阳方位角 (0.0 - 360.0) 0.0表示太阳在正东方
        sun_altitude_angle=new_degress  # 太阳高度角，90.0表示太阳在天空最高点
    )
    return dust_storm

def set_weather(world, weather, degree):
    weather = 'set_'+weather
    weather_par = globals()[weather](degree)
    world.set_weather(weather_par)  # 应用天气参数
    print('------')

if __name__ == '__main__':
    import simulator as sim
    client, world, carla_map = sim.inital_carla()
    set_weather(world, 'sunny', -90)