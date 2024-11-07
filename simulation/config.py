#修改carla bridge配置，启动carla bridge
import yaml
import docker
import subprocess

carla_bridge_setting_path = '/home/hahabai/code/apollo-8.0.0/modules/carla_bridge/config/settings.yaml'
def modify_map(map_name):
    with open(carla_bridge_setting_path, 'r') as file:
        setting = yaml.safe_load(file)
    setting['carla']['town'] = map_name
    with open(carla_bridge_setting_path, 'w') as file:
        yaml.safe_dump(setting, file)

def run_carla_bridge():
    # client = docker.from_env()
    # container = client.containers.get('apollo_dev_root')
    # container.exec_run('python /apollo/modules/carla_bridge/main.py')
    # 执行Docker命令，进入root权限的容器
    obj = subprocess.Popen('sudo bash /home/hahabai/code/apollo-8.0.0/docker/scripts/dev_into.sh',
                           stdout=subprocess.PIPE, shell=True)
    output, error = obj.communicate()
    if obj.returncode != 0:
        print(f"Error: {error}")
    else:
        print(f"Output: {output.decode()}")
    # obj.stdin.write("012")
    # obj.stdin.write("cd modules/carla_bridge")
    # obj.stdin.write("./install.sh")
    # obj.stdin.write("source ~/.bashrc")
    # obj.stdin.write("cd ../..")
    # obj.stdin.write("source cyber/setup.bash")
    # obj.stdin.write("python /apollo/modules/carla_bridge/main.py")
    # out_error_list = obj.stdout.read()
    # print(out_error_list)

if __name__ == '__main__':
    #modify_map('Town01')
    run_carla_bridge()