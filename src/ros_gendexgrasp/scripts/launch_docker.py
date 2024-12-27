import subprocess
import os

def activate_conda_and_run():
    # 激活conda环境并运行脚本
    conda_env = "gendexgrasp"
    conda_activate = f"source /opt/anaconda/etc/profile.d/conda.sh && conda activate {conda_env}"

    # 切换到指定目录
    target_dir = "/home/lab/GenDexGrasp/Gendexgrasp_ros_ok/src/ros_gendexgrasp"
    os.chdir(target_dir)

    # 运行两个 Python 文件
    contact_service_command = f"bash -c '{conda_activate} && python3 ros_Contact_service.py'"
    gengrasp_service_command = f"bash -c '{conda_activate} && python3 ros_Gengrasp_service.py'"

    # 使用 Popen 来并行运行两个命令
    try:
        # 启动 ros_Contact_service.py
        contact_process = subprocess.Popen(contact_service_command, shell=True)
        print("ros_Contact_service.py started")

        # 启动 ros_Gengrasp_service.py
        gengrasp_process = subprocess.Popen(gengrasp_service_command, shell=True)
        print("ros_Gengrasp_service.py started")

        # 等待两个进程完成
        contact_process.wait()
        gengrasp_process.wait()

    except subprocess.CalledProcessError as e:
        print(f"Error running the services: {e}")

if __name__ == "__main__":
    activate_conda_and_run()
