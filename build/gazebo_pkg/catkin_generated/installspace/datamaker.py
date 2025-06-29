#!/usr/bin/env python3
# -*- coding: utf-8 -*-
 
import sys
import os
import time
import rospy
import random
from gazebo_msgs.srv import SpawnModel, DeleteModel
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Pose, Point, Quaternion

def load_model(model_path):
    if not os.path.exists(model_path):
        raise FileNotFoundError(f"Model file {model_path} does not exist!")
    with open(model_path, 'r') as model_file:
        return model_file.read()

def spawn_model(model_name, model_xml, pose, reference_frame="world"):
    rospy.wait_for_service('/gazebo/spawn_sdf_model')
    try:
        spawn_model_prox = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
        spawn_model_prox(model_name, model_xml, "", pose, reference_frame)
        rospy.loginfo(f"Model {model_name} spawned successfully")
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")

def delete_all_models():
    """ 删除之前生成的所有板子 """
    room_model_map = {
        "A": "fruit_board_A",
        "B": "vage_board_B", 
        "C": "dessert_board_C"
    }
    for model_name in room_model_map.values():
        try:
            rospy.wait_for_service('/gazebo/delete_model', 1.0)
            delete_model_prox = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
            delete_model_prox(model_name)
        except (rospy.ServiceException, rospy.ROSException) as e:
            rospy.logwarn(f"Delete {model_name} failed: {str(e)}")

if __name__ == "__main__":
    rospy.init_node('spawn_fixed_models_node')
    delete_all_models()  # 清理旧模型
    
    # -------------------- 固定房间对应固定板子 --------------------
    room_model_configs = {
        # 房间A固定生成水果板，B固定蔬菜板，C固定甜点板
        "A": {
            "name_prefix": "fruit_board_",
            "model_path": "/home/zythyra/.gazebo/models/fruit2_board/model.sdf"  # 路径固定为水果板
        },
        "B": {
            "name_prefix": "vage_board_",
            "model_path": "/home/zythyra/.gazebo/models/vage3_board/model.sdf"    # 路径固定为蔬菜板
        },
        "C": {
            "name_prefix": "dessert_board_",
            "model_path": "/home/zythyra/.gazebo/models/dessert2_board/model.sdf" # 路径固定为甜点板
        }
    }
    
    # -------------------- 加载所有固定模型 --------------------
    models = {}
    for room_name, config in room_model_configs.items():
        try:
            model_xml = load_model(config["model_path"])
            models[room_name] = {
                "name_prefix": config["name_prefix"],
                "model_xml": model_xml
            }
        except Exception as e:
            rospy.logerr(f"Failed to load model for room {room_name}: {e}")
            continue

    # -------------------- 房间范围和生成规则 --------------------
    rooms = [
        {"name": "A", "x_range": (0, 0.95), "y_range": (1.4, 3.4)},
        {"name": "B", "x_range": (1.5, 2.5), "y_range": (1.4, 3.4)},
        {"name": "C", "x_range": (3.5, 4.3), "y_range": (1.4, 3.4)},
    ]

    # -------------------- 生成模型 --------------------
    for room in rooms:
        room_name = room["name"]
        if room_name not in models:
            rospy.logwarn(f"No model config for room {room_name}")
            continue
            
        model_config = models[room_name]
        
        # 生成随机位置
        pose = Pose()
        pose.position = Point(
            x=random.uniform(*room["x_range"]),
            y=random.uniform(*room["y_range"]),
            z=0
        )
        pose.orientation = Quaternion(x=0, y=0, z=0, w=1)

        # 生成模型（名称示例：fruit_board_A）
        model_name = f"{model_config['name_prefix']}{room_name}"
        spawn_model(model_name, model_config["model_xml"], pose)
    
    rospy.loginfo("所有固定板子生成完成！")