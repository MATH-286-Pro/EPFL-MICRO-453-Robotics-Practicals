#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    # 定义要启动的节点
    Pub_Node_Temp = Node(
        package    = 'temp_pkg',
        namespace  = 'test',
        executable = 'Pub_Node_Temp',
        name       = 'Pub_Node_Temp',

    )

    Pub_Node_Humi = Node(
        package    = 'temp_pkg',
        namespace  = 'test',
        executable = 'Pub_Node_Humi',
        name       = 'Pub_Node_Humi',
    )

    Sub_Node = Node(
        package    = 'temp_pkg',
        namespace  = 'test',
        executable = 'Sub_Node',
        name       = 'Sub_Node',
    )

    return LaunchDescription([
        Pub_Node_Humi,
        Pub_Node_Temp,
        Sub_Node,
    ])
