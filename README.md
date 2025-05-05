# 存在疑问:

1. colcon build --symlink-install (For python?)


# 使用记录
```bash
# 进入 launch 文件夹，可以直接启动 launch 而不需要拷贝到 install
ros2 launch launch.py
```


## **学习过程**

### Gazebo
- 安装
```bash
# 下载并安装 gazebo
curl -sSL http://get.gazebosim.org | sh

# 查看 gazebo 位置
which gazebo

# 启动 gazebo
gazebo
```

### **launch 文件**

什么是 .xml 文件  

.xml（eXtensible Markup Language）全称为可扩展标记语言，是一种标记语言，有点像 .html

```xml
<?xml version="1.0"?>                                   <!-- XML声明：指定XML版本 -->
<robot name="example">                                  <!-- 定义机器人模型，name属性指定机器人名称 -->

    <!-- 定义材质，name=blue -->
    <material name="blue">                              
        <color rgba="0 0 1 1" />                        <!-- color 元素定义材质的颜色，rgba 属性依次为 红、绿、蓝、透明度 (范围 0–1)，此处表示纯蓝且不透明 -->
    </material>                               

    <!-- 定义第一个杆 base_link -->
    <link name="base_link">                             
        <visual>                                        <!-- visual 元素：描述链接外观 -->
            <origin xyz="0 0 0"  rpy="0 0 0"/>          <!-- 定义 link 在坐标系中的位置 xyz 与姿态 roll pitch yaw -->
            <geometry>                                  <!-- geometry 元素：包含几何形状定义 -->
                <box size="0.2 0.3 0.6"/>               <!-- box 定义一个长方体，size 属性依次为 x、y、z 方向的尺寸（单位：米） -->
            </geometry>                       
            <material name="blue"/>                     <!-- 应用先前定义的 blue 材质 -->
        </visual>                              
    </link>                                   

    <!-- 定义第二个杆 second_link -->
    <link name="second_link">                           
        <visual>                                        <!-- visual 元素：描述链接外观 -->
            <origin xyz="0 0 0" rpy="0 0 0"/>           <!-- 定义 visual 在链接坐标系中的位置与姿态 -->
            <geometry>                                  <!-- geometry 元素：包含几何形状定义 -->
                <cylinder length="0.8" radius="0.05"/>  <!-- 定义长度 0.8 米、半径 0.05 米的圆柱体 -->
            </geometry>                        
            <material name="blue"/>                     <!-- 应用先前定义的 blue 材质 -->
        </visual>                             
    </link> 

    <!-- 注意:如果不添加 joint，则上面两个 link 都不会显示 -->
    <!-- 因为 URDF 不允许出现没有被连接的 link           -->
    <joint name="second_link_joint" type="fixed">
        <parent link="base_link"/>
        <child link="second_link"/>
        <origin xyz="0 0 0" rpy="0 0 0"/>
    </joint>

</robot>                                   
```

什么是 .URDF 文件  

.urdf (United Robotics Description Format) 统一机器人描述格式

# Session 1 Homework Questions

## 1. ROS2 Installation & Environment

1. **What do you observe when running the talker node after commenting out the `source /opt/ros/humble/setup.bash` line in your `~/.bashrc`?**
2. **What do you conclude? What does the `source` command do?**
3. **What is the benefit of having the `source /opt/ros/humble/setup.bash` command in your `~/.bashrc`?**

## 2. First Node Behavior

4. **What difference do you observe when you comment out the `rclpy.spin(minimal_node)` line in `minimal_node.py`, rebuild the package, and run it again?**

## 3. Parameters Overview

5. **What are the essential steps involved in working with a parameter?**
6. **What happens if no parameter value is provided during execution? Why?**

## 4. Services Overview

7. **What are the essential elements of a server?**
8. **What happens if the client node starts before the server? Why?**

