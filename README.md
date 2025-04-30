# 存在疑问:

1. colcon build --symlink-install (For python?)


# 使用记录
```bash
# 进入 launch 文件夹，可以直接启动 launch 而不需要拷贝到 install
ros2 launch launch.py
```


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

