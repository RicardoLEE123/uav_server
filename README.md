# UAV Server

## 代码下载
运行以下命令克隆代码到本地：
```
git clone https://github.com/RicardoLEE123/uav_server.git
```

## 代码使用
```
roslaunch uav_server uav_server.launch
```

## 安装可视化日志分析

```
sudo apt install ros-noetic-plotjuggler
sudo apt install ros-noetic-plotjuggler-ros
```

使用
```
roslaunch uav_server bagplay.launch 
```

## 遥控器设置

遥控器通道映射（从1开始计数）：  
```
    油门：  65 ( 低——>高        174   ——> 1811        )
    YAW:   66 ( 左——>右        65496 ——> 40          )
    PITCH: 67 ( 高——>低        40    ——> 65496       )
    ROLL:  68 ( 左——>右        40    ——> 65496       )
    SA:    69 ( 未按下——>按下   191   ——> 1792        )
    SB:    70 ( 低——>中——>高   191 ——> 997 ——> 1792  )
    SC:    71 ( 低——>中——>高   191 ——> 997 ——> 1792  )
    SD:    72 ( 未按下——>按下   191   ——> 1792        )
```

### 起飞
按下 **SA** 解锁，等待螺旋桨旋转然后直接推油门；  
**SD** 是定点/定高切换，按下去为定点，不按下去为定高；  
**SB** 对应：最下面是 **manual**，中间是定点/定高，最上面是 **offboard**。

### 进入 offboard 模式
1. **SB 高位**，**SD 按下**，解锁即可进入 offboard 模式，此时可以直接给定轨迹飞行。  
2. **SC 低位**：进入执行轨迹模式；**SC 高位**：降落并上锁。

### 飞行记录
每次飞行都会记录发送的轨迹和odom信息，最新一次的飞行保存在最新的日志中，可以提取对应日志并基于可视化进行查看，日志保存在```data```路径下


### 飞机的odom代码更新步骤
1. 备份
```
    cp -r /home/vslam/catkin_ws_vins/src/seeker_utils /home/vslam
```

2. 删除&更新
```
    rm -rf catkin_ws_vins
    scp -r /home/uav/water_swarm/src/uav_simulator/Utils/uav_server/source/catkin_ws_vins_omni_new_pub.tar.gz vslam@10.42.0.2:/home/vslam
    tar -zxvpf catkin_ws_vins_omni_new_pub.tar.gz
    sync
```

3. 卸载&重装
```
    md5sum seeker_utils/config/seeker1/kalibr_cam_*
    rosrun robot_upstart uninstall seeker
```

## 未解决的问题
1. 需要记录每次飞行的轨迹 ✅  
2. 轨迹启动的时候很长，明确切换状态延迟的原因 (check代码) ✅  
3. 轨迹会出现抖动，tracking 的时候会有急停  ✅ 
4. 起飞的时候 yaw 角会旋转，需要明确问题的解决  ✅ 
5. 响应 mavlink 上锁的指令  


