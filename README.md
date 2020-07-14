在运行前请确保已安装ubuntu16.04和ros kinetic 并安装好turtlebot3的相关包

上述环境的安装可以参照https://www.ncnynl.com/archives/201707/1840.html

此文件主要实现多个移动机器人的协同控制，SRC中包含了python源代码，launch中包含了启动gazebo仿真的launch文件，urdf中是模型的xroca文件

multi_turtlebot3.launch和turtlebot3_formation.py实现移动机器人的简单编队
效果如下

![image](https://github.com/zyq321/multi-agents-formulation/blob/master/gif/formation_res.gif)

multi_turtlebot3_visual.launch和turtlebot3_visual.py实现基于视觉的编队,具体算法可以参照
Formation Control of Nonholonomic Mobile Robots Without Position and Velocity Measurements

效果如下

![image](https://github.com/zyq321/multi-agents-formulation/blob/master/gif/visual_fomation.gif)

multi_turtlebot3_optimal.launch和实现多机器人的协同分布式kalman filter（DKF）
效果如下

观测值：

![image](https://github.com/zyq321/multi-agents-formulation/blob/master/gif/measurement.png)

单个机器人Kalman Filter

![image](https://github.com/zyq321/multi-agents-formulation/blob/master/gif/KF.png)

集群Distributed Kalman Filter

![image](https://github.com/zyq321/multi-agents-formulation/blob/master/gif/DKF.png)



如有问题可联系493050589@qq.com
