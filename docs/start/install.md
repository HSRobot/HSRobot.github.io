# 环境搭建

当前教程基于ROS indigo

------

## ROS环境搭建

### 设置源
```bash
sudo sh -c '. /etc/lsb-release && echo "deb http://mirrors.ustc.edu.cn/ros/ubuntu/ $DISTRIB_CODENAME main" > /etc/apt/sources.list.d/ros-latest.list'
```

### 设置密钥
```bash
sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
```

### 更新源
```bash
sudo apt-get update
sudo apt-get upgrade
```

### 安装ROS
```bash
sudo apt-get install ros-indigo-desktop-full
```

### 初始化rosdep
```bash
sudo rosdep init
```

### 更新rosdep
```bash
rosdep update
```

### 设置环境变量
```bash
echo source /opt/ros/indigo/setup.bash >> ~/.bashrc
```


## HIROP环境搭建

### 创建工作区
```bash
$ cd ~; mkdir catkin_ws
$ cd catkin_ws; mkdir src
$ catkin_make
$ echo source ~/catkin_ws/devel/setup.bash >> ~/.bashrc
```



### 下载源码
```bash
sudo apt-get install git
cd ~/catkin_ws/src
git clone https://github.com/HSRobot/hsr_irp.git
```

### 下载依赖项
```bash
sudo apt-get install ros-indigo-industrial-core
sudo apt-get install ros-indigo-moveit-full
sudo apt-get install ros-indigo-serial
sudo apt-get install ros-indigo-gazebo-ros-control
sudo apt-get install ros-indigo-joint-trajectory-controller 
sudo apt-get install ros-indigo-joint-state-controller
sudo apt-get install ros-indigo-effort-controllers
sudo apt-get install ros-indigo-position-controllers 
```

### 编译
```bash
catkin_make
```
