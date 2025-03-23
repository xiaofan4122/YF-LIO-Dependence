# YF-LIO-Dependence

这里是yifanlio的依赖库，其中包含三个模块
- FW-mini-ros1 ： 煜禾森小车底盘的ROS1驱动
- [Livox-SDK2](https://github.com/Livox-SDK/Livox-SDK2/tree/master) ： Livox激光雷达开发套件
- [Livox_ros_driver2](https://github.com/Livox-SDK/livox_ros_driver2) ： 激光雷达ROS驱动包

## 下载

由于本仓库中包含两个子仓库，使用`--recursive`命令来递归克隆子模块

```bash
git clone --recursive https://github.com/xiaofan4122/YF-LIO-Dependence.git
```

## 安装

确保在ROS1环境下安装

### FW-mini-ros1

以下命令均在YF-LIO-Dependence根目录下执行
```bash
cd FW-mini-ros1
catkin_make
source devel/setup.bash
cd ..
```

**如果之前安装过对应驱动包，下面的步骤可以跳过**

### Livox-SDK2
以下命令均在YF-LIO-Dependence根目录下执行
```bash
cd Livox-SDK2
mkdir build
cd build
cmake .. && make -j
sudo make install
cd ../..
```

### livox_ros_driver2
以下命令均在YF-LIO-Dependence根目录下执行
```bash
cd ws_livox/src/livox_ros_driver2
./build.sh ROS1
cd ../..
source devel/setup.bash
cd ..
```