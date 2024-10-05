# YingLoong ROS

## 安装

- 安装[ROS 2](https://docs.ros.org/en/humble/index.html)（Humble）

- 安装依赖

```bash
sudo apt-get install git ros-dev-tools
```

- 自动补全依赖并编译

```bash
mkdir -p yl_ws/src && cd yl_ws/src && \
git clone https://github.com/LauZanMo/yingloong_ros && \
vcs-import < ./yingloong_ros/dependencies.yaml && \
cp ./yingloong_ros/colcon.meta ../ && \
cd .. && \
rosdep install --from-paths src --ignore-src -r -y && \
colcon build --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
```

