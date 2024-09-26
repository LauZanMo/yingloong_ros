# YingLoong ROS

## 安装

- 安装依赖

```bash
sudo apt-get install git python3-vcstool python3-rosdep python3-colcon-common-extensions
```

- 自动补全依赖并编译

```bash
mkdir -p yl_ws/src && cd yl_ws/src \
git clone https://github.com/LauZanMo/yingloong_ros \
vcs-import < ./yingloong_ros/dependencies.yaml \
cp ./yingloong_ros/dependencies.yaml ../ \
cd .. \
rosdep install --from-paths src --ignore-src -r -y \
colcon build --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
```

