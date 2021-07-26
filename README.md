# SRSLAM: SCUT RoboLab Simultaneous Localization And Mapping

![Image text](img-folder/91426772_p0_master1200.jpg)

[![Author](https://img.shields.io/badge/Author-cypypccpy-blue.svg "Author")](https://github.com/cypypccpy "Author")
[![license](https://img.shields.io/github/license/:user/:repo.svg)](LICENSE)
[![standard-readme compliant](https://img.shields.io/badge/readme%20style-standard-brightgreen.svg?style=flat-square)](https://github.com/RichardLitt/standard-readme)
<br></br>

## 内容列表

- [背景](#背景)
- [安装](#安装)
- [用法](#用法)
- [TODO](#TODO)
- [如何贡献](#如何贡献)
- [使用许可](#使用许可)
<br></br>

## 背景

- 本库是基于LK光流前端与GTSAM后端的SLAM系统，纯个人瞎写，用于SLAM学习。
- 本库基于ROS，前后端分离且实现了基本的数据类型与传感器类型，可拓展性强，后续可以加入基于不同算法的前端或后端，用于SLAM学习。
- 目前只实现了前端部分，还差后端与可视化。
- 目前只支持双目相机作为传感器。
<br></br>

## 安装

环境:

- `Ubuntu 18.04`
- `ROS Melodic`
- `OpenCV 4.5.1`

依赖:

- [Eigen](http://www.boost.org/users/download/)
```bash
sudo apt-get install libeigen3-dev
```
- [GTSAM](http://www.cmake.org/cmake/resources/software.html)
```bash
git clone https://github.com/borglab/gtsam.git
cd gtsam
mkdir build && cd build
cmake ..
make check
sudo make install
```
- A modern compiler, i.e., at least gcc 4.7.3 on Linux.
<br></br>

## 用法

```bash
mkdir srslam_ws && cd srslam_ws && mkdir src
git clone https://github.com/cypypccpy/SRSLAM.git
cd ..
catkin_make
echo "source ~/srslam_ws/devel/setup.bash" >> ~/.bashrc
```

### 启动
```bash
rosrun srslam srslam_node
```

## TODO

<br></br>

## 如何贡献

See [the contributing file](CONTRIBUTING.md)!
<br></br>

## 使用许可

[MIT © Richard McRichface.](../LICENSE)

