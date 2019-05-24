#ros_qtc_plugin编译
ROS下编译qtc插件

## 准备
在编译qtc插件之前，需要准备下载源码

* [QT Creator源码](http://download.qt.io/archive/qtcreator/)
* [ros_qtc_plugin源码](https://github.com/ros-industrial/ros_qtc_plugin)

!!! warning

	需要注意两个源码的版本需要对应，建议使用系统中的QT版本所对应的QT Creator版本

## 编译
在编译之前，我们需要注意以下事项

* QT Creator和ros_qtc_plugin的源码应当放在同一目录下
* QT Creator的源码目录名应当为`qt-creator`
* QT Creator的编译目录名应当为`qt-creator-build`


### 编译 lxqt-build-tools
``` bash
$ git clone https://github.com/lxqt/lxqt-build-tools.git
$ cd lxqt-build-tools
$ cmake ./
$ sudo make install
```

### 编译 qtermwidget
``` bash
$ git clone https://github.com/lxqt/qtermwidget.git
$ cd qtermwidget
$ cmake ./
$ make; sudo make install
```
### 编译ros_qtc_plugin插件
使用qtcreator进行编译即可
