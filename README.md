1.fdilink_ahrs需要先安装serial
$ colcon build --packages-select serial --symlink-install
$ source install/setup.bash
2.livox_ros_driver2需要先安装Livox-SDK2
$ cd src/Livox-SDK2/
$ mkdir build
$ cd build
$ cmake .. && make -j
$ sudo make install
3.编译其他包
$ colcon build --symlink-install
