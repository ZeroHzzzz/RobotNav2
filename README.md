# 医疗巡诊机器人

## 软件环境

ROS2 iron

## 特点

仅使用一个激光雷达进行建图/定位/导航,精度比 amcl+rf2o_laser_odometry 的方式高,且不容易发生跳变

## 注意事项

代码没有整理的很好,可能需要先行学习 ros 相关知识.主要的就是使用了 cartographer.在 dev 分支中也放入了未经过测试的 rtap 定位方式的代码,供后人参考

testdddddddd
