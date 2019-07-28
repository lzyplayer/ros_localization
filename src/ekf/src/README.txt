这是一份用来解释各个可执行文件具体内容的可爱文档n(*≧▽≦*)n

<maintainer email="1159364090@qq.com">ShenYanqing</maintainer>

基于EKF滤波框架，有bicycle模型和akerman模型
①akerman模型：包含轮速的计算，输出归到后轴中心
②bicycle模型：不包含轮速的计算，输出归到GPS接收机
量测信息包含 二维位置、加速度、角速度、速度、偏航角、方向盘转角、四轮轮速

add_executable(bicycleslamlidarnode src/bicycleslamlidar.cpp)  bicycle模型，融合SLAM 激光里程计 方向盘转角 IMU
add_executable(akermanslamlidarnode src/akermanslamlidar.cpp)  akerman模型，融合SLAM 激光里程计 方向盘转角 IMU 轮速计


##无依托框架的各种版本
add_executable(akermangpsnode src/akermangps.cpp)              akerman模型，融合GPS  方向盘转角 IMU 轮速计
add_executable(akermanlidarnode src/akermanlidar.cpp)          akerman模型，融合GPS  激光里程计 方向盘转角 IMU 轮速计
add_executable(akermangpsvelnode src/akermangpsvel.cpp)        akerman模型，融合GPS  方向盘转角 IMU 轮速计 林肯车车速
add_executable(akermannode src/akerman-un.cpp)                 akerman模型，融合GPS  激光里程计 方向盘转角 IMU 轮速计 林肯车车速  选择性注释即可得到akermanlidar、akermangps、akermangpsvel


注意事项：加速度