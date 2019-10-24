## 实时定位系统

1. 实时定位系统采用UKF算法将较低频率的激光定位结果与高频的IMU数据，融合出高频且更加平滑的定位结果。

2. ###### 依赖安装

   - openMP
   - alglib
   - eigen3
   - `sudo apt-get install -y libomp-dev libalglib-dev libeigen3-dev`以安装上述依赖
   - [激光雷达驱动](http://wiki.ros.org/velodyne/Tutorials/Getting Started with the Velodyne VLP16)安装（实时运行必需）

3. ###### 离线运行

   1. `source ./install/setup.bash`
   2. `roslaunch prm_localization yuyao_factory_offline.launch`离线启动定位系统
   3. `rosbag play /PATH_TO_TESTBAG/test.bag --clock` 以clock模式播放数据包
   4. 初始化2~5s后可以看到rviz中定位结果

4. ###### 在线运行

   1. `source ./install/setup.bash`

   2. （在确保激光雷达驱动已安装且测试可用后）启动激光雷达

      `roslaunch velodyne_pointcloud VLP16_points.launch`

   3. 检测`topic: /velodyne_points`是否有输出

      `rostopic echo /velodyne_points`

   4. 根据当前agv所在可能位置在`yuyao_factory.launch`中给出大致初始定位

      ```xml
      以右上坐标系（地图参考系）为参考系
      	<arg name="init_x" default="-4.1" />    初始x位置（单位：米）
          <arg name="init_y" default="6.0" />     初始y位置（单位：米）
          <arg name="init_yaw" default="-1.57" /> 初始yaw朝向（单位：弧度）
      给出位置为左下坐标系标识位置
      
          <arg name="init_x" default="0" />    
          <arg name="init_y" default="0" />     
          <arg name="init_yaw" default="0" /> 
      给出位置为右上坐标系标识位置
      红x轴 绿y轴 蓝z轴
      ```

      <img src="/home/vickylzy/.config/Typora/typora-user-images/1567480320406.png"  width="500px">

   5. 启动实时定位系统`roslaunch prm_localization yuyao_factory.launch`

      启动遇到错误请联系lzyplayer@stu.xjtu.edu.cn,并尝试

      `roslaunch prm_localization yuyao_factory_selfManager.launch`启动

   6. 可以在rviz中查看到当前定位，并看到红色（激光），黄色（UKF滤波）两种里程计信息

5. ###### 进阶设置

   激光定位计设置`/install/share/prm_localization yuyao_factory.launch`

   ```xml
   <?xml version="1.0"?>
   <launch>
   <!--argument-->
       <arg name="points_topic" default="/velodyne_points" />激光点云订阅话题
       <arg name="map_tf" default="odom" />地图坐标系名称
       <arg name="base_lidar_tf" default="velodyne" />激光坐标系名称
       <arg name="kf_odometry" default="/karman_filter_odom"/>滤波器里程计发布话题
       <arg name="base_foot_tf" default="base_footprint"/>agv控制中心坐标系名称（由于未标定激光雷达与控制中心相对位置，故暂不可用）
       <arg name="init_x" default="-4.1" />初始x
       <arg name="init_y" default="6.0" />初始y
       <arg name="init_yaw" default="-1.57" />初始yaw
       <arg name="lidar_height" default="0.4"/>雷达距离地面高度（设置时略小于实际高度）
       <arg name="trim_low" default="-0.4" />以雷达中心为高度0，裁剪其trim_low下点云
       <arg name="trim_high" default="7" />以雷达中心为高度0，裁剪其trim_high上点云
       <arg name="radius" default="20.0" />局部地图半径
       <arg name="mapUpdateTime" default="3" />局部地图更新时间
      <arg name="global_map_pcd_path" default="$(find prm_localization)/data/shunyu_factory_half.pcd" />全局地图路径
       <!--regis para-->
       <arg name="use_GPU_ICP" default="false"/>是否使用GPU
       <arg name="downsample_resolution" default="0.10" />地图降采样体素大小，单位m
       <arg name="TransformationEpsilon" default="0.01" />电云匹配收敛判断阈值
       <!--ndt-->
       <arg name="ndt_resolution" default="1.0" />匹配算法分辨率
       <!--filter-->
       <arg name="farPointThreshold" default="20" />去除当前远点信息
       <arg name="nearPointThreshold" default="0.98" />去除当前车体信息
       <arg name="manager_name" default="localization_nodelet_manager"/>nodelet_manager命名
       <!-- lidar_predict -->
       <arg name="lp_odom_rate" default="0"/>激光预测结果输出频率
   
   ```

   无迹卡尔曼滤波器设置`/install/share/ukf/launch/ukf.launch`

   ```xml
   <?xml version="1.0"?>
   <launch>
   <!--argument-->
     <!-- topic -->
     <arg name="lidar_topic" default="/odom" />
     <arg name="imu_topic" default="/pioneer_sensors/IMU_Xsens_RS232/raw_acceleration" />
     <arg name="out_topic" default="/karman_filter_odom" />
     <!-- 坐标系 -->
     <arg name="frame_id" default="odom_world" />
     <arg name="child_frame_id" default="base_foot_frame" />
   
    <!-- 参数调整主要针对R、Q矩阵，均为UKF内的参数矩阵 -->
     <!-- Matrix R 反映测量量的误差大小，分别是激光位置、朝向、速度，IMU的加速度、角速度；若认为误差偏大，则在此基础上增大-->
     <arg name="cov_slampos" default="0.04" />
     <arg name="cov_slamyaw" default="0.02" />
     <arg name="cov_slamvel" default="0.1" />
     <arg name="cov_imuacc" default="0.1" />
     <arg name="cov_imuang" default="0.1" />
     <!-- Matrix Q 反映模型对变量的信任度，分别为激光位置、朝向、速度，IMU的加速度、角速度；若提高信任度，数值在此基础上减小-->
     <arg name="ppos" default="0.05" />
     <arg name="pv" default="0.05" />
     <arg name="pyaw" default="0.01" />
     <arg name="pa" default="0.5" />
     <arg name="pw" default="0.02" />
     <!-- 是否后续去除向心加速度 -->
     <arg name="flag_linearacc" default="false" />
     <!-- 是否使用SLAM速度 -->
     <arg name="flag_slamv" default="false" />
     <!-- 是否后续进行加速度标定 -->
     <arg name="flag_cali" default="false" />
   ```

   ###### ukf.launch文件参数说明：

   - ###### topic名称说明  

   1. lidar_topic   雷达传感器原始数据的topic
   2. IMU_topic    IMU传感器原始数据topic
   3. out_topic     自定义输出位姿信息的topic

   - ###### 参数调整

   1. R矩阵反映测量量的误差大小，文件中的“cov_**” 分别是激光位置、朝向、速度，IMU的加速度、角速度；若认为误差偏大，则在launch文件基础上略微增大，反之同理
   2. Q矩阵反映模型对变量的信任度，文件中的“p**” 分别为激光位置、朝向、速度，IMU的加速度、角速度；若提高信任度，则在launch文件基础上略微减小，反之同理

   email : 1159364090@qq.com

   
