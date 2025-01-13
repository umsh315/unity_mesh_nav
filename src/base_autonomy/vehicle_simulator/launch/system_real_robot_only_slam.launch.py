<launch>

  <arg name="world_name" default="real_world"/>
  <arg name="sensorOffsetX" default="0.3"/>
  <arg name="sensorOffsetY" default="0.0"/>
  <arg name="cameraOffsetZ" default="0.0"/>
  <arg name="vehicleX" default="0.0"/>
  <arg name="vehicleY" default="0.0"/>
  <arg name="checkTerrainConn" default="true"/>

  <!-- <node pkg="joy" exec="joy_node" name="ps3_joy" output="screen" >
    <param name="dev" value="/dev/input/js0" />
    <param name="deadzone" value="0.12" />
    <param name="autorepeat_rate" value="0.0" />
  </node> -->
  
  <!-- <node pkg="go2_h264_repub" exec="go2_h264_repub" name="go2_h264_repub" output="screen">
    <param name="multicast_iface" value="enp3s0"/> 
  </node> -->

  <include file="$(find-pkg-share point_lio_unilidar)/launch/mapping_utlidar.launch">
    <arg name="rviz" value="true"/>
  </include>


</launch>
