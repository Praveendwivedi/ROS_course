# ROS_course
## steps to create a simple bot model in gazebo and rviz

* step1. **create a folder**
```
$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws/
$ catkin_make
```
*The catkin_make command is a convenience tool for working with catkin workspaces. Running it the first time in your workspace, it will create a CMakeLists.txt link in your 'src' folder.*

* step 2. Before continuing source your new setup.*sh file: 
```
$ source devel/setup.bash
```
---
* step 3. Create a package named model_make in the  ~/catkin_ws/src
```
$ catkin_create_pkg model_make  urdf joint_state_publisher gazebo_ros

```
>Now you need to build the packages in the catkin workspace:
```
$ cd ..
$ catkin_make
```
>After the workspace has been built it has created a similar structure in the devel subfolder as you usually find under /opt/ros/$ROSDISTRO_NAME.
>
>To add the workspace to your ROS environment you need to source the generated setup file
```
$ . devel/setup.bash
```
---
> #### Note: you can go to this [link](https://www.youtube.com/watch?v=Ale55LcdZeE) to see the video tutorial of the same which is being discussed below
* step 4. goto *model_make* pkg and create new folder named *urdf* and create a xml file and name it *bot.xacro*. Put following content in that file:
```
<?xml version="1.0" ?>

<robot name = "bot" xmlns:xacro="http://www.ros.org/wiki/xacro">
<link name="base_link">
       <visual>
       	<origin xyz = "0 0 0" rpy =" 0 0 0"  />
         	<geometry>
           		<box size = "1 1 1" />
        	 </geometry>
       </visual>
</link>
       <joint name = "base_link1" type = "revolute" >
       <axis xyz = "0 0 1"/>
       <limit effort = "1000.0" lower = "-3.14" upper = "3.14"  velocity = "0.5"/>
       <origin rpy = "0 0 0" xyz = "0 0 0.5" />
       <parent link = "base_link" />
       <child link = "link1" />
      </joint>
       
<link name = "link1">
	<visual>
		<origin rpy =" 0 0 0" xyz = "0 0 0.2"   />
		<geometry>
			<cylinder  radius = "0.35" length = "0.4" />
		</geometry>
	</visual>
</link>
       

</robot>
```

* step 5. Now create a launch folder under model_make. Create a file named *rviz.launch* and put following code:
```
<launch>

<param name = "robot_description" command = "$(find xacro)/xacro --inorder '$(find model_make)/urdf/bot.xacro' "/>

 <node name="robot_state_publisher" pkg="robot_state_publisher" type="robot_state_publisher" />
 
<node name = "rviz" pkg = "rviz" type = "rviz" args="-d $(find model_make)/launch/config.rviz"/>

  <!-- send joint values -->
   <node name="joint_state_publisher" pkg="joint_state_publisher" type="joint_state_publisher" >
   </node>
   
    <node name="joint_state_publisher_gui" pkg="joint_state_publisher_gui" type="joint_state_publisher_gui" >
  </node>
</launch>
```

* step 6. Go back to *~/catkin_ws* and run following command in terminal:
```
$ catkin_make
$ rospack profile
```
* step 7. now launch the bot in rviz with following command:
```
$ roslaunch model_make rviz.launch 
```



