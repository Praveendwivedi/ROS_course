
## Creating Publisher using Python

```bash
~/catkin_ws/src$ catkin_create_pkg pub_sub rospy std_msgs
~/catkin_ws/src/pub_sub$  mkdir scripts
~/catkin_ws/src/pub_sub$ cd scripts
~/catkin_ws/src/pub_sub$ gedit talker.py
```

* put following code in it
```
#!/usr/bin/env python3
import rospy 

from std_msgs.msg import String

def talker():
	pub = rospy.Publisher('myname', String, queue_size=10)##don't forget to write *queue_size* also otherwise it throws error when subsriber contacts
	rospy.init_node("talker", anonymous = True) ## anonymous is used to make the node unique from other node of same name
	rate = rospy.Rate(10)
	while not rospy.is_shutdown():
		name = "praveen"
		rospy.loginfo(name)
		pub.publish(name)
		rate.sleep()
		
if __name__ == '__main__':
	try:
		talker()
	except rospy.ROSInterruptException:
		pass
```
* in same folder create listener.py and put following code
```bash
#!/usr/bin/env python3

import rospy 

from std_msgs.msg import String

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    
def l():
	rospy.init_node('listener', anonymous = True)
	rospy.Subscriber("/myname", String, callback)
	rospy.spin()
if __name__ == '__main__':
	l()
  ```

* Add the following to your CMakeLists.txt. This makes sure the python script gets installed properly, and uses the right python interpreter. 

```bash
catkin_install_python(PROGRAMS script/talker.py script/listener.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
```
<Warning>
*bar*
</Warning>
