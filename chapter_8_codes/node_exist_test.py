import rospy
import rosgraph
ros_master = rosgraph.Master("")
ros_node_name = 'usb_cam'

rospy.init_node('node_tester')

rate = rospy.Rate(2) # 10hz
while not rospy.is_shutdown():
    try:
        result = ros_master.lookupNode(ros_node_name)
        print('The node <%s> is at <%s>' % (ros_node_name, result))
    except:
        print('Can not reach the node <%s>' % ros_node_name)
    rate.sleep()
