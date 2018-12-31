try:
    import rospy
    ros_exist = True
except ImportError:
    ros_exist = False

ros_error = SystemError("Please make sure that ROS has been properly set up!")
