#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from ar_track_alvar_msgs.msg import AlvarMarkers
from std_srvs.srv import Empty, EmptyResponse

IDLE = 0
APPROACH_MARKER = 1
DOCK = 2
CHARGE = 3
UNDOCK = 4
     
def empty_cb(req):
    return EmptyResponse()
     
def ar_message_handler(data):
    global leitud_markeri_ID
	
    if len(data.markers) > 0:
        for marker in data.markers:
            rospy.loginfo("Detected marker with ID " + str(marker.id))
            leitud_markeri_ID = marker.id            
            rospy.loginfo("y coordinate: " + str(marker.pose.pose.position.y))
            	
    else:
        leitud_markeri_ID = None
        rospy.loginfo("No AR markers detected.")
		
def main():
    global olek
    olek = IDLE
    
    global leitud_markeri_ID
    leitud_markeri_ID = None

    rospy.init_node("ar_subscriber")
    ar_sub = rospy.Subscriber("ar_pose_marker", AlvarMarkers, ar_message_handler)
    empty_service = rospy.Service("dock", Empty, empty_cb)
    empty_service = rospy.Service("undock", Empty, empty_cb)
   
    velocity_pub = rospy.Publisher("cmd_vel", Twist, queue_size=0)
	
    rospy.sleep(2)
	
    loop_rate = rospy.Rate(10)
    starting_time = rospy.get_time()
	
    def move(time, x, y, z):
        starting_time = rospy.get_time()
        while not (rospy.is_shutdown()) and (rospy.get_time() - starting_time < time):
            robot_vel = Twist()
            robot_vel.linear.x = x
            robot_vel.linear.y = y
            robot_vel.linear.z = 0.0
            robot_vel.angular.x = 0.0
            robot_vel.angular.y = 0.0
            robot_vel.angular.z = z
            
            velocity_pub.publish(robot_vel)
			
            loop_rate.sleep()
	
    otsitav = 1
    while not rospy.is_shutdown():
	
        if olek == IDLE:
            move(0, 0, 0, 0)
            
        elif olek == APPROACH_MARKER: 
            move(0.3, 0, 0, 0.3)
            if marker.pose.pose.position.y == 0:
                move(1, 1, 0, 0)
                if marker.pose.pose.position.x == 0.4:
                    move(0, 0, 0, 0)
                    olek = DOCK
         
        elif olek == DOCK: 
            move(0.3, 0, 0, 0.3)    
            if leitud_markeri_ID == otsitav:           
                move(1, 0, 0, 3.14)
                move(0.1, -0.1, 0, 0)
                olek = CHARGE
        
        elif olek == CHARGE:
            move(0, 0, 0, 0)
            rospy.sleep(10)
            olek = UNDOCK
        
        elif olek == UNDOCK:
            move(1, 1, 0, 0) 
            olek = IDLE
                  	
if __name__ =='__main__':
    try:  
        main()
    except rospy.ROSInterruptException:
        pass
