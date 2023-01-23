#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from ar_track_alvar_msgs.msg import AlvarMarkers
from std_srvs.srv import Empty, EmptyResponse
from math import atan2

DOCK_MARKER_ID = 1 # AR marker ID that will be searched for

IDLE = 0
APPROACH_MARKER = 1
DOCK = 2
CHARGE = 3
UNDOCK = 4
     
def ar_message_handler(data):
    global latest_marker
	
    latest_marker = None
    if len(data.markers) > 0:
        for marker in data.markers:
            rospy.loginfo("Detected marker with ID " + str(marker.id))
            if marker.id == DOCK_MARKER_ID:
                latest_marker = marker # Correct marker was detected
            rospy.loginfo("y coordinate: " + str(marker.pose.pose.position.y))
    else:
        rospy.loginfo("No AR markers detected.")

def dock_cb(req):
    global state
    if state == IDLE:
        state = APPROACH_MARKER
    return EmptyResponse()

def undock_cb(req):
    global state
    if state == CHARGE:
        state = UNDOCK
    return EmptyResponse()
		
def main():
    global state
    state = IDLE
    
    global latest_marker
    latest_marker = None

    rospy.init_node("ar_subscriber")
    ar_sub = rospy.Subscriber("ar_pose_marker", AlvarMarkers, ar_message_handler)
    empty_service = rospy.Service("dock", Empty, dock_cb)
    empty_service = rospy.Service("undock", Empty, undock_cb)
   
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
	
    while not rospy.is_shutdown():

        rospy.loginfo_throttle(1, "state: %d" % state)
	
        if state == IDLE:
            move(0.5, 0, 0, 0) # stop
            
        elif state == APPROACH_MARKER: 
            robot_vel = Twist()
            
            if latest_marker == DOCK_MARKER_ID:
                # Marker detected, use shorter naming for convenience
                m_pos = latest_marker.pose.pose.position
                
                # Marker detected, check if we are close enough
                if m_pos.x <= 0.4:
                    # advance to the DOCK state, zero twist will stop the robot
                    state = DOCK
                else:
                    # Marker detected too far, let's get closer
                    robot_vel.linear.x = 0.2

                    # Rotate towards the marker by calculating the angle first
                    angle = atan2(m_pos.y, m_pos.x)
                    robot_vel.angular.z = angle
                    #if m_pos.y > 0:
                    #    # Marker found on the right side -> turn right
                    #    robot_vel.angular.z = -0.3 
                    #else:
                    #    # Marker has to be on the left side -> turn left
                    #    robot_vel.angular.z = 0.3 
                    
            else:
                # Marker not found, do something slowly to detect it
                robot_vel.angular.z = .3 

            velocity_pub.publish(robot_vel)

         
        elif state == DOCK: 
            # Do blind movements to dock the robot
            move(4, 0, 0, 3.14/4) # turn around in 4 sec
            move(1, -0.1, 0, 0) # backup
            move(0.5, 0, 0, 0) # stop
            state = CHARGE
        
        elif state == CHARGE:
            # Fakes charging for now
            rospy.sleep(10)
            state = UNDOCK
        
        elif state == UNDOCK:
            # Do bind movements to undock the robot
            move(1, 0.4, 0, 0) 
            move(0.5, 0, 0, 0) # stop
            state = IDLE
                  	
if __name__ =='__main__':
    try:  
        main()
    except rospy.ROSInterruptException:
        pass
