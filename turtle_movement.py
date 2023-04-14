import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose



def move(velocity_publisher, speed, distance, is_forward):

    velocity_message = Twist()   #declaring a Twist message, same as ur Ureca

    global x,y
    x0 = x
    y0 = y
    distance_moved = 0.0

    loop_rate = rospy.Rate(10)

    if (is_forward):
        velocity_message.linear.x = abs(speed)
    else:
        velocity_message.linear.x = -abs(speed)
    
    while True:
        rospy.loginfo("Turtlesim moves forward")
        velocity_publisher.publish(velocity_message)
        loop_rate.sleep()
        distance_moved = abs(math.sqrt(((x-x0)**2) + ((y-y0)**2)))

        print(distance_moved)
        if not (distance_moved<distance):
            rospy.loginfo("reached")
            break

    velocity_message.linear.x = 0
    velocity_publisher.publish(velocity_message)

def poseCallback(pose_messasge):
    
    global x
    global y,yaw
    x = pose_messasge.x
    y = pose_messasge.y
    yaw = pose_messasge.theta

if __name__ == 'main':
    try:
        rospy.init_node('turtlesim_motion_pose', anonymous=True)
        pose_messasge = Pose()


        #declare velocity publisher
        cmd_vel_topic = 'turtle1/cmd_vel'
        velocity_publisher = rospy.Publisher(cmd_vel_topic, Twist, queue_size=10)

        position_topic = 'turtle1/pose'
        pose_subscriber = rospy.Subscriber(position_topic, Pose, poseCallback)
        time.sleep(2)

        move(velocity_publisher, 1.0, 4.0, True)

    except rospy.ROSInterruptException:
        rospy.loginfo("node terminated.")
        



    
