#!/usr/bin/env python
# license removed for brevity
import tf
import rospy
from kinova_msgs.msg import FingerPosition, SetFingersPositionActionGoal, ArmPoseActionGoal
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from tf2_msgs.msg import TFMessage
from std_msgs.msg import Header, Int16MultiArray
import math


def generic_tracker_callback(data):
    global generic_tracker_position
    global generic_tracker_orientation
    g_id = 4

    generic_tracker_position = [data.transforms[g_id].transform.translation.y, -data.transforms[g_id].transform.translation.x, data.transforms[g_id].transform.translation.z]
    generic_tracker_orientation = [data.transforms[g_id].transform.rotation.x, data.transforms[g_id].transform.rotation.y, data.transforms[g_id].transform.rotation.z, data.transforms[g_id].transform.rotation.w]
    generic_tracker_orientation = tf.transformations.quaternion_multiply(generic_tracker_orientation, [0.5,0.5,0.5,0.5])


def pose_callback(data):
    global robot_position
    global robot_orientation

    robot_position = [data.pose.position.x, data.pose.position.y, data.pose.position.z]
    robot_orientation = [data.pose.orientation.x, data.pose.orientation.y, data.pose.orientation.z, data.pose.orientation.w]


def finger_callback(data):
    global finger_positions
    global fingers_grasping
    global finger_state
    global finger_holding_positions
    global finger_counters
    global switch
    global finger_instructions
    global fingers_moving

    change = False

    gripping_threshold = 475
    destination_threshold = 550
    counter_threshold = 3

    thumb_open = 150
    index_open = 150
    middle_open = 30
    servos_open = [thumb_open, index_open, middle_open]
    thumb_closed = 0
    index_closed = 0
    middle_closed = 180
    servos_closed = [thumb_closed, index_closed, middle_closed]
    
    servo_instructions = servos_closed

    finger_positions = [data.finger1, data.finger2, data.finger3]

    open_pos = 0 # Open finger range is 0-4866
    closed_pos = 7300 # Closed finger range is 2430-7300
    
    # Figure out which state each finger is in
    for i in range(3):
        if(finger_state[i] == 'open_range'):
            if(finger_goals[i] > 4866):
                finger_state[i] = 'closing'
                finger_instructions[i] = closed_pos
                finger_holding_positions[i] = finger_positions[i]
                change = True
        elif(finger_state[i] == 'closed_range'):
            if(finger_goals[i] < 2433):
                finger_state[i] = 'opening'
                finger_instructions[i] = open_pos
                finger_holding_positions[i] = finger_positions[i]
                change = True
        elif(finger_state[i] == 'closing'):
            if(abs(finger_goals[i] - finger_positions[i]) < destination_threshold):
                finger_state[i] = 'closed_range'
                finger_instructions[i] = finger_positions[i]
                finger_counters[i] = 0
                change = True
            elif(abs(finger_holding_positions[i] - finger_positions[i]) < gripping_threshold):
                finger_counters[i] += 1
                if(finger_counters[i] > counter_threshold):
                    finger_state[i] = 'gripping'        
            elif(abs(finger_holding_positions[i] - finger_positions[i]) > gripping_threshold):
                finger_holding_positions[i] = finger_positions[i]
                finger_counters[i] = 0
        elif(finger_state[i] == 'opening'):
            if(abs(finger_goals[i] - finger_positions[i]) < destination_threshold):
                finger_state[i] = 'open_range'
                finger_instructions[i] = finger_positions[i]
                finger_counters[i] = 0
                change = True
        elif(finger_state[i] == 'gripping'):
            finger_instructions[i] = finger_positions[i]
            finger_counters[i] = 0
            if(finger_goals[i] < 2433):
                finger_state[i] = 'opening'
                finger_instructions[i] = open_pos
                finger_holding_positions[i] = finger_positions[i]
                change = True

    # Update finger position only if there is a significant change 
    if(change):
        set_fingers(finger_instructions, finger_pub)

    # Figure out if the fingers are currently moving so that arm changes can be suppressed
    for i in range(3):
        if(finger_state[i] == 'opening' or finger_state[i] == 'closing'):
            fingers_moving = True
        else:
            fingers_moving = False
    
    gripping = False

    # Check if any finger is gripping
    for i in range(3):
        if(finger_state[i] == 'gripping'):
            gripping = True
    
    # Move the servos if gripping state changing
    if(gripping):
        if(switch is False):
            set_servos(servos_open, servo_pub)
            switch = True
    else:
        if(switch):
            switch = False
            set_servos(servos_closed, servo_pub)


def glove_callback(flex_sensors):
    global finger_goals # Finger positions from the Arduino (eventually scaled)
    global finger_state # What each finger is doing at the time
    global fingers_grasping # Bool stating whether the finger is currently grasping

    finger_goals = [flex_sensors.data[0], flex_sensors.data[1], flex_sensors.data[2]]

    # Thumb
    if flex_sensors.data[0] > 180.0:
        finger_goals[0] = 180.0
    elif flex_sensors.data[0] < 90.0:
        finger_goals[0] = 90.0
    # Index finger
    if flex_sensors.data[1] > 260.0:
        finger_goals[1] = 260.0
    elif flex_sensors.data[1] < 140.0:
        finger_goals[1] = 140.0
    # Middle finger
    if flex_sensors.data[2] > 420.0:
        finger_goals[2] = 420.0
    elif flex_sensors.data[2] < 270.0:
        finger_goals[2] = 270.0

    # Map flex sensor values to robot range
    finger_goals[0] = abs((1 - ((finger_goals[0] - 90.0) / (180.0 - 90.0))) * 7300.0) + 10.0
    finger_goals[1] = abs((1 - ((finger_goals[1] - 140.0) / (260.0 - 140.0))) * 7300.0) + 10.0
    finger_goals[2] = abs((1 - ((finger_goals[2] - 270.0) / (420.0 - 270.0))) * 7300.0) + 10.0


def set_pose(position_goal, orientation_goal, publisher):
    pose_msg = ArmPoseActionGoal()
    pose_msg.header = Header(frame_id='j2s7s300_link_base')
    pose_msg.goal_id.stamp = rospy.get_rostime()
    pose_msg.goal_id.id = str(rospy.get_rostime())
    pose_msg.goal.pose.header = Header(frame_id='j2s7s300_link_base')
    pose_msg.goal.pose.pose.position = Point(x=position_goal[0], y=position_goal[1], z=position_goal[2])
    pose_msg.goal.pose.pose.orientation = Quaternion(x=orientation_goal[0], y=orientation_goal[1], z=orientation_goal[2], w=orientation_goal[3])
    publisher.publish(pose_msg)


def set_fingers(finger_goals, publisher):
    finger_msg = SetFingersPositionActionGoal()
    finger_msg.goal.fingers.finger1 = float(finger_goals[0])
    finger_msg.goal.fingers.finger2 = float(finger_goals[1])
    finger_msg.goal.fingers.finger3 = float(finger_goals[2])
    publisher.publish(finger_msg)


def set_servos(servo_positions, publisher):
    servo_msg = Int16MultiArray()
    servo_msg.data = [int(servo_positions[0]), int(servo_positions[1])] # [Thumb, Fingers]
    publisher.publish(servo_msg)


def distance(p_1, p_2):
    return math.sqrt( (p_1[0]-p_2[0])**2 + (p_1[1]-p_2[1])**2 + (p_1[2]-p_2[2])**2 ) 


if __name__ == '__main__':
    generic_tracker_initialized = False
    robot_initialized = False
    
    initial_orientation = [0.0, 0.0, 0.0, 0.0]

    generic_tracker_position = [0.0, 0.0, 0.5]
    generic_tracker_orientation = [0.0, 0.0, 0.0, 0.0]
    
    initial_robot_position = [0.0, 0.0, 0.5]
    initial_robot_orientation = [0.0, 0.0, 0.0, 0.0]
    
    robot_position = [0.0, 0.0, 0.5]
    robot_orientation = [0.0, 0.0, 0.0, 0.0]
    
    correction_orientation = [0.0, 0.0, 0.0, 0.0]

    finger_goals = [0, 0, 0]
    finger_state = [0, 0, 0]
    finger_counters = [0, 0, 0]
    fingers_grasping = [False, False, False]
    finger_holding_positions = [0.0, 0.0, 0.0]
    finger_positions = [0.0, 0.0, 0.0]

    finger_goals = [0, 0, 0]
    finger_state = ['open_range', 'open_range', 'open_range']
    finger_counters = [0, 0, 0]
    fingers_grasping = [False, False, False]
    finger_holding_positions = [0.0, 0.0, 0.0]
    finger_positions = [0.0, 0.0, 0.0]
    finger_instructions = [0.0, 0.0, 0.0]
    fingers_moving = False
    switch = False

    rospy.init_node('demo', anonymous=True)
    rate = rospy.Rate(5)

    try:
        # Publishers
        pose_pub = rospy.Publisher('/kinova_arm_driver/pose_action/tool_pose/goal', ArmPoseActionGoal, queue_size=10)
        finger_pub = rospy.Publisher('/kinova_arm_driver/fingers_action/finger_positions/goal', SetFingersPositionActionGoal, queue_size=10)
        servo_pub = rospy.Publisher('/servo_angles', Int16MultiArray, queue_size=10)

        # Subscribers
        rospy.Subscriber("/kinova_arm_driver/out/tool_pose", PoseStamped, pose_callback)
        rospy.Subscriber("/kinova_arm_driver/out/finger_position", FingerPosition, finger_callback)
        rospy.Subscriber("/thanos", Int16MultiArray, glove_callback)
        rospy.Subscriber('/tf', TFMessage, generic_tracker_callback)

        while not rospy.is_shutdown():  

            print(generic_tracker_position, generic_tracker_orientation, generic_tracker_initialized)
            # Check if generic tracker is working
            if(not(generic_tracker_position == [0.0, 0.0, 0.5]) and generic_tracker_initialized is False):
                initial_position = generic_tracker_position
                initial_orientation = generic_tracker_orientation
                generic_tracker_initialized = True
                print('Generic tracker is good!')

            # Check if the robot is working
            if(not(robot_position == [0.0, 0.0, 0.5]) and robot_initialized is False):
                initial_robot_position = robot_position
                initial_robot_orientation = robot_orientation
                altered_initial_orientation = initial_orientation
                altered_initial_orientation[3] = -altered_initial_orientation[3]
                correction_orientation = tf.transformations.quaternion_multiply(initial_robot_orientation, altered_initial_orientation)
                robot_initialized = True
                print("Robot is good!")

            # If all of the sensors are good and the fingers aren't moving, update the end effector pose
            if(not(robot_position == [0.0, 0.0, 0.5]) and generic_tracker_initialized and robot_initialized):
                print('Running.')
                goal_position = [1.1*(a - b) + c for a, b, c in zip(generic_tracker_position, initial_position, initial_robot_position)]
                
                goal_orientation = tf.transformations.quaternion_multiply(correction_orientation, generic_tracker_orientation)
                
                if(not(fingers_moving)):
                    if distance(robot_position, goal_position) < 0.01:
                        pass
                        # print("Stop mode | Distance: ", distance(robot_position, goal_position))
                    elif distance(robot_position, goal_position) < 0.040:
                        # goal_position = [1.5*(a - b) + c for a, b, c in zip(generic_tracker_position, initial_position, initial_robot_position)]
                        rate = rospy.Rate(10)
                        set_pose(goal_position, goal_orientation, pose_pub)
                        # print("Detail mode | Distance: ", distance(robot_position, goal_position))
                    else:
                        rate = rospy.Rate(3)
                        set_pose(goal_position, goal_orientation, pose_pub)
                        # print("Wide mode | Distance: ", distance(robot_position, goal_position))

            rate.sleep()

    except rospy.ROSInterruptException:
        pass
