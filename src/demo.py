#!/usr/bin/env python
# Convert VR tracker pose into Kinova robot end effector pose.
# Convert force feedback glove finger positions into robot finger positions.
# Detect gripping with robot end effector and apply force on glove using servos.
import tf
import rospy
from kinova_msgs.msg import FingerPosition, SetFingersPositionActionGoal, ArmPoseActionGoal
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from tf2_msgs.msg import TFMessage
from std_msgs.msg import Header, Int16MultiArray
import math


def generic_tracker_callback(data):
    '''
    VR tracker position/orientation callback.
    '''
    global generic_tracker_position
    global generic_tracker_orientation
    g_id = 4

    generic_tracker_position = [data.transforms[g_id].transform.translation.x,
                                data.transforms[g_id].transform.translation.y,
                                data.transforms[g_id].transform.translation.z]
    
    generic_tracker_orientation = [data.transforms[g_id].transform.rotation.x,
                                   data.transforms[g_id].transform.rotation.y,
                                   data.transforms[g_id].transform.rotation.z,
                                   data.transforms[g_id].transform.rotation.w]


def pose_callback(data):
    '''
    Kinova robot current position/orientation callback.
    '''
    global robot_position
    global robot_orientation

    robot_position = [data.pose.position.x, data.pose.position.y, data.pose.position.z]
    robot_orientation = [data.pose.orientation.x,
                         data.pose.orientation.y,
                         data.pose.orientation.z,
                         data.pose.orientation.w]


def finger_callback(data):
    '''
    Finger position callback. Robot finger updates will suppress arm movement, so finger positions will only be updated if there's a signifcant change.
    Flex sensors must pass thresholds before robot fingers will move and robot fingers will stop at flex sensor value when reached.
    No pressure sensors on the end effector, so grips are detected when the fingers don't move significantly over the course of the COUNTER_THRESHOLD.
    Servos updated if grip status changes.
    '''
    # Gripping constants
    OPENING_THRESHOLD = 4866 # Triggers fingers closed->open
    CLOSING_THRESHOLD = 2433 # Triggers fingers open->closed
    OPEN_POS = 0 # Robot open finger (range: 10-4866)
    CLOSED_POS = 7300 # Robot closed finger (range: 2433-7300)
    GRIPPING_THRESHOLD = 475 # Minimum change before grip status
    DESTINATION_THRESHOLD = 550 # Minimum change for goal
    COUNTER_THRESHOLD = 3 # Time before grip status changes

    # Servo constants
    SERVO_THUMB_OPEN = 150
    SERVO_THUMB_CLOSED = 0
    SERVO_INDEX_OPEN = 150
    SERVO_INDEX_CLOSED = 0
    SERVO_MIDDLE_OPEN = 30
    SERVO_MIDDLE_CLOSED = 180
    
    global finger_positions
    global fingers_grasping
    global finger_state
    global finger_holding_positions
    global finger_counters
    global switch_servo_state
    global finger_instructions
    global fingers_moving

    change = False
    gripping = False

    servos_open = [SERVO_THUMB_OPEN, SERVO_INDEX_OPEN, SERVO_MIDDLE_OPEN]
    servos_closed = [SERVO_THUMB_CLOSED, SERVO_INDEX_CLOSED, SERVO_MIDDLE_CLOSED]

    finger_positions = [data.finger1, data.finger2, data.finger3]
    
    # Figure out which state each finger is in
    for i in range(3):
        if(finger_state[i] == 'open_range'):
            if(finger_goals[i] > OPENING_THRESHOLD):
                finger_state[i] = 'closing'
                finger_instructions[i] = CLOSED_POS
                finger_holding_positions[i] = finger_positions[i]
                change = True
        elif(finger_state[i] == 'closed_range'):
            if(finger_goals[i] < CLOSING_THRESHOLD):
                finger_state[i] = 'opening'
                finger_instructions[i] = OPEN_POS
                finger_holding_positions[i] = finger_positions[i]
                change = True
        elif(finger_state[i] == 'closing'):
            if(abs(finger_goals[i] - finger_positions[i]) < DESTINATION_THRESHOLD):
                finger_state[i] = 'closed_range'
                finger_instructions[i] = finger_positions[i]
                finger_counters[i] = 0
                change = True
            elif(abs(finger_holding_positions[i] - finger_positions[i]) < GRIPPING_THRESHOLD):
                finger_counters[i] += 1
                if(finger_counters[i] > COUNTER_THRESHOLD):
                    finger_state[i] = 'gripping'        
            elif(abs(finger_holding_positions[i] - finger_positions[i]) > GRIPPING_THRESHOLD):
                finger_holding_positions[i] = finger_positions[i]
                finger_counters[i] = 0
        elif(finger_state[i] == 'opening'):
            if(abs(finger_goals[i] - finger_positions[i]) < DESTINATION_THRESHOLD):
                finger_state[i] = 'open_range'
                finger_instructions[i] = finger_positions[i]
                finger_counters[i] = 0
                change = True
        elif(finger_state[i] == 'gripping'):
            finger_instructions[i] = finger_positions[i]
            finger_counters[i] = 0
            if(finger_goals[i] < CLOSING_THRESHOLD):
                finger_state[i] = 'opening'
                finger_instructions[i] = OPEN_POS
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

    # Check if any finger is gripping
    for i in range(3):
        if(finger_state[i] == 'gripping'):
            gripping = True
    
    # Move the servos if gripping state changing
    if(gripping):
        if(switch_servo_state is False):
            set_servos(servos_open, servo_pub)
            switch_servo_state = True
    else:
        if(switch_servo_state):
            switch_servo_state = False
            set_servos(servos_closed, servo_pub)


def glove_callback(flex_sensors):
    '''
    Flex sensor callback from glove. Values are mapped to robot finger range. Tune the min/max voltages.
    '''
    # Min/max values
    THUMB_MAX = 180.0
    THUMB_MIN = 90.0
    INDEX_MAX = 260.0
    INDEX_MIN = 140.0
    MIDDLE_MAX = 420.0
    MIDDLE_MIN = 270.0
    ROBOT_MAX = 7300.0
    ROBOT_MIN = 10.0
    
    global finger_goals # Finger positions from the Arduino (eventually scaled)

    finger_goals = [flex_sensors.data[0], flex_sensors.data[1], flex_sensors.data[2]]

    # Thumb
    if flex_sensors.data[0] > THUMB_MAX:
        finger_goals[0] = THUMB_MAX
    elif flex_sensors.data[0] < THUMB_MIN:
        finger_goals[0] = THUMB_MIN
    # Index finger
    if flex_sensors.data[1] > INDEX_MAX:
        finger_goals[1] = INDEX_MAX
    elif flex_sensors.data[1] < INDEX_MIN:
        finger_goals[1] = INDEX_MIN
    # Middle finger
    if flex_sensors.data[2] > MIDDLE_MAX:
        finger_goals[2] = MIDDLE_MAX
    elif flex_sensors.data[2] < MIDDLE_MIN:
        finger_goals[2] = MIDDLE_MIN

    # Map flex sensor values to robot range
    finger_goals[0] = abs((1 - ((finger_goals[0] - THUMB_MIN) / (THUMB_MAX - THUMB_MIN))) * ROBOT_MAX) + ROBOT_MIN
    finger_goals[1] = abs((1 - ((finger_goals[1] - INDEX_MIN) / (INDEX_MAX - INDEX_MIN))) * ROBOT_MAX) + ROBOT_MIN
    finger_goals[2] = abs((1 - ((finger_goals[2] - MIDDLE_MIN) / (MIDDLE_MAX - MIDDLE_MIN))) * ROBOT_MAX) + ROBOT_MIN


def set_pose(position_goal, orientation_goal, publisher):
    '''
    Set end effector pose goal. ([x, y, z], [x, y, z, w], pub)
    '''
    pose_msg = ArmPoseActionGoal()
    pose_msg.header = Header(frame_id='j2s7s300_link_base')
    pose_msg.goal_id.stamp = rospy.get_rostime()
    pose_msg.goal_id.id = str(rospy.get_rostime())
    pose_msg.goal.pose.header = Header(frame_id='j2s7s300_link_base')
    pose_msg.goal.pose.pose.position = Point(x=position_goal[0],
                                             y=position_goal[1],
                                             z=position_goal[2])
    pose_msg.goal.pose.pose.orientation = Quaternion(x=orientation_goal[0],
                                                     y=orientation_goal[1],
                                                     z=orientation_goal[2],
                                                     w=orientation_goal[3])
    publisher.publish(pose_msg)


def set_fingers(finger_goals, publisher):
    '''
    Set fingers goal (10.0-7300.0/open-closed). ([a, b, c], pub)
    '''
    finger_msg = SetFingersPositionActionGoal()
    finger_msg.goal.fingers.finger1 = float(finger_goals[0])
    finger_msg.goal.fingers.finger2 = float(finger_goals[1])
    finger_msg.goal.fingers.finger3 = float(finger_goals[2])
    publisher.publish(finger_msg)


def set_servos(servo_positions, publisher):
    '''
    Set servo positions (0.0-180.0/closed-open). ([a, b], pub)
    '''
    servo_msg = Int16MultiArray()
    servo_msg.data = [int(servo_positions[0]), int(servo_positions[1])] # [Thumb, Fingers]
    publisher.publish(servo_msg)


def distance(p_1, p_2):
    return math.sqrt( (p_1[0]-p_2[0])**2 + (p_1[1]-p_2[1])**2 + (p_1[2]-p_2[2])**2 ) 


if __name__ == '__main__':
    generic_tracker_initialized = False
    robot_initialized = False
    
    INITIAL_POS = [0.0, 0.0, 0.5]
    INITIAL_ORI = [0.0, 0.0, 0.0, 0.0] # Not a valid quaternion

    initial_orientation = [0.0, 0.0, 0.0, 0.0]

    initial_robot_position = [0.0, 0.0, 0.5]
    initial_robot_orientation = [0.0, 0.0, 0.0, 0.0]

    generic_tracker_position = [0.0, 0.0, 0.5]
    generic_tracker_orientation = [0.0, 0.0, 0.0, 0.0]
    
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
    switch_servo_state = False

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
        rospy.Subscriber("/glove", Int16MultiArray, glove_callback)
        rospy.Subscriber('/tf', TFMessage, generic_tracker_callback)

        while not rospy.is_shutdown():  
            # Check if generic tracker is working
            if(not(generic_tracker_position == INITIAL_POS) and generic_tracker_initialized is False):
                initial_position = generic_tracker_position
                initial_orientation = generic_tracker_orientation
                generic_tracker_initialized = True
                print('Generic tracker is good!')

            # Check if the robot is working
            if(not(robot_position == INITIAL_POS) and robot_initialized is False):
                initial_robot_position = robot_position
                initial_robot_orientation = robot_orientation
                altered_initial_orientation = initial_orientation
                altered_initial_orientation[3] = -altered_initial_orientation[3]
                correction_orientation = tf.transformations.quaternion_multiply(initial_robot_orientation, altered_initial_orientation)
                robot_initialized = True
                print("Robot is good!")

            # If all of the sensors are good and the fingers aren't moving, update the end effector pose
            if(not(robot_position == INITIAL_POS) and generic_tracker_initialized and robot_initialized):
                print('Running.')
                
                # Add the scaled VR tracker position to the robot position
                goal_position = [1.1*(a - b) + c for a, b, c in zip(generic_tracker_position, initial_position, initial_robot_position)]
                
                # Rotate the VR tracker orientation if it didn't initially match the robot base frame
                goal_orientation = tf.transformations.quaternion_multiply(correction_orientation, generic_tracker_orientation)
                
                # Update robot if the fingers aren't moving.
                # To appear smoother, ROS Rate is slow when far from goal, fast when near, and passed when very close.
                if(not(fingers_moving)):
                    if distance(robot_position, goal_position) < 0.01:
                        pass
                        # print("Stop mode | Distance: ", distance(robot_position, goal_position))
                    elif distance(robot_position, goal_position) < 0.040:
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
