import rospy
import math
from hiwonder_servo_msgs.msg import MultiRawIdPosDur
from hiwonder_servo_controllers.bus_servo_control import set_servos
from kinematics.search_kinematics_solutions import SearchKinematicsSolutions


rospy.init_node('servo_test')
joints_pub = rospy.Publisher('servo_controllers/port_id_1/multi_id_pos_dur', MultiRawIdPosDur, queue_size=1)
rospy.sleep(0.2) ### without this sleep the while loop does not start
## calculate joint positions

while not rospy.is_shutdown():
    try:
        if rospy.get_param('/hiwonder_servo_manager/running') and rospy.get_param('/joint_states_publisher/running'):
            break
    except:
        rospy.sleep(0.1)
solution_searcher = SearchKinematicsSolutions()
res = SearchKinematicsSolutions.solveIK((0, 0.2, 0.4), math.radians(10), math.radians(-90), math.radians(90))
joint_data = res[1]
## convert to int 
for key,value in joint_data.items():
    print(joint_data[key],"=>", int(joint_data[key]))
    joint_data[key] = int(joint_data[key])

print("joint_data:", joint_data)
## set servos
ret = set_servos(joints_pub, 1000, ((1, 100), (2, joint_data['joint4']), (3, joint_data['joint3']), (4, joint_data['joint2']), (5, joint_data['joint1'])))
print(ret)



