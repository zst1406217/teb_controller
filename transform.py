from tf.transformations import euler_from_quaternion
import rospy
from gazebo_msgs.srv import GetModelState
from std_msgs.msg import Float32

_model_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)

def get_model_state():
    rospy.wait_for_service("/gazebo/get_model_state")
    try:
        return _model_state('jackal', 'world')
    except (rospy.ServiceException):
        rospy.logwarn("/gazebo/get_model_state service call failed")


rospy.init_node("transform_euler", anonymous=True)
theta_publisher = rospy.Publisher('/theta', Float32, queue_size=1)

r = rospy.Rate(50) # define rate here

while not rospy.is_shutdown():

    ori= get_model_state().pose.orientation
    eul=euler_from_quaternion([ori.x,ori.y,ori.z,ori.w])
    # print(eul[2])
    msg=Float32()
    msg.data=eul[2]
    theta_publisher.publish(msg)
    r.sleep()
