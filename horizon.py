import rospy
from std_msgs.msg import Bool
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose
from nav_msgs.msg import Path
import numpy as np
from gazebo_msgs.srv import GetModelState
import time

horizon_lookahead=8.0 # horizon lookahead of the global path
max_deviation=0.2 # max deviation of the current position to the global path
free_threshold=50 # free threshold of costmap2d [0,100]

_model_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
INIT_POSITION = [-2, 2, 1.57]  # in world frame
map = np.zeros(80000)
path = Path().poses
resolution = 0.05
width = 800
height = 800
origin_x = -20
origin_y = -20


def compute_distance(x1, x2, y1, y2):
    return ((x1 - x2) ** 2 + (y1 - y2) ** 2) ** 0.5


def costmap_callback(data):
    global map
    map = np.array(data.data)
    # map=map.reshape(400,200) # reshape(height,width)  cost=map[iy][ix]
    # print(map.shape)
    

def path_callback(data):
    global path
    path=data.poses
    print(len(path))
    

def get_model_state():
  rospy.wait_for_service("/gazebo/get_model_state")
  try:
      return _model_state('jackal', 'world')
  except (rospy.ServiceException):
      rospy.logwarn("/gazebo/get_model_state service call failed")

def main():
    global map,path
    rospy.init_node("horizon_publisher", anonymous=True)
    horizon_publisher = rospy.Publisher('/move_base_simple/horizon', Bool, queue_size=1)
    rospy.Subscriber('/move_base/global_costmap/costmap', OccupancyGrid, costmap_callback, queue_size = 1)
    rospy.Subscriber('/move_base/GlobalPlanner/plan', Path, path_callback, queue_size = 1)
    
    r = rospy.Rate(20) # define rate here
    # first plan
    
    while not rospy.is_shutdown():
        
        replan_flag=False
        deviation_flag=False
        pos = get_model_state().pose.position
        x1,y1=pos.x-INIT_POSITION[0],pos.y-INIT_POSITION[1]
        
        path_last=path.copy()
        path_length=len(path_last)
        for i in range(path_length):
            pos_path=path_last[i].pose.position
            x2,y2=pos_path.x,pos_path.y
            # print(compute_distance(x1,x2,y1,y2))
            if not replan_flag:
                if compute_distance(x1,x2,y1,y2)<max_deviation:
                    replan_flag=True
                    deviation_flag=True
                    
            else:
                if compute_distance(x1,x2,y1,y2)<horizon_lookahead:
                    delta_x=x2-origin_x
                    delta_y=y2-origin_y
                    ix=round(delta_x/resolution)
                    iy=round(delta_y/resolution)
                    idx=ix+iy*width
                    # print(delta_x,delta_y,idx)
                    if map[idx]>free_threshold:
                        msg=Bool()
                        msg.data=True
                        horizon_publisher.publish(msg)
                        print("horizon replan")
                        time.sleep(0.5)
                        break
            
        if not deviation_flag and path_length>0:
            msg=Bool()
            msg.data=True
            horizon_publisher.publish(msg)
            print("deviation replan")
            time.sleep(0.5)
        
        r.sleep()
    
        
        

if __name__ == '__main__': 
    
    main()
