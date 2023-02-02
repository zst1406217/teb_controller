#!/usr/bin/env python

# This small script subscribes to the FeedbackMsg message of teb_local_planner
# and plots the current velocity.
# publish_feedback must be turned on such that the planner publishes this information.
# Author: christoph.roesmann@tu-dortmund.de

import rospy, math
from teb_local_planner.msg import FeedbackMsg, TrajectoryMsg, TrajectoryPointMsg
from geometry_msgs.msg import PolygonStamped, Point32, Twist
from rospy.rostime import Time
import numpy as np
from gazebo_msgs.srv import SetModelState, GetModelState
from scipy import interpolate
import matplotlib.pyplot as plt
import transforms3d


look_ahead=0.2
f_1=0.25 # Feedforward control coefficient
f_2=0.25
Kv=3.75 # Feedback control coefficient
Kw=100
lambda1=1 # dy coefficient
lambda2=2 # dtheta coefficient
max_v=1.0
max_w=1.0

INIT_POSITION = [-2, 3, 1.57]  # in world frame


stamp=Time(0)
_model_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)

def feedback_callback(data):
  global trajectory
  global stamp

  if not data.trajectories: # empty
    trajectory = []
    stamp=Time(0)
    return
  trajectory = data.trajectories[data.selected_trajectory_idx].trajectory
  stamp = data.trajectories[data.selected_trajectory_idx].header.stamp

  
def plot_velocity_profile(fig, ax_v, ax_omega, t, v, omega):
  ax_v.cla()
  ax_v.grid()
  ax_v.set_ylabel('Trans. velocity [m/s]')
  ax_v.plot(t, v, '-bx')
  ax_omega.cla()
  ax_omega.grid()
  ax_omega.set_ylabel('Rot. velocity [rad/s]')
  ax_omega.set_xlabel('Time [s]')
  ax_omega.plot(t, omega, '-bx')
  fig.canvas.draw()

  
def get_model_state():
  rospy.wait_for_service("/gazebo/get_model_state")
  try:
      return _model_state('jackal', 'world')
  except (rospy.ServiceException):
      rospy.logwarn("/gazebo/get_model_state service call failed")
  
  
def velocity_plotter():
  global trajectory
  global stamp
  global f_1
  global f_2
  rospy.init_node("visualize_velocity_profile", anonymous=True)
  
  topic_name = "/move_base/TebLocalPlannerROS/teb_feedback"
  rospy.Subscriber(topic_name, FeedbackMsg, feedback_callback, queue_size = 1) # define feedback topic here!
  velocity_publisher = rospy.Publisher('/teb_controller/cmd_vel', Twist, queue_size=1)

  rospy.loginfo("Visualizing velocity profile published on '%s'.",topic_name) 
  rospy.loginfo("Make sure to enable rosparam 'publish_feedback' in the teb_local_planner.")

  r = rospy.Rate(100) # define rate here
  while not rospy.is_shutdown():
    x = []
    y = []
    t = []
    v = []
    theta = []
    omega = []  

    for point in trajectory:
      x.append(point.pose.position.x)
      y.append(point.pose.position.y)
      mat1=transforms3d.quaternions.quat2mat([point.pose.orientation.x,point.pose.orientation.y,point.pose.orientation.z,point.pose.orientation.w])
      theta.append(transforms3d.euler.mat2euler(mat1)[0])
      v.append(point.velocity.linear.x)
      omega.append(point.velocity.angular.z)
      t.append(point.time_from_start.to_sec())

    pos = get_model_state().pose
    vel = get_model_state().twist
    # print(vel.velocity)
    vel_msg = Twist()
    vel_msg.linear.x = 0
    vel_msg.angular.z = 0
    
    
    if len(t)>0:
      try:
        xck = interpolate.splrep(t, x)
        yck = interpolate.splrep(t, y)
        vck = interpolate.splrep(t, v)
        thetack = interpolate.splrep(t, theta)
        omegack = interpolate.splrep(t, omega)
        mat=transforms3d.quaternions.quat2mat([pos.orientation.x,pos.orientation.y,pos.orientation.z,pos.orientation.w])
        euler=transforms3d.euler.mat2euler(mat)[0]
        time_from_start = rospy.get_rostime().to_sec()-stamp.to_sec()
        time_ref=time_from_start+look_ahead
        x_n_t=interpolate.splev(time_ref, xck, der=0) # Refer to the init pose
        y_n_t=interpolate.splev(time_ref, yck, der=0) # Refer to the init pose
        v_n_t=interpolate.splev(time_ref, vck, der=0)
        theta_n_t=interpolate.splev(time_ref, thetack, der=0)
        omega_n_t=interpolate.splev(time_ref, omegack, der=0)
      except:
        x_n_t=x[1]
        y_n_t=y[1]
        v_n_t=v[1]
        theta_n_t=theta[1]
        omega_n_t=omega[1]
      x0=pos.position.x-INIT_POSITION[0] # Both coordinate take the initial point as the origin
      y0=pos.position.y-INIT_POSITION[1] # Right is positive x, forward is positive y
      theta0=euler
      if y_n_t>y0:
        fai = np.arctan((y_n_t-y0)/(x_n_t-x0))
        if fai<0:
          fai=fai+np.pi
      else:
        fai = np.arctan((y_n_t-y0)/(x_n_t-x0))
        if fai>0:
          fai=fai-np.pi
      alpha=fai-theta0
      if np.abs(alpha)>np.pi/2:
        f_1=1
        f_2=2
      ld=((y_n_t-y0)**2+(x_n_t-x0)**2)**0.5
      dx=ld*np.cos(alpha)
      dy=ld*np.sin(alpha)
      dtheta=theta_n_t-theta0
      out_v=f_1*v_n_t+(1-f_1)*Kv*dx
      out_w=f_2*omega_n_t+(1-f_2)*Kw*(lambda1*dy+lambda2*dtheta)
      if out_v>max_v:
        out_v=max_v
      if out_w>max_w:
        out_w=max_w
      vel_msg.linear.x = out_v
      vel_msg.angular.z = omega[1]
      
    velocity_publisher.publish(vel_msg)
    
    r.sleep()

if __name__ == '__main__': 
  

  
  try:
    trajectory = []
    velocity_plotter()
  except rospy.ROSInterruptException:
    pass

