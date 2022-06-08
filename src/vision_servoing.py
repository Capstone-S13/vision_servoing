import rospy
#import actionlib

from apriltag_ros.msg import AprilTagDetectionArray
from geometry_msgs.msg import Twist

tag_data = AprilTagDetectionArray()
tag_dt = {}

def detectionCB(data):
    tag_data = data
    
    # clear data in dt when not updated
    for k in tag_dt:
      tag_dt[k] = {}

    if (data.detections):
      for row in data.detections:
        if (row.id[0] not in tag_dt):
          tag_dt[row.id[0]] = {}
          tag_dt[row.id[0]]['size'] = row.size[0]
          tag_dt[row.id[0]]['pose'] = row.pose
          tag_dt[row.id[0]]['pixelPosRight'] = row.pixelPosRight
          tag_dt[row.id[0]]['pixelPosDown'] = row.pixelPosDown
          tag_dt[row.id[0]]['tagArea'] = row.tagArea
        else:
          tag_dt[row.id[0]]['size'] = row.size[0]
          tag_dt[row.id[0]]['pose'] = row.pose
          tag_dt[row.id[0]]['pixelPosRight'] = row.pixelPosRight
          tag_dt[row.id[0]]['pixelPosDown'] = row.pixelPosDown
          tag_dt[row.id[0]]['tagArea'] = row.tagArea

    #rospy.loginfo(tag_dt)


def calc_linear_spd_cmd(KP_linear, MAX_SPEED_METRESEC, tag_area_setpoint, tag_area):
  tag_area_error = (tag_area - tag_area_setpoint)/1000.0

  spd_cmd = 0

  if (abs(tag_area_error) > 5.0):
    spd_cmd = KP_linear * tag_area_error
    has_reached_lin = False
  else:
    spd_cmd = 0
    has_reached_lin = True

  if (spd_cmd > MAX_SPEED_METRESEC):
    spd_cmd = MAX_SPEED_METRESEC
  elif (spd_cmd < -1.0 * MAX_SPEED_METRESEC):
    spd_cmd = -1.0 * MAX_SPEED_METRESEC
  
  return spd_cmd, has_reached_lin

def calc_yaw_rate_cmd(KP_yaw, MAX_YAW_RATE_RADSEC, pixelPosRight):
  # pixelPosRight 0.0 is left | 0.5 is center | 1.0 is right
  boresight_error = 0.5 - pixelPosRight

  # Intel Realsense L515
  # Field of View 70 deg Horiz x 55 deg Vert
  yaw_error_degs = -1.0 * boresight_error * 70   

  yaw_cmd = 0

  if (abs(yaw_error_degs) > 2.0):
    yaw_cmd = KP_yaw * yaw_error_degs
    has_reached_yaw = False
  else:
    yaw_cmd = 0
    has_reached_yaw = True

  if (yaw_cmd > MAX_YAW_RATE_RADSEC):
    yaw_cmd = MAX_YAW_RATE_RADSEC
  elif (yaw_cmd < -1.0 * MAX_YAW_RATE_RADSEC):
    yaw_cmd = -1.0 * MAX_YAW_RATE_RADSEC

  return yaw_cmd, has_reached_yaw


def calc_docking_cmd(tag_area, pixelPosRight):
  # calculates speed cmds based on the tag and publishes to cmdvel topic
  docking_goal_reached = False
  vel_msg = Twist()

  ## LINEAR SPEED CALCULATION
  KP_linear = 0.0006
  MAX_SPEED_METRESEC = 0.2
  # 180000
  tag_area_setpoint = 220000
  has_reached_lin = False

  linear_spd_cmd, has_reached_lin = calc_linear_spd_cmd(KP_linear, MAX_SPEED_METRESEC, tag_area_setpoint, tag_area)
  # uncomment to reverse
  # linear_spd_cmd = linear_spd_cmd * -1.0

  ## ANGULAR SPEED CALCULATION
  KP_yaw = 0.0025
  MAX_YAW_RATE_RADSEC = 0.5
  has_reached_yaw = False

  yaw_rate_cmd, has_reached_yaw = calc_yaw_rate_cmd(KP_yaw, MAX_YAW_RATE_RADSEC, pixelPosRight)

  vel_msg.angular.z = -1.0 * yaw_rate_cmd
  vel_msg.linear.x = linear_spd_cmd

  if (has_reached_lin and has_reached_yaw):
    docking_goal_reached = True
  else:
    docking_goal_reached = False

  return vel_msg, docking_goal_reached


if __name__ == '__main__':
    try:
        rospy.init_node('vision_servo', anonymous=True)
        cmdvel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber('/tag_detections', AprilTagDetectionArray, detectionCB)

        dock = True

        rate = rospy.Rate(10) # 10hz
        while not rospy.is_shutdown():
          if dock:
            docking_goal_reached = False
            dock_id = 3
            zero_cmd = Twist()
            zero_cmd.linear.x = 0
            zero_cmd.angular.z = 0

            while (not docking_goal_reached):
              # publish to cmd_vel
              if (tag_dt):
                if (dock_id in tag_dt):
                  if (tag_dt[dock_id]):
                    #rospy.loginfo("tag id: " + str(dock_id) + ", tag area: " + str(tag_dt[dock_id]['tagArea']))
                    vel_msg, docking_goal_reached = calc_docking_cmd(tag_dt[dock_id]['tagArea'], tag_dt[dock_id]['pixelPosRight'])
                    rospy.loginfo(vel_msg)
                    cmdvel_pub.publish(vel_msg)

                    if (docking_goal_reached):
                      docking_status = "Docking Successful."
                  else:
                    # Tag not updating, E-stop Robot
                    cmdvel_pub.publish(zero_cmd)
                    docking_goal_reached = True
                    docking_status = "Docking Failed."

            rospy.loginfo(docking_status)

            dock = False

          rate.sleep()
        
        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()

    except rospy.ROSInterruptException:
        pass