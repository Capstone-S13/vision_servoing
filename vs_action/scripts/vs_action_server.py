#!/usr/bin/env python3

from vs_interfaces.msg import DockState
from vs_interfaces.msg import VisionServoActionAction
from vs_interfaces.msg import VisionServoActionGoal
from vs_interfaces.msg import VisionServoActionResult
from vs_interfaces.msg import VisionServoActionFeedback

import rospy
import actionlib
import math

from apriltag_ros.msg import AprilTagDetectionArray
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler

class VisionServoActionServer():
    def __init__(self):
    
        self.cmdvel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        rospy.Subscriber('/tag_detections', AprilTagDetectionArray, self.detectionCB)

        self.action_name = 'vs_action'
        self._as = actionlib.SimpleActionServer(
            self.action_name,
            VisionServoActionAction,
            execute_cb=self.execute_callback,
            auto_start = False
        )

        self.tag_data = AprilTagDetectionArray()
        self.tag_dt = {}

        check = False
        
        try:
            rospy.wait_for_message("/tag_detections", AprilTagDetectionArray, timeout=10.0)
            check = True
        except rospy.ROSException as e:
            rospy.logerr("AprilTag detections not running!")
            rospy.logerr(e)
            rospy.logerr("Shutting down node...")
            rospy.signal_shutdown("")

        self._state = DockState.IDLE
        self._stage_one_reached = False
        self._stage_two_reached = False
        self._goal_reached = False
        
        self._feedback = VisionServoActionFeedback()
        self._feedback.dock_state = self._state
        self._feedback.success = False

        self._result = VisionServoActionResult()
        self._result.dock_state = self._state
        self._result.success = False

        if check == True:
            rospy.loginfo('action server is up!')
            self._as.start()

    def execute_callback(self, goal_handle):
        # vision servo action

        self._state = DockState.IDLE
        self._stage_one_reached = False
        self._stage_two_reached = False
        self._goal_reached = False
        
        self._feedback = VisionServoActionFeedback()
        self._feedback.dock_state = self._state
        self._feedback.success = False

        self._result = VisionServoActionResult()
        self._result.dock_state = self._state
        self._result.success = False

        dock_id = goal_handle.tag_id

        rospy.loginfo(f"Docking to tag id: {dock_id}")
        rate = rospy.Rate(10) # 10hz

        self._state = DockState.PLANNING
        # self._feedback.dock_state = self._state
        # self._as.publish_feedback(self._feedback)

        while (not self._goal_reached):
            if (self._state == DockState.PLANNING):
                rospy.loginfo("===PLANNING STATE===")
                # calculate angle of tag to camera
                pose_w_cov = self.tag_dt[dock_id]['pose']
                # rospy.loginfo(pose_w_cov)
                _, angle, _ = self.get_rotation(pose_w_cov)
                #rospy.loginfo(f"Angle of Tag: {angle}")
                if (abs(angle) > 0.5):
                    self._state = DockState.STAGEONE

                    # get orthogonal position to tag based on map frame
                    #TBD
                else:
                    # minimal displacement -- skip STAGEONE
                    self._stage_one_reached = True
                    self._state = DockState.STAGETWO
                
                self._feedback.dock_state = self._state
                self._as.publish_feedback(self._feedback)
            
            elif (self._state == DockState.STAGEONE):
                rospy.loginfo("===STAGE ONE===")
                # move AGV using movebase
                # NOT DONE
                self._stage_one_reached = True
                self._state = DockState.STAGETWO
                self._feedback.dock_state = self._state
                self._as.publish_feedback(self._feedback)

            elif (self._state == DockState.STAGETWO):
                rospy.loginfo("===STAGE TWO===")
                vel_msg, self._stage_two_reached = self.calc_docking_cmd(self.tag_dt[dock_id]['tagArea'], self.tag_dt[dock_id]['pixelPosRight'])
                rospy.loginfo(vel_msg)
                self.cmdvel_pub.publish(vel_msg)

                if(self._stage_one_reached and self._stage_two_reached):
                    self._goal_reached = True
                    self._state = DockState.IDLE

                    self._result.dock_state = self._state
                    self._result.success = True
                else:
                    self._feedback.dock_state = self._state
                    self._as.publish_feedback(self._feedback)
            
            rate.sleep()
        
        rospy.loginfo('%s: Succeeded' % self.action_name)
        self._as.set_succeeded(self._result)



    def detectionCB(self, data):
        self.tag_data = data

        # clear data in dt when not updated
        for k in self.tag_dt:
            self.tag_dt[k] = {}

        if (data.detections):
            for row in data.detections:
                if (row.id[0] not in self.tag_dt):
                    self.tag_dt[row.id[0]] = {}
                    self.tag_dt[row.id[0]]['size'] = row.size[0]
                    self.tag_dt[row.id[0]]['pose'] = row.pose
                    self.tag_dt[row.id[0]]['pixelPosRight'] = row.pixelPosRight
                    self.tag_dt[row.id[0]]['pixelPosDown'] = row.pixelPosDown
                    self.tag_dt[row.id[0]]['tagArea'] = row.tagArea
                else:
                    self.tag_dt[row.id[0]]['size'] = row.size[0]
                    self.tag_dt[row.id[0]]['pose'] = row.pose
                    self.tag_dt[row.id[0]]['pixelPosRight'] = row.pixelPosRight
                    self.tag_dt[row.id[0]]['pixelPosDown'] = row.pixelPosDown
                    self.tag_dt[row.id[0]]['tagArea'] = row.tagArea

    def get_rotation(self, pose_w_cov):
        orientation_q = pose_w_cov.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = euler_from_quaternion (orientation_list)
        return roll, pitch, yaw

    def calc_docking_cmd(self, tag_area, pixelPosRight):
        # calculates speed cmds based on the tag and publishes to cmdvel topic
        docking_goal_reached = False
        vel_msg = Twist()

        ## LINEAR SPEED CALCULATION
        KP_linear = 0.001
        MAX_SPEED_METRESEC = 0.08
        tag_area_setpoint = 185000
        has_reached_lin = False

        linear_spd_cmd, has_reached_lin = self.calc_linear_spd_cmd(KP_linear, MAX_SPEED_METRESEC, tag_area_setpoint, tag_area)
        # uncomment to reverse
        # linear_spd_cmd = linear_spd_cmd * -1.0

        ## ANGULAR SPEED CALCULATION
        KP_yaw = 0.03
        MAX_YAW_RATE_RADSEC = 0.8
        has_reached_yaw = False

        yaw_rate_cmd, has_reached_yaw = self.calc_yaw_rate_cmd(KP_yaw, MAX_YAW_RATE_RADSEC, pixelPosRight)

        vel_msg.angular.z = -1.0 * yaw_rate_cmd
        vel_msg.linear.x = linear_spd_cmd

        if (has_reached_lin and has_reached_yaw):
            docking_goal_reached = True
        else:
            docking_goal_reached = False

        return vel_msg, docking_goal_reached

    def calc_linear_spd_cmd(self, KP_linear, MAX_SPEED_METRESEC, tag_area_setpoint, tag_area):
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

    def calc_yaw_rate_cmd(self, KP_yaw, MAX_YAW_RATE_RADSEC, pixelPosRight):
        # pixelPosRight 0.0 is left | 0.5 is center | 1.0 is right
        boresight_error = 0.5 - pixelPosRight

        # Intel Realsense L515
        # Field of View 70 deg Horiz x 55 deg Vert
        yaw_error_degs = -1.0 * boresight_error * 70   

        yaw_cmd = 0

        if (abs(yaw_error_degs) > 0.5):
            yaw_cmd = KP_yaw * yaw_error_degs
            has_reached_yaw = False
        else:
            yaw_cmd = 0
            has_reached_yaw = True

        if (yaw_cmd > MAX_YAW_RATE_RADSEC):
            yaw_cmd = MAX_YAW_RATE_RADSEC
        elif (yaw_cmd < -1.0 * MAX_YAW_RATE_RADSEC):
            yaw_cmd = -1.0 * MAX_YAW_RATE_RADSEC

        # if (abs(yaw_cmd) < 0.15):
        #     if (yaw_cmd < 0):
        #         yaw_cmd = -0.15
        #     else:
        #         yaw_cmd = 0.15

        return yaw_cmd, has_reached_yaw

def main(args=None):
    rospy.init_node('vision_servo_service_node')
    server = VisionServoActionServer()
    rospy.spin()

if __name__ == '__main__':
    main()

            
