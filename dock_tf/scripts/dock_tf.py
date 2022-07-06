from tkinter.messagebox import NO
from genpy import Time
import rospy
from tf2_ros import TransformListener, Buffer
from tf import transformations, TransformBroadcaster
from threading import Thread
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib import SimpleActionClient
from std_msgs.msg import String
import numpy


class DockTfPub():
    def __init__(self):
        self.tf2_buffer = Buffer()
        self.tf2_listener = TransformListener(self.tf2_buffer)
        self.broadcaster = TransformBroadcaster()
        self.move_base_client = SimpleActionClient('move_base', MoveBaseAction)
        self.camera_link = "camera_color_optical_frame"
        self.april_tag_link = "ID0"

        self.subscriber = rospy.Subscriber("dock", String, self.dock_cb)

        self.map_frame = "map"
        self.robot_frame = "base_link"

        self.april_tag_to_cam_tf = None
        self.map_to_robot_tf = None

    def dock_cb(self, msg):
        rospy.loginfo(f"got message {msg.data}")
        if (msg.data == "dock"):
            dock_tf = self.get_dock_tf()
            if (dock_tf is not None):
                goal = MoveBaseGoal()
                goal.target_pose.header.frame_id = self.map_frame

                goal.target_pose.pose.position.x = dock_tf.transform.translation.x
                goal.target_pose.pose.position.y = dock_tf.transform.translation.y
                goal.target_pose.pose.position.z = dock_tf.transform.translation.z

                quat = []
                quat.append(dock_tf.transform.rotation.x)
                quat.append(dock_tf.transform.rotation.y)
                quat.append(dock_tf.transform.rotation.z)
                quat.append(dock_tf.transform.rotation.w)

                euler = transformations.euler_from_quaternion(quat)
                # rospy.loginfo(f"numpy.allclose: {numpy.allclose(euler)} ")
                rospy.loginfo(f"target euler: {euler[1]}")
                target_quat = transformations.quaternion_from_euler(0.0, 0.0, (euler[2] + numpy.pi/2))

                goal.target_pose.pose.orientation.x = target_quat[0]
                goal.target_pose.pose.orientation.y = target_quat[1]
                goal.target_pose.pose.orientation.z = target_quat[2]
                goal.target_pose.pose.orientation.w = target_quat[3]
                rospy.loginfo("sending goal")
                self.move_base_client.send_goal(goal)

    def get_dock_tf(self):
        try:
            transform = self.tf2_buffer.lookup_transform(
                self.map_frame,
                "dock",
                rospy.Time(0)
            )
            return transform

        except Exception as e:
            rospy.logwarn(e)
            return None

    def get_april_tag_tf(self):
        try:
            transform = self.tf2_buffer.lookup_transform(
                self.camera_link,
                self.april_tag_link,
                rospy.Time(0)
            )

            self.april_tag_to_cam_tf = transform
        except Exception as e:
            rospy.logwarn(e)

    def get_robot_transform(self):
        try:
            transform = self.tf2_buffer.lookup_transform(
                self.map_frame,
                self.robot_frame,
                rospy.Time(0)
            )
            self.map_to_robot_tf = transform

        except Exception as e:
            rospy.logwarn(e)

    def get_map_to_tag_tf(self):
        try:
            transform = self.tf2_buffer.lookup_transform(
                self.map_frame,
                self.april_tag_link,
                rospy.Time(0)
            )
            return transform

        except Exception as e:
            rospy.logwarn(e)



    def get_april_tag_rotation(self):
        # +x should point to the right in the image
        # +y should point down in the image
        # +z should point into the plane of the image
        # because of the conventional orientation stated above
        # we are interested in the roll
        tag_quaternion = []
        tag_quaternion[0] = self.april_tag_to_cam_tf.transform.rotation.x
        tag_quaternion[1] = self.april_tag_to_cam_tf.transform.rotation.y
        tag_quaternion[2] = self.april_tag_to_cam_tf.transform.rotation.z
        tag_quaternion[3] = self.april_tag_to_cam_tf.transform.rotation.w

        yaw = transformations.euler_from_quaternion(
                tag_quaternion)[0]
        return yaw

    def get_robot_rotation(self):
        robot_quaternion = []
        robot_quaternion[0] = self.map_to_robot_tf.transform.rotation.x
        robot_quaternion[1] = self.map_to_robot_tf.transform.rotation.y
        robot_quaternion[2] = self.map_to_robot_tf.transform.rotation.z
        robot_quaternion[3] = self.map_to_robot_tf.transform.rotation.w

        yaw = transformations.euler_from_quaternion(
            robot_quaternion)[2]
        return yaw


    def combine_yaw_to_quaternion(self):
        tag_roll = self.get_april_tag_rotation
        robot_yaw = self.get_robot_rotation
        combined_rotation = tag_roll + robot_yaw
        quaternion = transformations.quaternion_from_euler(
            0.0, 0.0, combined_rotation
        )
        return quaternion

    def get_map_to_dock_tf(self):
        # self.april_tag_to_cam_tf.transform.translation.z
        return

    def loop(self):
        while not rospy.is_shutdown():
            rospy.loginfo("loop")
            tf  = self.get_map_to_tag_tf()

            quat = []
            coord = {}
            if tf is not None:

                quat.append(tf.transform.rotation.x)
                quat.append(tf.transform.rotation.y)
                quat.append(tf.transform.rotation.z)
                quat.append(tf.transform.rotation.w)


                coord['x']= (tf.transform.translation.x)
                coord['y']= (tf.transform.translation.y)
                coord['z']= (tf.transform.translation.z)

                self.broadcaster.sendTransform(
                    (0.0, 0.0, 1.5),
                    transformations.quaternion_from_euler(0.0, 0.0, 0.0),
                    rospy.Time.now(),
                    "dock",
                    self.april_tag_link
                )

            euler_angles = transformations.euler_from_quaternion(quat)
            info = ""
            for i in range(len(euler_angles)):
                info += f"euler[{i}]: {euler_angles[i]}"

            for key in coord:
                rospy.loginfo(f"coordinates {key}: {coord[key]}")


            rospy.loginfo(info)
            rospy.Rate(0.5).sleep()



def main():
    rospy.init_node('dock_tf_node')
    node = DockTfPub()
    rate = rospy.Rate(0.5)
    loop_thread = Thread(target=node.loop, args=())
    rospy.loginfo("starting loop")
    loop_thread.start()
    rospy.loginfo("starting spin")
    rospy.spin()



if __name__ == '__main__':
    main()





