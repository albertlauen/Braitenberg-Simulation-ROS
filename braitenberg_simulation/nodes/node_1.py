#!/usr/bin/env python

import rospy
import tf
from tf import transformations as ts
from geometry_msgs.msg import Transform, Vector3, Quaternion
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import Header

if __name__ == '__main__':
    rospy.init_node('fixed_tf_broadcaster')
    br = tf.TransformBroadcaster()
    listener = tf.TransformListener()
    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        # this is here to show your that this function signature is not consistent with others
        br.sendTransform((3.0, 2.0, 0.0),
                         tf.transformations.quaternion_from_euler(0, 0, 45.),
                         rospy.Time.now(),
                         "bad_python_child",
                         "world")

        # the Transform in python is not from tf::Transform, you need to feed in the exactly msg type.
        trans = Transform(translation=Vector3(1.0, 2.0, 0),
                          rotation=Quaternion(*tf.transformations.quaternion_from_euler(0, 0, 0))
                        )

        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = 'world'   # the parent link
        # I use the stamped msg signature call to prevent the order confusion
        trans_stamp = TransformStamped(header, 'child_py', trans)
        br.sendTransformMessage(trans_stamp)

        try:
            (trans, rot) = listener.lookupTransform('world', 'child_py', rospy.Time(0))
            # because it is not the tf::Transform, there is no reverse member function call, do it manually
            transform = ts.concatenate_matrices(ts.translation_matrix(trans), ts.quaternion_matrix(rot))
            inversed_trans = ts.inverse_matrix(transform)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

        # now publish the inverse frame back, it should stay at the same location of '/world'
        header.frame_id = 'child_py'
        trans = Transform(translation=Vector3(*ts.translation_from_matrix(inversed_trans)),
                          rotation=Quaternion(*ts.quaternion_from_matrix(inversed_trans))
                        )

        trans_stamp_inv = TransformStamped(header, 'world_inv_py', trans)

        br.sendTransformMessage(trans_stamp_inv)

        rate.sleep()