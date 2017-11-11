#! /usr/bin/env python
import roslib
roslib.load_manifest('record_skeleton')
import rospy
import tf
#import geometry_msgs.msg

#def callback(data):
#  print data.child_frame_id
#  print data.transform.translation.x
#  print data.transform.translation.y
#  print data.transform.translation.z


#def listeneir():
if __name__=='__main__':
  rospy.init_node('record_skeleton', anonymous=True)
  listener=tf.TransformListener()
  rate=rospy.Rate(30.0)
  while not rospy.is_shutdown():
      try:
          (trans,rot)=listener.lookupTransform('openni_depth_frame','head_2',rospy.Time(0))
      except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
          continue
      f1=file("/home/bianjiang/catkin_ws/bagfiles/head.txt","ab+")
      f1.write(str(trans[0]))
      f1.write("      ")
      f1.write(str(trans[1]))
      f1.write("      ")
      f1.write(str(trans[2]))
      f1.write("      ")
      f1.write("\n")
      f1.close()
      try:
          (trans,rot)=listener.lookupTransform('openni_depth_frame','neck_2',rospy.Time(0))
      except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
          continue
      f2=file("/home/bianjiang/catkin_ws/bagfiles/neck.txt","ab+")
      f2.write(str(trans[0]))
      f2.write("      ")
      f2.write(str(trans[1]))
      f2.write("      ")
      f2.write(str(trans[2]))
      f2.write("      ")
      f2.write("\n")
      f2.close()
      try:
          (trans,rot)=listener.lookupTransform('openni_depth_frame','torso_2',rospy.Time(0))
      except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
          continue
      f3=file("/home/bianjiang/catkin_ws/bagfiles/torso.txt","ab+")
      f3.write(str(trans[0]))
      f3.write("      ")
      f3.write(str(trans[1]))
      f3.write("      ")
      f3.write(str(trans[2]))
      f3.write("      ")
      f3.write("\n")
      f3.close()
      try:
          (trans,rot)=listener.lookupTransform('openni_depth_frame','left_shoulder_2',rospy.Time(0))
      except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
          continue
      f4=file("/home/bianjiang/catkin_ws/bagfiles/left_shoulder.txt","ab+")
      f4.write(str(trans[0]))
      f4.write("      ")
      f4.write(str(trans[1]))
      f4.write("      ")
      f4.write(str(trans[2]))
      f4.write("      ")
      f4.write("\n")
      f4.close()
      try:
          (trans,rot)=listener.lookupTransform('openni_depth_frame','left_elbow_2',rospy.Time(0))
      except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
          continue
      f5=file("/home/bianjiang/catkin_ws/bagfiles/left_elbow.txt","ab+")
      f5.write(str(trans[0]))
      f5.write("      ")
      f5.write(str(trans[1]))
      f5.write("      ")
      f5.write(str(trans[2]))
      f5.write("      ")
      f5.write("\n")
      f5.close()
      try:
          (trans,rot)=listener.lookupTransform('openni_depth_frame','left_hand_2',rospy.Time(0))
      except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
          continue
      f6=file("/home/bianjiang/catkin_ws/bagfiles/left_hand.txt","ab+")
      f6.write(str(trans[0]))
      f6.write("      ")
      f6.write(str(trans[1]))
      f6.write("      ")
      f6.write(str(trans[2]))
      f6.write("      ")
      f6.write("\n")
      f6.close()
      try:
          (trans,rot)=listener.lookupTransform('openni_depth_frame','right_shoulder_2',rospy.Time(0))
      except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
          continue
      f7=file("/home/bianjiang/catkin_ws/bagfiles/right_shoulder.txt","ab+")
      f7.write(str(trans[0]))
      f7.write("      ")
      f7.write(str(trans[1]))
      f7.write("      ")
      f7.write(str(trans[2]))
      f7.write("      ")
      f7.write("\n")
      f7.close()
      try:
          (trans,rot)=listener.lookupTransform('openni_depth_frame','right_elbow_2',rospy.Time(0))
      except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
          continue
      f8=file("/home/bianjiang/catkin_ws/bagfiles/right_elbow.txt","ab+")
      f8.write(str(trans[0]))
      f8.write("      ")
      f8.write(str(trans[1]))
      f8.write("      ")
      f8.write(str(trans[2]))
      f8.write("      ")
      f8.write("\n")
      f8.close()
      try:
          (trans,rot)=listener.lookupTransform('openni_depth_frame','right_hand_2',rospy.Time(0))
      except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
          continue
      f9=file("/home/bianjiang/catkin_ws/bagfiles/right_hand.txt","ab+")
      f9.write(str(trans[0]))
      f9.write("      ")
      f9.write(str(trans[1]))
      f9.write("      ")
      f9.write(str(trans[2]))
      f9.write("      ")
      f9.write("\n")
      f9.close()
      try:
          (trans,rot)=listener.lookupTransform('openni_depth_frame','left_hip_2',rospy.Time(0))
      except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
          continue
      f10=file("/home/bianjiang/catkin_ws/bagfiles/left_hip.txt","ab+")
      f10.write(str(trans[0]))
      f10.write("      ")
      f10.write(str(trans[1]))
      f10.write("      ")
      f10.write(str(trans[2]))
      f10.write("      ")
      f10.write("\n")
      f10.close()
      try:
          (trans,rot)=listener.lookupTransform('openni_depth_frame','left_knee_2',rospy.Time(0))
      except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
          continue
      f11=file("/home/bianjiang/catkin_ws/bagfiles/left_knee.txt","ab+")
      f11.write(str(trans[0]))
      f11.write("      ")
      f11.write(str(trans[1]))
      f11.write("      ")
      f11.write(str(trans[2]))
      f11.write("      ")
      f11.write("\n")
      f11.close()
      try:
          (trans,rot)=listener.lookupTransform('openni_depth_frame','left_foot_2',rospy.Time(0))
      except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
          continue
      f12=file("/home/bianjiang/catkin_ws/bagfiles/left_foot.txt","ab+")
      f12.write(str(trans[0]))
      f12.write("      ")
      f12.write(str(trans[1]))
      f12.write("      ")
      f12.write(str(trans[2]))
      f12.write("      ")
      f12.write("\n")
      f12.close()
      try:
          (trans,rot)=listener.lookupTransform('openni_depth_frame','right_hip_2',rospy.Time(0))
      except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
          continue
      f13=file("/home/bianjiang/catkin_ws/bagfiles/right_hip.txt","ab+")
      f13.write(str(trans[0]))
      f13.write("      ")
      f13.write(str(trans[1]))
      f13.write("      ")
      f13.write(str(trans[2]))
      f13.write("      ")
      f13.write("\n")
      f13.close()
      try:
          (trans,rot)=listener.lookupTransform('openni_depth_frame','right_knee_2',rospy.Time(0))
      except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
          continue
      f14=file("/home/bianjiang/catkin_ws/bagfiles/right_knee.txt","ab+")
      f14.write(str(trans[0]))
      f14.write("      ")
      f14.write(str(trans[1]))
      f14.write("      ")
      f14.write(str(trans[2]))
      f14.write("      ")
      f14.write("\n")
      f14.close()
      try:
          (trans,rot)=listener.lookupTransform('openni_depth_frame','right_foot_2',rospy.Time(0))
      except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
          continue
      f15=file("/home/bianjiang/catkin_ws/bagfiles/right_foot.txt","ab+")
      f15.write(str(trans[0]))
      f15.write("      ")
      f15.write(str(trans[1]))
      f15.write("      ")
      f15.write(str(trans[2]))
      f15.write("      ")
      f15.write("\n")
      f15.close()

     # print trans[0]
     # print trans[1]
     # print trans[2]

      rate.sleep()


 # rospy.Subscriber("/tf", tf.TransformBroadcaster,  callback)
 # rospy.spin()

#if __name__=='__main__':
#  listener()
