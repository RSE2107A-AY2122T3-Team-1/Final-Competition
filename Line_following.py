#!/usr/bin/env python2
# BEGIN ALL
import rospy, cv2, cv_bridge, numpy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist

class Follower:
  def __init__(self):
    self.bridge = cv_bridge.CvBridge()
    self.image_sub = rospy.Subscriber('/camera/rgb/image_raw', 
                                      Image, self.image_callback)
    self.cmd_vel_pub = rospy.Publisher('/cmd_vel',
                                       Twist, queue_size=1)
    self.twist = Twist()

  def image_callback(self, msg):
    image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    lower_white = numpy.array([ 0,  0,  0])
    upper_white = numpy.array([ 255, 0, 255])
    mask = cv2.inRange(hsv, lower_white, upper_white)
    
    h, w, d = image.shape
    search_top = 3*h/4 -10
    search_bot = 3*h/4 + 10
    mask[0:search_top, 0:w] = 0
    mask[search_bot:h, 0:w] = 0
    mask[0:h, 0:w/4] = 0
    mask[0:h, 3*w/4:w] = 0
    M = cv2.moments(mask)
    if M['m00'] > 0:
      cx = int(M['m10']/M['m00'])
      cy = int(M['m01']/M['m00'])
      cv2.circle(image, (cx, cy), 20, (0,0,255), -1)
      # BEGIN CONTROL
      err = cx - w/2
      self.twist.linear.x = 0.50
      self.twist.angular.z = -float(err) / 100
      self.cmd_vel_pub.publish(self.twist)
      # END CONTROL

    image = cv2.resize(image, (600,600), interpolation = cv2.INTER_AREA)
    mask = cv2.resize(mask, (400,400), interpolation = cv2.INTER_AREA)
    cv2.imshow("Window", image)
#    cv2.imshow("Mask", mask)

    #cv2.imshow("HSV, Masked", numpy.hstack([image, mask]))
    k = cv2.waitKey(5) & 0xFF
    if k == 27:
      cv2.destroyAllWindows()

rospy.init_node('follower')
follower = Follower()
rospy.spin()
# END ALL
