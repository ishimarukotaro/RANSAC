#!/usr/bin/env python3
import rospy
# メッセージの型等のimport
from shipcon_pcc.msg import ransac_and_quay_msgs
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

class Publishsers():
    def __init__(self):
        # Publisherを作成
        self.pub_marker = rospy.Publisher('ransac_line', Marker, queue_size=1)
        self.msg_marker = Marker()
        # messageの型を作成

    def make_msg(self, line):

        self.msg_marker.header.frame_id = "velodyne"
        self.msg_marker.type = self.msg_marker.LINE_STRIP
        self.msg_marker.action = self.msg_marker.ADD

        # marker scale
        self.msg_marker.scale.x = 0.03
        self.msg_marker.scale.y = 0.03
        self.msg_marker.scale.z = 0.03

        # marker color
        # self.msg_marker.color.a = 1.0
        # self.msg_marker.color.r = 1.0
        # self.msg_marker.color.g = 0.0
        # self.msg_marker.color.b = 0.0

        # marker color2
        self.msg_marker.color.a = 1.0
        self.msg_marker.color.r = 0.0
        self.msg_marker.color.g = 1.0
        self.msg_marker.color.b = 0.0

        # marker orientaiton
        self.msg_marker.pose.orientation.x = 0.0
        self.msg_marker.pose.orientation.y = 0.0
        self.msg_marker.pose.orientation.z = 0.0
        self.msg_marker.pose.orientation.w = 1.0

        # marker position
        self.msg_marker.pose.position.x = 0.0
        self.msg_marker.pose.position.y = 0.0
        self.msg_marker.pose.position.z = 0.0

        pt = line.line2pt(x_min=-5, x_max=5, y_min=-5, y_max=5)
        # marker line points
        self.msg_marker.points = []
        # first point
        first_line_point = Point()
        first_line_point.x = pt[0]
        first_line_point.y = pt[2]
        first_line_point.z = 0.0
        self.msg_marker.points.append(first_line_point)
        # second point
        second_line_point = Point()
        second_line_point.x = pt[1]
        second_line_point.y = pt[3]
        second_line_point.z = 0.0
        self.msg_marker.points.append(second_line_point)

    def send_msg(self, line):
        self.make_msg(line)
        self.pub_marker.publish(self.msg_marker)

class Subscribers():
    def __init__(self):
        #importしたメッセージファイルを箱に代入
        self.ransac_and_quay = ransac_and_quay_msgs()
        self.ransac_info = rospy.Subscriber("/ransac_and_quay", ransac_and_quay_msgs, self.ransac_cb)

    def ransac_cb(self, msg):
        self.ransac_and_quay = msg

class params2line():
    def __init__(self):
        pass

    def refrsh_data(self, sub):
        print(sub.ransac_and_quay.quay_params)
        self.a = sub.ransac_and_quay.quay_params[0]
        self.b = sub.ransac_and_quay.quay_params[1]
        self.c = sub.ransac_and_quay.quay_params[2]

    def line2pt(self, x_min=-5, x_max=5, y_min=-5, y_max=5):
        a = self.a
        b = self.b
        c = self.c
        if b != 0:
            m = -a/b
            n = -c/b
            x_min = x_min
            x_max = x_max
            y_min = m*x_min + n
            y_max = m*x_max + n
        else:
            x = -c/a
            x_min = x
            x_max = x
            y_min = y_min
            y_max = y_max
        return x_min, x_max, y_min, y_max

def main():
    # nodeの立ち上げ
    rospy.init_node('NODE_ransac_marker', anonymous = True)
    # インスタンスの作成と実行
    pub = Publishsers()
    sub = Subscribers()
    line = params2line()
    #rospy.spin()
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        if len(sub.ransac_and_quay.quay_params) == 0:
            continue
        line.refrsh_data(sub)
        pub.send_msg(line)
        #実行周波数を維持する?
        rate.sleep()

if __name__ == '__main__':
   main()
