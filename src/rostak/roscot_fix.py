import rospy
from rostak.cot_utility import CotUtility
from std_msgs.msg import String
from sensor_msgs.msg import NavSatFix
from mavros_msgs.msg import VFR_HUD

class RosCotFix:
    def __init__(self):
        rospy.init_node("roscot_fix")
        config_path = rospy.get_param('~cot_params')
        self.util = CotUtility(config_path)
        self.rate = rospy.get_param('~rate', 0.2)
        self.tx = rospy.Publisher('tak_tx', String, queue_size=1)
        self.msg = String()
        rospy.Subscriber("fix", NavSatFix, self.publish_fix)
        self.heading = 0.0
        rospy.Subscriber("/mavros/vfr_hud", VFR_HUD, self.update_heading)
        rospy.loginfo(self.util.get_config())

    def update_heading_and_speed(self, msg):
        """Update heading from VFR_HUD message."""
        self.heading = msg.heading
        rospy.loginfo(f"Updated heading: {self.heading}")
        # Convert ground speed to knots
        #self.ground_speed = msg.groundspeed * 1.94384

    def publish_fix(self, msg):
        """Generate a status COT Event."""
        #self.util.set_point(msg)
        self.util.set_point({
            "latitude": msg.latitude,
            "longitude": msg.longitude,
            "altitude": msg.altitude
        })
        stale_in = 2 * max(1, 1 / self.rate)
        self.msg.data = self.util.new_status_msg(stale_in, self.heading)
        self.tx.publish(self.msg)

if __name__ == '__main__':
    RosCotFix()
    rospy.spin()