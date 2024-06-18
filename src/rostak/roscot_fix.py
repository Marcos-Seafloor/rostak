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
        rospy.Subscriber("/mavros/global_position/global", NavSatFix, self.publish_fix)
        self.heading = 0
        self.groundspeed = 0.0
        rospy.Subscriber("/mavros/vfr_hud", VFR_HUD, self.update_heading_groundspeed)
        rospy.loginfo(self.util.get_config())

    def update_heading_groundspeed(self, msg):
        """Update the heading from VFR_HUD message."""
        self.heading = msg.heading
        #rospy.loginfo(f"Updated heading: {self.heading}")
        """Update the groundspeed from VFR_HUD message and convert to knots."""
        self.groundspeed = msg.groundspeed * 1.94384
        #rospy.loginfo(f"Updated groundspeed: {self.groundspeed}")

    def publish_fix(self, msg):
        """Generate a status COT Event."""
        try:
            #rospy.loginfo(f"Received NavSatFix message: {msg}")
            self.util.set_point({
                "latitude": msg.latitude,
                "longitude": msg.longitude,
                "altitude": msg.altitude
            })
            #rospy.loginfo(f"Updated fix: {self.util.fix}")
            stale_in = 2 * max(1, 1 / self.rate)
            self.msg.data = self.util.new_status_msg(stale_in, self.heading, self.groundspeed)
            #rospy.loginfo(f"Publishing status message: {self.msg.data}")
            self.tx.publish(self.msg)
            #rospy.loginfo("Message published successfully")
        except Exception as e:
            rospy.logerr(f"Error in publish_fix: {e}")

if __name__ == '__main__':
    RosCotFix()
    rospy.spin()
