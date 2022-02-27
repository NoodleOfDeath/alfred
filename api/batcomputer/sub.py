#!/usr/bin/env python3

import rospy
import sqlite3

from std_msgs.msg import Empty, Float32, Int16, UInt16
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from tf2_msgs.msg import TFMessage
	
rospy.init_node('batcomputer_subscriber')

def update_record(topic: str, value: any):
	#print(f"{topic}: {value}")
	con = sqlite3.connect('status.db')
	cursor = con.cursor()
	cursor.execute(f"SELECT * FROM sensor_states WHERE topic = '{topic}'")
	query = f"INSERT INTO sensor_states VALUES ('{topic}', '{value}', NULL);"
	if len(cursor.fetchall()) > 0:
		query = f"UPDATE sensor_states SET value = '{value}', timestamp = CURRENT_TIMESTAMP WHERE topic = '{topic}';"
	con.execute(query)
	con.commit()
	con.close()

# Subscribers
topics = {
	("/battery/capacity", 		Float32, lambda data: update_record("/battery/capacity", data.data)),
	("/battery/charge", 		Float32, lambda data: update_record("/battery/charge", data.data)),
	("/battery/charge_ratio", 	Float32, lambda data: update_record("/battery/charge_ratio", data.data)),
	# ("/battery/charging_state", create_msgs.ChargingState),
	("/battery/current", 		Float32, lambda data: update_record("/battery/current", data.data)),
	("/battery/temperature", 	Int16, lambda data: update_record("/battery/temperature", data.data)),
	("/battery/voltage", 		Float32, lambda data: update_record("/battery/voltage", data.data)),
	# ("/bumper", create_msgs.Bumper),
	# ("/clean_button", 	Empty, lambda data: print('/clean_button')),
	# ("/day_button", 	Empty, lambda data: print('/day_button')),
	# ("/debris_led", 	Empty, lambda data: print('/debris_led')),
	# ("/dock_button", 	Empty, lambda data: print('/dock_button')),
	# ("/dock_led", 		Empty, lambda data: print('/dock_led')),
	# ("/hour_button", 	Empty, lambda data: print('/hour_button')),
	("/ir_omni", 		UInt16, lambda data: update_record("/battery/ir_omni", data.data)),
	("/joint_states", 	JointState),
	# ("/minute_button", 	Empty, lambda data: print('/minute_button')),
	# ("/mode", create_msgs.Mode),
	("/odom", Odometry),
	# ("/power_led", 		Empty, lambda data: print('/power_led')),
	# ("/spot_button", 	Empty, lambda data: print('/spot_button')),
	# ("/spot_led", 		Empty, lambda data: print('/spot_led')),
	# ("/wheeldrop",		Empty, lambda data: print('/wheeldrop')),
	("/tf", TFMessage),
}
subscribers = []

for topic in topics:
	if (len(topic) == 3):
		rospy.Subscriber(topic[0], topic[1], topic[2] );

rospy.spin()
