# nav_goal_pub_key_demo2.py
#!/usr/bin/env python3
# This script publishes navigation goals to a robot using ROS.
# It supports keyboard input for manual goal setting, audio commands for location names,
# and automatic navigation from a CSV file of coordinates.
"""
This script publishes navigation goals to a robot using ROS.
It supports keyboard input for manual goal setting, audio commands for location names,
and automatic navigation from a CSV file of coordinates.
"""

import rospy
import csv
import sys
import select
import math
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry


class NavGoalPublisher:
    def __init__(self):
        rospy.init_node('nav_goal_publisher', anonymous=True)
        self.goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
        
        # Location dictionary
        self.locations = {
            'home': [0.0, 0.0],
            'door': [1.0, 2.0],
            'kitchen': [5.0, 6.0],
            'bedroom': [10.0, 10.0]
        }
        
        # Mode control variables
        self.mode = "keyboard"  # Default mode
        self.auto_goals = []
        self.auto_index = 0
        self.current_pose = None
        self.count = 0
        
        # Subscribe to robot's current position
        self.odom_sub = rospy.Subscriber(
            '/odometry/map',
            Odometry,
            self.odom_callback,
            queue_size=10)

    def odom_callback(self, msg):
        """Update current robot position from odometry"""
        self.current_pose = msg.pose.pose
        
        # Periodic logging (every 50 messages)
        if self.count % 100 == 0:
            if self.current_pose:
                rospy.loginfo(
                    "Current position: x=%.2f, y=%.2f, frame_id=%s",
                    self.current_pose.position.x,
                    self.current_pose.position.y,
                    msg.header.frame_id
                )
        self.count += 1

    def distance_to_goal(self, goal_coords):
        """Calculate distance from current position to target"""
        if not self.current_pose:
            return float('inf')  # Large distance if pose unknown
            
        dx = goal_coords[0] - self.current_pose.position.x
        dy = goal_coords[1] - self.current_pose.position.y
        return math.sqrt(dx**2 + dy**2)

    def publish_goal_from_coords(self, coords):
        """Publish navigation goal from coordinates"""
        goal = PoseStamped()
        goal.header.stamp = rospy.Time.now()
        goal.header.frame_id = "map"
        goal.pose.position.x = coords[0]
        goal.pose.position.y = coords[1]
        goal.pose.position.z = 0.0
        goal.pose.orientation.w = 1.0
        self.goal_pub.publish(goal)
        rospy.loginfo("Published goal at: (%.2f, %.2f)", coords[0], coords[1])

    def publish_goal(self, location_name):
        """Publish goal for named location"""
        if location_name in self.locations:
            self.publish_goal_from_coords(self.locations[location_name])
        else:
            rospy.logwarn("Unknown location: %s", location_name)

    def load_auto_goals(self, filename):
        """Load navigation goals from CSV file"""
        self.auto_goals = []
        try:
            with open(filename, 'r') as f:
                reader = csv.reader(f)
                for row in reader:
                    if len(row) >= 2:
                        try:
                            x = float(row[0])
                            y = float(row[1])
                            self.auto_goals.append([x, y])
                        except ValueError:
                            rospy.logwarn("Skipping invalid row: %s", row)
            rospy.loginfo("Loaded %d goals from %s", len(self.auto_goals), filename)
            return True
        
        except Exception as e:
            rospy.logerr("Failed to load CSV: %s", e)
            return False

    def change_mode(self, new_mode):
        """Switch operation mode"""
        valid_modes = ["keyboard", "audio", "auto_nav"]
        if new_mode in valid_modes:
            self.mode = new_mode
            rospy.loginfo("Switched to %s mode", new_mode)
            
            # Reset auto-nav state when entering this mode
            if new_mode == "auto_nav":
                self.auto_goals = []
                self.auto_index = 0
            return True
        rospy.logwarn("Invalid mode: %s", new_mode)
        return False

    def check_keyboard(self):
        """Check for keyboard input (non-blocking)"""
        if select.select([sys.stdin], [], [], 0)[0]:
            return sys.stdin.readline().strip()
        return None

    def run(self):
        rate = rospy.Rate(10)  # 10Hz
        rospy.loginfo("Starting in keyboard mode. Commands: 'mode <name>', 'quit'")
        
        while not rospy.is_shutdown():
            # Check for keyboard input
            user_input = self.check_keyboard()
            
            if user_input:
                if user_input.startswith("mode "):
                    new_mode = user_input.split()[1]
                    self.change_mode(new_mode)
                elif user_input == "quit":
                    break
                elif self.mode == "keyboard":
                    self.publish_goal(user_input)

            # Handle auto_nav mode processing
            if self.mode == "auto_nav":
                # Load goals if not done
                if not self.auto_goals:
                    rospy.loginfo("Enter CSV filename for auto_nav:")
                    filename = sys.stdin.readline().strip()
                    if not self.load_auto_goals(filename):
                        self.change_mode("keyboard")
                        continue
                    self.auto_index = 0
                    self.publish_goal_from_coords(self.auto_goals[self.auto_index])
                
                # Process auto navigation sequence
                if self.auto_index < len(self.auto_goals):
                    current_goal = self.auto_goals[self.auto_index]
                    distance = self.distance_to_goal(current_goal)
                    
                    if distance < 0.2:  # Goal reached threshold
                        rospy.loginfo("Goal %d reached! (%.2f m)", self.auto_index, distance)
                        self.auto_index += 1
                        
                        if self.auto_index < len(self.auto_goals):
                            rospy.loginfo("Sending next goal: %d", self.auto_index)
                            self.publish_goal_from_coords(self.auto_goals[self.auto_index])
                        else:
                            rospy.loginfo("All goals completed!")
                            self.change_mode("keyboard")
                else:
                    rospy.loginfo("Auto navigation completed")
                    self.change_mode("keyboard")

            rate.sleep()

if __name__ == '__main__':
    try:
        node = NavGoalPublisher()
        node.run()
    except rospy.ROSInterruptException:
        pass

