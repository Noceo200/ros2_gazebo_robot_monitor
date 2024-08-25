import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import os
from gazebo_msgs.msg import ModelStates
from rosgraph_msgs.msg import Clock
from rclpy.qos import qos_profile_sensor_data

class MonitorRobot(Node):

    def __init__(self):
        super().__init__('monitor_robot_node')
        #customization
        self.robot_name = "call_m_bot"
        self.sav_frequency = 10
        self.sav_state = True #x,y,z,quat_x,quat_y,quat_z,quat_w  (in gazebo world frame)
        self.sav_speed = True #dx,dy,dz,d_angle_x,d_angle_y,d_angle_z (in gazebo world frame)
        self.print_vals = False

        # Get the directory parameter
        self.declare_parameter('directory', 'monitor_robot_node_results/')
        self.directory = self.get_parameter('directory').get_parameter_value().string_value

        #variable
        self.directory = self.directory+"robot_stats/"
        self.time = 0
        self.data_got = False
        self.time_got = False
        self.file_reseted = []

        #ROS2 variables
        self.timer = self.create_timer(1.0/self.sav_frequency, self.timer_callback) 
        self.states = [] #Vector7 position and orientation
        self.speeds = [] #Vector6 linear speeds and rotationnal speeds
        # Create a subscriber to the /gazebo/model_states topic
        self.subscription = self.create_subscription(
            ModelStates,
            '/gazebo/model_states',
            self.model_states_callback,
            qos_profile_sensor_data)
        # Create a subscriber to the /clock topic
        self.clock_subscription = self.create_subscription(
            Clock,
            '/clock',
            self.clock_callback,
            qos_profile_sensor_data)

        #initialisation
        # Check if the directory exists
        if not os.path.exists(self.directory):
            # Create the directory
            os.makedirs(self.directory)
            print(f"Directory '{self.directory}' created.")

    def model_states_callback(self, msg):
        # Find the index of the robot by its name
        robot_index = -1
        for i, name in enumerate(msg.name):
            if name == self.robot_name:
                robot_index = i
                break

        if robot_index != -1:
            pose = msg.pose[robot_index]
            twist = msg.twist[robot_index]

            # Extract position and orientation
            position = pose.position
            orientation = pose.orientation
            self.states = [
                position.x, position.y, position.z,
                orientation.x, orientation.y, orientation.z, orientation.w
            ]

            # Extract linear and angular velocities
            linear = twist.linear
            angular = twist.angular
            self.speeds = [
                linear.x, linear.y, linear.z,
                angular.x, angular.y, angular.z
            ]

            self.data_got = True
        else:
            print("ERROR: Couldn't find robot: "+self.robot_name+" in Gazebo objects.")

    def clock_callback(self, msg):
        # Update the time variable with the current simulation time
        self.time = msg.clock.sec + msg.clock.nanosec / 1e9
        self.time_got = True

    def timer_callback(self):
        if self.sav_state and self.data_got and self.time_got:
            self.save_state()
        if self.sav_speed and self.data_got and self.time_got:
            self.save_speed()
        if self.print_vals: print("")
        """if not self.data_got:
            print("Waiting topic /gazebo/model_states")"""
        
    def save_state(self):
        file = self.directory+"states.txt"
        msg = msg_template(self.time,self.states)
        if self.print_vals: print("states: "+msg[:-1]+"(time,x,y,z,quat_x,quat_y,quat_z,quat_w)(m and rad)")
        self.write_in_file(file,msg)

    def save_speed(self):
        file = self.directory+"speeds.txt"
        msg = msg_template(self.time,self.speeds)
        if self.print_vals: print("speeds: "+msg[:-1]+"(time,dx,dy,dz,d_angle_x,d_angle_y,d_angle_z)(m/s and rad/s)")
        self.write_in_file(file,msg)

    def write_in_file(self,file,msg):
        if file in self.file_reseted:
            with open(file, 'a') as file:
                file.write(msg)
        else:
            self.file_reseted.append(file)
            with open(file, 'w') as file:
                file.write(msg)

def msg_template(t,x_list):
    msg = str(t)
    for x in x_list:
        msg+=";"+str(x)
    return msg+"\n"

def main(args=None):
    rclpy.init(args=args)

    monitor_robot = MonitorRobot()

    rclpy.spin(monitor_robot)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    monitor_robot.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()