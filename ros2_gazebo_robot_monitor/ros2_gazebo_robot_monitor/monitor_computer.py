import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import psutil
import os
import GPUtil
from rosgraph_msgs.msg import Clock
from rclpy.qos import qos_profile_sensor_data

class MonitorComputer(Node):

    def __init__(self):
        super().__init__('monitor_computer_node')
        #customization
        self.sav_frequency = 10
        self.sav_cpu = True
        self.sav_mem = True
        self.sav_gpus = True
        self.sav_custom_process = True
        self.custom_process_name = "planner_server"
        self.print_vals = False

        # Get the directory parameter
        self.declare_parameter('directory', 'monitor_robot_node_results/')
        self.directory = self.get_parameter('directory').get_parameter_value().string_value

        #variable
        self.directory = self.directory+"computer_stats/"
        self.time = 0
        self.time_got = False
        self.file_reseted = []

        #ROS2 variables
        self.timer = self.create_timer(1.0/self.sav_frequency, self.timer_callback) 
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

    def clock_callback(self, msg):
        # Update the time variable with the current simulation time
        self.time = msg.clock.sec + msg.clock.nanosec / 1e9
        self.time_got = True

    def timer_callback(self):
        if self.sav_cpu and self.time_got:
            self.save_cpu()
        if self.sav_mem and self.time_got:
            self.save_mem()
        if self.sav_gpus and self.time_got:
            gpus = GPUtil.getGPUs()
            if gpus:
                for gpu in gpus:
                    self.save_gpu(gpu)
                    """print(f"GPU {gpu.id}: {gpu.name}")
                    print(f"  Load: {gpu.load * 100}%")
                    print(f"  Memory Free: {gpu.memoryFree}MB")
                    print(f"  Memory Used: {gpu.memoryUsed}MB")
                    print(f"  Memory Total: {gpu.memoryTotal}MB")
                    print(f"  Temperature: {gpu.temperature} °C")"""
            else:
                print("No GPUs found.")
        if self.sav_custom_process:
            self.sav_process(self.custom_process_name)

        if self.print_vals: print("")
        
    def save_cpu(self):
        file = self.directory+"cpu_load.txt"
        msg = msg_template(self.time,[psutil.cpu_percent()])
        if self.print_vals: print("cpu_load: "+msg[:-1]+"%")
        self.write_in_file(file,msg)

    def sav_process(self,process_name):
        for proc in psutil.process_iter(['pid', 'name', 'cpu_percent']):
            if proc.info['name'] == process_name:
                num_cores = psutil.cpu_count()
                file = self.directory+"cpu_load_"+process_name+".txt"
                msg = msg_template(self.time,[proc.info['cpu_percent']/num_cores])
                if self.print_vals: print("cpu_load: "+msg[:-1]+"%")
                self.write_in_file(file,msg)

    def save_mem(self):
        file = self.directory+"memory_load.txt"
        msg = msg_template(self.time,[psutil.virtual_memory().percent])
        if self.print_vals: print("memory_load: "+msg[:-1]+"%")
        self.write_in_file(file,msg)

    def save_gpu(self,gpu):
        file = self.directory+gpu.name+"_load.txt"
        msg = msg_template(self.time,[gpu.load*100])
        if self.print_vals: print(gpu.name+"_load: "+msg[:-1]+"%")
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

    monitor_computer = MonitorComputer()

    rclpy.spin(monitor_computer)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    monitor_computer.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()