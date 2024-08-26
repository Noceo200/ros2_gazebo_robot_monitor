from roblib import *
import os

def get_data_from_file_to_matrix(file_path):
    # Read the data from the file
    with open(file_path, 'r') as file:
        lines = file.readlines()
    data = []
    for line in lines:
            if line.strip():  # Ignore empty lines
                val_list = line.strip().split(';')
                data.append([float(x) for x in val_list])
    return data

def read_and_plot_file(file_path, title, x_label, y_label, x_val_index, y_val_index, output_graph_path=None, show_plot=True):
    # Read the data from the file
    with open(file_path, 'r') as file:
        lines = file.readlines()

    # Parse the data
    x_values = []
    y_values = []
    for line in lines:
        if line.strip():  # Ignore empty lines
            val_list = line.strip().split(';')
            x_values.append(float(val_list[x_val_index]))
            y_values.append(float(val_list[y_val_index]))

    # Plot the data
    if show_plot or output_graph_path != None:
        plt.figure()
        plt.plot(x_values, y_values)
        plt.xlabel(x_label)
        plt.ylabel(y_label)
        plt.title(title)
        if show_plot:
            plt.show()
        if output_graph_path != None:
            plt.savefig(output_graph_path)
            print(output_graph_path+" saved...")

    return len(y_values),mean(y_values),np.std(y_values),np.min(y_values),np.max(y_values)

def save_cpu_load(file,output_indicators_path=None,output_graph_path=None,show_plot=False):
    #we save and/or plot the data
    print("CPU Load datas...")
    nb_vals,avr_load, std_load, min_load, max_load  = read_and_plot_file(file, "CPU_Load","time (s)","Load (%)", 0, 1, output_graph_path=output_graph_path, show_plot=show_plot)
    #we compute some indicators and save them
    sav_indicators(output_indicators_path,[nb_vals,avr_load, std_load, min_load, max_load],["Amount of values","Load average","Standard deviation","Min Load","Max Load"])

def save_memory_load(file,output_indicators_path=None,output_graph_path=None,show_plot=False):
    #we save and/or plot the data
    print("RAM Load datas...")
    nb_vals,avr_load, std_load, min_load, max_load  = read_and_plot_file(file, "RAM_Load","time (s)","Load (%)", 0, 1, output_graph_path=output_graph_path, show_plot=show_plot)
    #we compute some indicators and save them
    sav_indicators(output_indicators_path,[nb_vals,avr_load, std_load, min_load, max_load],["Amount of values","Load average","Standard deviation","Min Load","Max Load"])

def save_gpu_load(file,output_indicators_path=None,output_graph_path=None,show_plot=False):
    #we save and/or plot the data
    print("GPU Load datas...")
    nb_vals,avr_load, std_load, min_load, max_load  = read_and_plot_file(file, "GPU_Load","time (s)","Load (%)", 0, 1, output_graph_path=output_graph_path, show_plot=show_plot)
    #we compute some indicators and save them
    sav_indicators(output_indicators_path,[nb_vals,avr_load, std_load, min_load, max_load],["Amount of values","Load average","Standard deviation","Min Load","Max Load"])

def save_robot_abs_linear_speed(file,output_indicators_path=None,output_graph_path=None,show_plot=False):
    print("Speed datas...")
    all_speeds = get_data_from_file_to_matrix(file)
    abs_speed_lin = []
    times = []
    for spds in all_speeds:
        times.append(spds[0])
        abs_speed_lin.append(sqrt((spds[1]**2)+(spds[2]**2)))

    # Plot the data
    if show_plot or output_graph_path != None:
        plt.figure()
        plt.plot(times, abs_speed_lin)
        plt.xlabel("time (s)")
        plt.ylabel("absolute speed (m/s)")
        plt.title("Robot Absolute Speed")
        if show_plot:
            plt.show()
        if output_graph_path != None:
            plt.savefig(output_graph_path)
            print(output_graph_path+" saved...")

    sav_indicators(output_indicators_path,[len(abs_speed_lin),mean(abs_speed_lin),np.std(abs_speed_lin),np.min(abs_speed_lin),np.max(abs_speed_lin)],["Amount of values","Speed average","Standard deviation","Min Speed","Max Speed"])


def sav_indicators(file,vals_list,vals_names_list):
    msg = "INDICATORS:\n"
    for i in range(len(vals_names_list)):
        msg += vals_names_list[i]+": "+str(vals_list[i])+"\n"
    if file == None:
        print(msg)
    else:
        print(msg)
        with open(file, 'w') as file:
            file.write(msg)

if __name__ == "__main__":
    
    #directory = "/home/rexilius/workspace/CALL-M_core/robot_ws_ros2/src/included_external_packages/ros2_gazebo_robot_monitor/data_results/2024-08-25__22-58-42/"
    directory = "/home/rexilius/workspace/CALL-M_core/robot_ws_ros2/src/included_external_packages/ros2_gazebo_robot_monitor/data_results/test_merged_scan_costmaps_40Hz_40Hz/"

    dir_save_outputs = directory+"VISUALS/"
    # Check if the directory exists
    if not os.path.exists(dir_save_outputs):
        # Create the directory
        os.makedirs(dir_save_outputs)
        print(f"Directory '{dir_save_outputs}' created.")

    #CPU Load plotting
    cpu_load_file_path = directory+"computer_stats/cpu_load.txt"
    save_cpu_load(cpu_load_file_path,output_indicators_path=dir_save_outputs+"cpu_load_results.txt",output_graph_path=dir_save_outputs+"cpu_load_results_graph.png",show_plot=False)

    #RAM Load plotting
    memory_load_file_path = directory+"computer_stats/memory_load.txt"
    save_memory_load(memory_load_file_path,output_indicators_path=dir_save_outputs+"memory_load_results.txt",output_graph_path=dir_save_outputs+"memory_load_results_graph.png",show_plot=False)

    #GPU Load plotting
    gpu_load_file_path = directory+"computer_stats/NVIDIA GeForce GTX 1060_load.txt"
    save_gpu_load(gpu_load_file_path,output_indicators_path=dir_save_outputs+"gpu_load_results.txt",output_graph_path=dir_save_outputs+"gpu_load_results_graph.png",show_plot=False)

    #robot's speed data
    speeds_file_path = directory+"robot_stats/speeds.txt"
    save_robot_abs_linear_speed(speeds_file_path,output_indicators_path=dir_save_outputs+"abs_speed_results.txt",output_graph_path=dir_save_outputs+"abs_speed_results_graph.png",show_plot=False)
