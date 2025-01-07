"""
 * CPGPlotter.py
 * This file is a visualizer of Envirobot running a CPG controller
 *
 *  Created on: Oct 29, 2024
 *      Author: Séverin Konishi
 *
 * How to use:
 * "python Plotter.py"              will run the CPG controller with the parameters specified below (and display an animation)
 * "python Plotter.py filename.csv" will display an animation of the robot from the joint setpoints stored in the file
"""
import matplotlib.pyplot as plt
import numpy as np
import time
from datetime import datetime
import sys
from CPG import *
from CPP_CPG import *
import threading
import queue
from LogLib import LogFile

#====================== #
#===== PARAMETERS ===== #
#====================== #
# === Hyper parameters === #
number_modules = 4      # number of modules/joints (at least 1), does not include the tail or the head

# === default CPG parameters === #
frequency = 0.5         # oscillator frequency
direction = 0           # turning direction (positive/negative) and amplitude
amplc = 0.3             # defines the amplitude of the first joint (closest to head)
amplh = 0.5             # defines the amplitude of the last joint (furthest from head)
nwave = 1               # number of waves peaks that can be seen on the robot at the same time
coupling_strength = 50  # speed at which the phase difference between coupled oscillators converges
a_r = 10                # speed at which the amplitudes of the oscillators converge

# === Computing parameters === #
use_cpp_version = True   # use the C++ implementation that is compiled into a .dll (did not work on personnal laptop, only lab desktop)
file_save = False        # saves the joint setpoints and timebase to a csv file
delta_ms = 1             # integration stepsize in milliseconds (lower is more expensive)

# === Plotting parameters === #
plot_robot_pose = True  # Plot the robot pose in real time
plot_power = True       # Plot the power consumption (if in input file)
plot_energy = True      # Plot the energy consumption (if in input file)
fps = 30                # animation frames per seconds (higher is more expensive)
speed = 1               # animation speed multiplier (higher is more expensive)
duration = 120          # animation duration in seconds

#============================#
#===== GLOBAL VARIABLES =====#
#============================#
#CPG controller (python or C++ version)
if use_cpp_version:
    controller = CPP_CPG(number_modules, frequency, direction, amplc, amplh, nwave, coupling_strength, a_r)
else:
    controller = CPG(number_modules, frequency, direction, amplc, amplh, nwave, coupling_strength, a_r)

#User shell commands
shell_queue = queue.Queue()
#data coming from cpg controller/robot
render_queue = queue.Queue()

#stop flag
stop = False

#===================== #
#====== THREADS ====== #
#===================== #
#Shell thread for the user to interact with the controller and plotter
#Not used when reading a file
def shell_thread():
    global stop, controller
    while not stop:
        command = input("[CPG]$ ")
        if command == "exit" or command == "stop":
            stop = True
            break
        elif len(command) == 0:
            continue
        elif len(command.split(" ")) == 2:
            name = command.split(" ")[0]
            value = float(command.split(" ")[1])
            if name == "freq":
                controller.set_frequency(value)
                shell_queue.put("[Plotter] frequency {0}".format(value))
            elif name == "dir":
                controller.set_direction(value)
                shell_queue.put("[Plotter] direction {0}".format(value))
            elif name == "amplc":
                controller.set_amplc(value)
                shell_queue.put("[Plotter] amplc {0}".format(value))
            elif name == "amplh":
                controller.set_amplh(value)
                shell_queue.put("[Plotter] amplh {0}".format(value))
            elif name == "nwave":
                controller.set_nwave(value)
                shell_queue.put("[Plotter] nwave {0}".format(value))
            elif name == "coupling":
                controller.set_coupling_strength(value)
                shell_queue.put("[Plotter] coupling {0}".format(value))
            elif name == "ar":
                controller.set_a_r(value)
                shell_queue.put("[Plotter] ar {0}".format(value))
            else:
                print("Unrecognized variable name")
        else:
            print("Unrecognized command")

#CPG computation thread
#Compute the CPG joint positions or reads them from the input file
def cpg_thread():
    global stop, controller, delta_ms, number_modules, plot_power, plot_energy
    stop_cpg = False

    #check if input log file
    if len(sys.argv) == 2:
        #initialize the input log file
        input_file = LogFile()
        input_file.open(sys.argv[1])
        # count the number of modules in the file
        number_modules = 0
        loop = True
        while loop:
            if not ("joint{0}".format(number_modules) in input_file.state_keys):
                loop = False
                break
            number_modules += 1
        print("Detected {0} modules".format(number_modules))

    #create output log file (only if no input file)
    elif len(sys.argv) == 1 and file_save:
        now = datetime.now()
        output_file = LogFile()
        output_file.new(now.strftime("exports/%d_%m_%Y_%H_%M_%S.csv"), ["joint{0}".format(i) for i in range(number_modules)], ["print"])

    #initialization
    robot_states = {"time": None, "joint": None, "energy": None, "power": None}
    robot_events = {"print": None}
    joint_setpoints = np.zeros(number_modules)  #joint angle for each module, given by CPG (in the module frame = motor position setpoints)

    #if there is an input file, read first entry to get the first timestamp
    t = 0
    if len(sys.argv) == 2:
        raw_data = input_file.read()
        t = int(raw_data["time"])
    next_frame = t

    #run the loop
    # This computes CPG setpoints or reads them from file and puts them into a queue
    # This queue is then read by the main thread to plot it with matplotlib
    while (not stop) and (not stop_cpg):
        #limit how much we compute in advance to still allow for live CPG parameter changes but keep a smooth framerate
        if render_queue.qsize() < int(fps/10):
            # == Compute a step of the CPG controller and update the joint setpoints == #
            if len(sys.argv) == 1:
                #compute the CPG step
                joint_setpoints = np.deg2rad(controller.step(delta_ms))
                #update the robot informations to put in the render queue
                robot_states["time"] = t
                robot_states["joint"] = joint_setpoints
                #save to file if enabled
                if len(sys.argv) == 1 and file_save:
                    output_data = {}
                    output_data["time"] = t
                    for i in range(number_modules):
                        output_data["joint{0}".format(i)] = str(int(np.rad2deg(joint_setpoints)[i]))
                    #add notification in the file that a CPG parameter was changed by the user
                    if shell_queue.qsize() > 0:
                        output_data["print"] = shell_queue.get()
                    output_file.write(output_data)
                t += delta_ms

                #stop the rendering if above the max simulation duration
                if t >= duration*1000:
                    render_queue.put(None) #notify the main thread that the simulation is over
                    stop_cpg = True
                    break

            # == Read a new line from the file == #
            else:
                robot_states["time"] = int(raw_data["time"])
                robot_states["joint"] = []
                robot_states["energy"] = []
                robot_states["power"] = []
                for i in range(number_modules):
                    robot_states["joint"].append(float(raw_data["joint{0}".format(i)]))
                    if "energy{0}".format(i) in raw_data:
                        robot_states["energy"].append(float(raw_data["energy{0}".format(i)]))
                    if "power{0}".format(i) in raw_data:
                        robot_states["power"].append(float(raw_data["power{0}".format(i)]))
                robot_states["joint"] = np.deg2rad(np.array(robot_states["joint"]))
                #Disable power plotting if not in file
                if len(robot_states["power"]) > 0:
                    robot_states["power"] = np.array(robot_states["power"])
                else:
                    plot_power = False
                #Disable energy plotting if not in file
                if len(robot_states["energy"]) > 0:
                    robot_states["energy"] = np.array(robot_states["energy"])
                else:
                    plot_energy = False
                #add print event if there is one
                if "print" in raw_data:
                    robot_events["print"] = raw_data["print"]

                #grab next file entry to know until when the current states should be shown
                raw_data = input_file.read()
                #if end of file
                if raw_data == None:
                    #render the last frame
                    render_queue.put([robot_states,robot_events])
                    #end of file, tell the renderer to stop
                    render_queue.put(None)
                    stop_cpg = True
                    break
                delta_ms = t - int(raw_data["time"])
                t = int(raw_data["time"])

            # == Add the data to the rendering queue == #
            while plot_robot_pose and (t >= next_frame):
                #compute when to grab the next frame
                next_frame += (1000*speed/fps)
                render_queue.put([robot_states,robot_events])
                robot_events = {"print": None}
        else:
            time.sleep(0.01)


#Only start the shell if not reading from a file
if len(sys.argv) == 1:
    shell_thread_handle = threading.Thread(target=shell_thread)
    shell_thread_handle.start()

#Start CPG thread
cpg_thread_handle = threading.Thread(target=cpg_thread)
cpg_thread_handle.start()
time.sleep(0.5) #give a head start to the thread start computing some joint setpoints before plotting

#================================== #
#===== Plotting (Main Thread) ===== #
#================================== #
if number_modules < 2:
    raise(Exception("Need at least 2 modules"))

#Plotting variables
module_absolute_angles = np.zeros(number_modules) #angle at which the module is in the global reference frame, +1 for the tail
joint_positions = np.zeros((number_modules, 2))   #position in 2D space of each joint (each module is 1 unit long) in the global reference frame

#Initialize plot of physical robot
if plot_robot_pose:
    fig, ax = plt.subplots()
    ax.set_title("Robot pose")
    axis_lenth = number_modules+2
    ax.set_xlim(-1, axis_lenth-1)
    ax.set_ylim(-axis_lenth/2,axis_lenth/2)
    line = ax.plot(np.zeros(1), np.zeros(1))[0]
    points = ax.scatter(np.zeros(1), np.zeros(1))

#Initialize plot of power and energy consumption (if reading from file)
if len(sys.argv) == 2 and plot_power:
    fig_power, ax_power = plt.subplots()
    ax_power.set_title("Power consumption")
    ax_power.set_ylabel("Power [W]")
    ax_power.set_xlabel("Time [ms]")
    lines_power = []
    for i in range(number_modules):
        lines_power.append(ax_power.plot(np.zeros(1), np.zeros(1), label="joint {0}".format(i))[0])
    ax_power.legend(loc="upper left")
if len(sys.argv) == 2 and plot_energy:
    fig_energy, ax_energy = plt.subplots()
    ax_energy.set_title("Energy consumption")
    ax_energy.set_ylabel("Energy [J]")
    ax_energy.set_xlabel("Time [ms]")
    lines_energy = []
    for i in range(number_modules):
        lines_energy.append(ax_energy.plot(np.zeros(1), np.zeros(1), label="joint {0}".format(i))[0])
    ax_energy.legend(loc="upper left")

#Give time to the user to rearange the matplotlib windows
plt.show(block=False)
input("Rearange the windows and press enter")

#Store all the power and energy data read from file
time_history = []
power_history = None
energy_history = None

start = time.time()
while(not stop) and plot_robot_pose:
    #wait for queue to fill (should normally never be empty)
    while(render_queue.empty() and (not stop)):
        print("[Warning] desired framerate not achieved, lower the framerate, speed or delta_t")
        time.sleep(1)
    
    #retrieve what the cpg thread computed (or what was read from file)
    raw_queue = render_queue.get()

    #end of animation
    if raw_queue is None:
        stop = True
        print("Animation done\n[CPG]$ - press enter to exit -", end="")
        break

    #retrieve the data to be plotted
    robot_states = raw_queue[0]
    robot_events = raw_queue[1]
    #joint data
    joint_setpoints = robot_states["joint"]
    #power and energy data
    time_history.append(robot_states["time"])
    if power_history is None:
        energy_history = np.array([robot_states["energy"]])
        power_history = np.array([robot_states["power"]])
    else:
        energy_history = np.concatenate((energy_history, np.array([robot_states["energy"]])))
        power_history = np.concatenate((power_history, np.array([robot_states["power"]])))
    #CPG parameter update
    if not robot_events["print"] == None:
        print(robot_events["print"])

    #compute absolute module angles and joint positions (for rendering)
    for i in range(number_modules):
        if i == 0:
            module_absolute_angles[i] = 0
            joint_positions[i,0] = 1
            joint_positions[i,1] = 0
        else:
            module_absolute_angles[i] = module_absolute_angles[i-1] + joint_setpoints[i-1]
            joint_positions[i,0] = joint_positions[i-1,0] + np.cos(module_absolute_angles[i])
            joint_positions[i,1] = joint_positions[i-1,1] + np.sin(module_absolute_angles[i])
    
    #Physical robot plotting
    line.set_xdata(np.concatenate((np.zeros(2), joint_positions[:,0]), axis=None))
    line.set_ydata(np.concatenate((np.zeros(2), joint_positions[:,1]), axis=None))
    points.set_offsets(np.column_stack((joint_positions[:,0],joint_positions[:,1])))

    #power and energy plotting
    if plot_power:
        ax_power.set_xlim(time_history[-1]-3000, time_history[-1])
        ax_power.set_ylim(0,np.max(power_history[:,i])+1)
        for i in range(number_modules):
            lines_power[i].set_xdata(time_history)
            lines_power[i].set_ydata(power_history[:,i])
    if plot_energy:
        ax_energy.set_xlim(time_history[-1]-3000, time_history[-1])
        ax_energy.set_ylim(0,np.max(energy_history[:,i])+1)
        for i in range(number_modules):
            lines_energy[i].set_xdata(time_history)
            lines_energy[i].set_ydata(energy_history[:,i])

    #show plots on screen in real time
    fig.canvas.flush_events()
    fig_power.canvas.flush_events()
    fig_energy.canvas.flush_events()
    plt.show(block=False)
    time.sleep(max((1/fps) - (time.time()-start-0.001), 0.001))
    start = time.time()

# == Final plotting of power and energy ==
#power final plotting
if plot_power:
    ax_power.set_xlim(time_history[0], time_history[-1])
    ax_power.set_ylim(0,np.max(power_history[:,i])+1)
    for i in range(number_modules):
        lines_power[i].set_xdata(time_history)
        lines_power[i].set_ydata(power_history[:,i])
#energy final plotting
if plot_energy:
    ax_energy.set_xlim(time_history[0], time_history[-1])
    ax_energy.set_ylim(0,np.max(energy_history[:,i])+1)
    for i in range(number_modules):
        lines_energy[i].set_xdata(time_history)
        lines_energy[i].set_ydata(energy_history[:,i])
plt.show()