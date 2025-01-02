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
import re
from LogLib import LogFile

#====================== #
#===== PARAMETERS ===== #
#====================== #
# === Hyper parameters === #
number_modules = 4      # number of modules (at least 1), does not include the tail

# === CPG parameters === #
frequency = 0.5           # oscillator frequency
direction = 0           # turning direction (positive/negative) and amplitude
amplc = 0.3             # defines the amplitude of the first joint (closest to head)
amplh = 0.5             # defines the amplitude of the last joint (furthest from head)
nwave = 1               # number of waves peaks that can be seen on the robot at the same time
coupling_strength = 50  # speed at which the phase difference between coupled oscillators converges
a_r = 10                # speed at which the amplitudes of the oscillators converge

# === Computing parameters === #
use_cpp_version = True # use the C++ implementation that is compiled into a .dll
file_save = False        # saves the joint setpoints and timebase to a csv file
delta_ms = 10            # integration stepsize in milliseconds (lower is more expensive)

# === Plotting parameters === #
plot_realtime = True    # enable animation
fps = 30                # animation frames per seconds (higher is more expensive)
speed = 1               # animation speed multiplier (higher is more expensive)
duration = 120           # animation duration in seconds

#============================#
#===== GLOBAL VARIABLES =====#
#============================#
#CPG controller (python or C++ version)
if use_cpp_version:
    controller = CPP_CPG(number_modules, frequency, direction, amplc, amplh, nwave, coupling_strength, a_r)
else:
    controller = CPG(number_modules, frequency, direction, amplc, amplh, nwave, coupling_strength, a_r)

#Joint setpoints to render, filled by the CPG thread
shell_queue = queue.Queue()
render_queue = queue.Queue()

#stop flag
stop = False

#===================== #
#====== THREADS ====== #
#===================== #
#shell thread to interact with the controller and plotter
def shell_thread():
    global stop, controller
    while not stop:
        command = input("[CPG]$ ")
        if command == "stop":
            stop = True
            break
        elif len(command) == 0:
            continue
        elif len(command.split(" ")) == 2:
            shell_queue.put(command)
            name = command.split(" ")[0]
            value = float(command.split(" ")[1])
            if name == "number_modules":
                controller.set_number_modules(value)
            elif name == "freq":
                controller.set_frequency(value)
            elif name == "dir":
                controller.set_direction(value)
            elif name == "amplc":
                controller.set_amplc(value)
            elif name == "amplh":
                controller.set_amplh(value)
            elif name == "nwave":
                controller.set_nwave(value)
            elif name == "cs":
                controller.set_coupling_strength(value)
            elif name == "a_r":
                controller.set_a_r(value)
            else:
                print("Unrecognized variable name")
        else:
            print("Unrecognized command")

#CPG computation thread
def cpg_thread():
    #to manage the thread
    global stop, controller, delta_ms, number_modules
    stop_cpg = False

    #open input log file
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
        print("Detected {0} modules\n[CPG]$ ".format(number_modules), end="")

    #create output log file
    elif len(sys.argv) == 1 and file_save:
        now = datetime.now()
        output_file = LogFile()
        output_file.new(now.strftime("exports/%d_%m_%Y_%H_%M_%S.csv"), ["time", "print"] + ["joint{0}".format(i) for i in range(number_modules)])

    #initialization
    robot_states = {"time": None, "joint": None, "energy": None, "power": None}
    robot_events = {"print": None}
    joint_setpoints = np.zeros(number_modules)  #joint angle for each module, given by CPG (in the module frame = motor position setpoints)
    t = 0

    #read first line to get the first timestamp
    if len(sys.argv) == 2:
        raw_data = input_file.read()
        t = int(raw_data["time"])
    next_frame = t

    #run the loop
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
                #save to file
                if len(sys.argv) == 1 and file_save:
                    output_data = {}
                    output_data["time"] = t
                    for i in range(number_modules):
                        output_data["joint{0}".format(i)] = str(int(np.rad2deg(joint_setpoints)[i]))
                    if shell_queue.qsize() > 0:
                        output_data["print"] = shell_queue.get()
                    output_file.write(output_data)
                t += delta_ms

                #stop the rendering if above the max simulation duration
                if t >= duration*1000:
                    render_queue.put(None)
                    stop_cpg = True
                    break

            # == Read a new line from the file == #
            else:
                #print(raw_data)
                robot_states["time"] = int(raw_data["time"])
                robot_states["joint"] = []
                robot_states["energy"] = []
                robot_states["power"] = []
                for i in range(number_modules):
                    robot_states["joint"].append(float(raw_data["joint{0}".format(i)]))
                    robot_states["energy"].append(float(raw_data["energy{0}".format(i)]))
                    robot_states["power"].append(float(raw_data["power{0}".format(i)]))
                robot_states["joint"] = np.deg2rad(np.array(robot_states["joint"]))
                robot_states["energy"] = np.array(robot_states["energy"])
                robot_states["power"] = np.array(robot_states["power"])
                if "print" in raw_data:
                    robot_events["print"] = raw_data["print"]

                #grab next timestamp to know until when the current states should be shown
                raw_data = input_file.read()
                if raw_data == None:
                    #render last frame
                    render_queue.put([robot_states,robot_events])
                    #end of file, tell the renderer to stop
                    render_queue.put(None)
                    stop_cpg = True
                    break
                delta_ms = t - int(raw_data["time"])
                t = int(raw_data["time"])

            # == Add the data to the rendering queue == #
            while plot_realtime and (t >= next_frame):
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

cpg_thread_handle = threading.Thread(target=cpg_thread)
cpg_thread_handle.start()

#wait for some points to be added to the render queue before starting to plot them
time.sleep(0.5)

#==================== #
#===== Plotting ===== #
#==================== #
if number_modules < 2:
    raise(Exception("Need at least 2 modules"))

#running the loop
start = time.time()
fps_warning_counter = 0
module_absolute_angles = np.zeros(number_modules) #angle at which the module is in the global reference frame
joint_positions = np.zeros((number_modules, 2))   #position in space of each joint (each module is 1 unit long) in the global reference frame

debug_counter = 0
debug_list = []

fig, ax = plt.subplots()
axis_lenth = number_modules+2
ax.set_xlim(-1, axis_lenth-1)
ax.set_ylim(-axis_lenth/2,axis_lenth/2)

line, = ax.plot(np.random.randn(100), np.random.randn(100))
points = ax.scatter(np.random.randn(100), np.random.randn(100))
#plt.show(block=False)

tstart = time.time()
num_plots = 0

while(not stop) and plot_realtime:
    #wait for queue to fill (should normally never be empty)
    while(render_queue.empty() and (not stop)):
        print("[Warning] desired framerate not achieved, lower the framerate, speed or delta_t")
        time.sleep(1)
    
    #retrieve what the cpg thread computed
    raw_queue = render_queue.get()

    #end of animation
    if raw_queue is None:
        stop = True
        print("Animation done\n[CPG]$ - press enter to exit -", end="")
        break

    robot_states = raw_queue[0]
    robot_events = raw_queue[1]
    joint_setpoints = robot_states["joint"]
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
    
    line.set_xdata(np.concatenate((np.zeros(2), joint_positions[:,0]), axis=None))
    line.set_ydata(np.concatenate((np.zeros(2), joint_positions[:,1]), axis=None))
    points.set_offsets(np.column_stack((joint_positions[:,0],joint_positions[:,1])))
    fig.canvas.flush_events()
    plt.show(block=False)
    time.sleep(max((1/fps) - (time.time()-start-0.001), 0.001))
    start = time.time()
plt.clf()

# == add your plotting code here ==