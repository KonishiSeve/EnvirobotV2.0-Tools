## Envirobot V2.0 Plotter
This program can plot a real-time model of Envirobot V2.0. The data can either be directly computed or read from a log file.

### Compute CPG locally and plot
To plot CPG steps computed directly, the plotter can be started with this command: **python Plotter.py**

There are two options for the CPG controller, a Python implementation or a C++ implementation (linked to the plotter with Ctypes).
"CPG.py" and "CPG.cpp" contain a class that implements the CPG controller. The "python_link.cpp" file is used for the Python to C++ bridging. These two .cpp files (and the .hpp file) are compiled into a .dll to be able to use it with Ctypes.
The "CPP_CPG.py" implements the same class as the "CPG.py" file but is using Ctypes and the .dll file to make all the CPG computation.
This feature is particularly usefull as the C++ implementation (CPG.cpp and CPG.hpp) can directly be used in the STM32CubeIDE Envirobot project without modification as long as the inputs and outputs of the CPG class did not change.

Once the plotter is started, if the "plot_robot_pose" option is enabled, a real-time plot of the robot pose will be shown. At this point, the user also has access to a shell and commands can be entered to modify CPG parameters and see the results in real time.
Once the user stopped the plotter or the max "duration" has been hit. Plots of the CPG states of all the oscillators is shown if the "plot_cpg_states" is enabled.
A .csv log file, is created if the "file_save" option is enabled.


### Read CPG from a file and plot
To plot from .csv log file, the plotter can be started with this command: **python Plotter.py logfile.csv**
If the "plot_robot_pose" option is enabled, a real-time plot of the robot pose will be shown. If the log file came from the real robot, "plot_power" and "plot_energy" can be enabled to also plot the power and energy consumption in real-time with a configurable sliding window.

Once the end of the log file is reached, the full power and energy consumption plots are shown (if enabled). If "plot_robot_pose" is not enabled, no real-time plotting will happen and the plotter will directly jump to this step.

It is not possible to plot the CPG oscillator states when reading from a log file.

If the .csv log file was created by the CM4 logger, there might be long pauses where nothing seems to happen. This is due to the fact that the log starts logging as soon as the robot is started (with the REG_REMOTE_MODE register). If the user waited some time between the remote starting the robot and pushing the joystick forward, this delay will be "shown" by the plotter.