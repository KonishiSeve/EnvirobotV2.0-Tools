#Virtual COM Port Spy
This folder contains a Python script used to spy on the VCP interface between a program and a peripheral. This was used in the Envirobot project to reverse engineer the protocol used between the Radio PC client V1.5 and the USB radio dongle.

This client requires the installation of the “com0com” software. This will create 2 virtual COM ports (normally COM4 and COM5) and send all the bytes coming into one of them to the other.

The software to spy on must be setup to use the COM5 port. “COM3” in the Python script should be replaced by the COM port of the peripheral being spied on. As an example, in the Envirobot project, The USB radio dongle was on COM3 so no need for change and the PC client V1.5 was started with this command: **.\setwlch.exe /ch:81 /reg /port:COM5**

The Python scripts then redirects every byte coming from COM3 to COM4 (which is then sent to COM5 by “com0com”) and the same in the other direction. In parallel, it prints the bytes that pass through it with timestamps.
