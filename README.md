**Introduction**

An institution required me to design a control system to control a valve, as depicted in Fig 1.

![image](https://user-images.githubusercontent.com/67689632/200152182-84769f47-bab9-4c80-9d90-2d9b27813581.png)
Fig 1

This system should meet the following requirement:

1.	It can read the position of motor and ADC value in a real time manner and show these values on the touch panel

2.	User may set target air pressure or flux value and the control system should receive that command sent via touch panel and actuate motor to the target position where the air pressure or flux value is the require one.

**Implementation**

Since the control system should simultaneously read ADC values and send them to user, receive and process commands from touch panel, and communicate with motor, a Real Time Operating System (RTOS) RtThread(https://www.rt-thread.io/) was adopted as the framework of the software. The RtThread can facilitate thread creation and it provided many useful thread synchronization mechanism.

Every task was assigned to a dedicated thread, and all threads shared data through a global data structure. A very sketchy demonstration of this software is shown in Fig 2.

![image](https://user-images.githubusercontent.com/67689632/200152183-9e4515f8-c815-4eff-8dfd-0b7affebc3b3.png)
Fig 2

All code files are in **application** folder

The **main.cpp** file in the application folder is the entry point of the whole system, in which several threads are launched and some thread synchronization data structures are initialized. RtThread core helps schedule threads, dispatch hardware interruptions and synchronize thread operations. To facilitate data sharing, all threads publish data into a shared data pool.
