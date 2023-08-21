# Instructions for using the emulator

Since we do not have the Robot physically, we will do it with an emulator. This emulator us
allows:

- Control two Bioloid AX12 type motors following the method described in the manualof these called endless turn (giving speed and direction of rotation of each of theengines).
- Represent the signal of the three "sensors" that would be in the AX-S1 module (values between 0-255).
- Emulate the joint movement that the two motors would represent on an axle and with a wheel "crazy", just as if they were mounted on the physical robot (Forward, Back, Right and Left), and simulating its position inside a room with obstacles, while measuring and indicating the distances to the obstacles detected by the sensors.

Dependencies: pyserial, python3-tk (tkinter), Pmw, numpy, matplotlib

This emulator consists of 3 windows:

- Control
- Plot
- Python console

Next, we will go on to explain each of these windows in detail.

## Control window

<div align="center">

![emulator](./public/emulator.png)

</div>

- **Debug level control**: in the first row, we have some buttons to control the amount of debugging information we want to see. These buttons enable/disable the presentation of this information by the python text console. By default they are disabled, because they slow down the movement simulation, but it can be very useful while we are debugging the communications between the MSP and the emulator.

  - **Debug Frames**: when we send a command from the MSP it will present the frame it receives the emulator, as well as the response sent by the emulator to the MSP (Status Packet).

  - **Debug Modules**: when we send a correct frame from the MSP, which has the effect of modification of the memory of a module, it will present the memory map of the modules AX12 (motors) and AX-S1 (sensors).

  - **Verbose**: if checked, presents additional information about the robot's activity (it is not a global enable/disable of the console).

  - **Print module memory**: can be pressed at any time to promptly print the memory map of the AX12 (motors) and AX-S1 (sensors) modules

- **Control of the simulation**: in the second row, we have some buttons to control the status of the simulation.

  - **Simul On/Off**: start (checked) or pause (unchecked) the simulation (robot movement). It is advisable to start the simulation just before starting the robot from the program the MSP

  - **Save**: If checked, saves the data with position and orientation to a file as it is they are simulating Therefore, it must be checked before starting the simulation (otherwise the dates would be missing simulated prior to dialing the button). This data, saved in the “movement.log” file they can be read again and represented with another independent script "Plot_Moviment.py".

  - **Reset**: resets all the parameters of the simulation, both the state of the robot and its position, as well as the data file. The robot stops, and returns to the initial position defined by default in the python script, discarding all points simulated so far. The data file is empty

- **Robot status**: below the control buttons, we find a message area to display the state of the robot at every moment.

  - **ROBOT STATUS**: indicates movements (Stop, Forward/Backward, Turn...)

  - **Left MOTOR, Right MOTOR**: speed and direction of rotation of each motor, state of the LEDs motors (on/off)

  - **IR Esq, Right, Center**: indicator bars that graphically show the distance to an obstacle which measures each of the 3 sensors, with values between 0 and 255. To the right of each bar, we can see the numerical value of these distances.

- **Serial port and bit rate configuration, refresh button**:

  These controls allow you to select and configure the USB port where the MSP UART is located. In Windows, the MSP should appear in the device manager, in the class

  - **COM and LPT ports**

  as in

  - **XDS110 Class Application/User UART (COMx)**

  In Linux the name of COM devices is usually of the type /dev/ttyACMx (equivalent to Windows we will have two serial ports, one to communicate with our application and another for the programmer).

  Usually, the same device in the same USB will always appear in the same one harbor So we can set the default Port by directly editing the python file, so you don't have to select it every time. This is the only python parameter that is allowed to be modified by you, given that the evaluation of the code you deliver is done with the same emulator for everyone and it will be the same as the one provided by the teachers. Like this, if the MSP is present when starting the emulator, it will automatically select it. If it is not present, we can plug it in anyway without having to close the emulator, and press the button "Refresh" to find and select it.

- **To go out**: exit the emulator, closing all windows, and terminating all threads and processes. (The cross from the top right of the window does not come out cleanly, which generates a whole series of error messages to the console, and should only be used in the event that one has occurred serious error that left the "Exit" button not working)

## Plot window:

Real-time visualization of the robot's path inside the room with its walls and the its obstacles At the bottom, there is a toolbar to zoom, save an image of the figure, etc..., and an indication of the amount of points simulated and drawn at all times.

<div align="center">

![plot](./public/plot.png)

</div>

A la següent figura, podem veure un exemple d’una imatge exportada des d’aquesta finestra de plot en format png, després de finalitzar amb èxit una simulació (evitant tots els obstacles sense cap col·lisió ni sortir de la habitació).

## Console

Various status messages (collision, leaving the room...), "Debug" messages according to the "Debug" options enabled in the control window and possible python error messages. In the latter case, inform your practice teachers, clearly showing the capturing the error and attaching the code you ran and possible steps followed in order to be able to- reproduce it

## Usage

1.  Modify the value of the **.env** file:

        PORT=COM4
        INITIAL_POS_X=2000
        INITIAL_POS_Y=2000
        PLOT_FILE=plot_movement.py
        ROOM_FILE=room.h
        LOG_FILE=movement.log
        LOCK_FILE=log.lock

2.  Installation required libraries:

        $ pip install -r requirements.txt

3.  start script:

    - run_emulator.bat (Windows)
    - run_emulator.sh (Linux)
