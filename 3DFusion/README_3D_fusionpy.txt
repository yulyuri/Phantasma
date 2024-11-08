3D Orientation Visualization for STM32 Device - Instructions

Follow the steps below to set up and run the Python script for real-time 3D orientation of the STM32 board using sensor data from the serial port.

1. Install Required Python Libraries:
   First, install the necessary libraries to run the Python script. Open your terminal or command prompt and run the following:


If `pip` is not installed on your machine, follow these steps to install it:
curl https://bootstrap.pypa.io/get-pip.py -o get-pip.py python get-pip.py


2. Configure Serial Port in Python Code:
Open the Python file in a text editor. On the 10th line of the Python code, change the serial port to match the correct one for your system.
- For example, if you are using Windows, update it to `COM3`, or for Linux, it could be `/dev/ttyUSB0`.

3. Run the `main.c` File on STM32:
Open STM32CubeIDE and load the `main.c` file into your project.
Build and flash the code to your STM32 board.

4. Ensure Serial Port Availability:
Before running the Python script, make sure no other programs are using the UART port (e.g., close Tera Term or any terminal program).
This avoids potential clashes when trying to access the serial port in the Python script.

5. Enter Ghost Capture Mode:
Once the STM32 is running, enter the Ghost Capture Mode by pressing the appropriate button as per the `main.c` file instructions.

6. Run the Python Script:
In your terminal or command prompt, run the Python file to visualize the real-time orientation data:


7. Visualize 3D Orientation:
Rotate the STM32 board, and you should see the 3D model rotate in real-time on your computer screen, reflecting the movement of the device.

Enjoy experimenting with the 3D orientation visualization!


