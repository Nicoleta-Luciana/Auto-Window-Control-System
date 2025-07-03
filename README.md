# Auto Window Control System using Raspberry Pi

This project simulates an automotive window control system using a Raspberry Pi, a motor driver (Pololu 2961), a DC motor, a 3-way switch, and a proximity sensor. Additionally, it features an I2C LCD display that shows the selected direction of movement.

## Features

- **Window open/close control**: Manually operated using a 3-position switch (left/right/stop).
- **User safety mechanism**: If an obstacle is detected while the window is closing, the motor stops and reverses direction.
- **Visual feedback**: An I2C LCD screen displays the chosen direction ("Left" / "Right").

## Components Used

- Raspberry Pi  
- Pololu 2961 Motor Driver  
- DC Motor  
- 3-way Switch  
- Proximity Sensor  
- 20x4 LCD with I2C interface  
- External 9V Power Supply  

## Code Structure

The Python code (`motorcontrol.py`) includes:

- Initialization of GPIO pins and hardware components  
- Functions for motor control (rotation and stop)  
- LCD control via the I2C protocol  
- Proximity sensor monitoring for obstacle detection  
- A main loop that continuously checks inputs and reacts accordingly  

## Circuit Diagram

The final circuit diagram is included in this repository to illustrate the physical connections and block-level logic between components.

## Notes

- The motor requires an external 9V power supply; the Raspberry Pi alone cannot provide sufficient power.  
- The original LCD with parallel pins was replaced by an I2C-compatible version for simpler and more reliable connections.
