# DroneFlightSystem

This repository contains the flight system for a drone that uses a Raspberry Pi as its flight controller. The repository is quite messy as many of the files were used for iterative testing (that's why there are so many pid files).

## Drone in action
### PID tuning 
![PID tuning](https://media.giphy.com/media/3TyZsBbAYOQViJGsa9/giphy.gif)

### Diagonal motor test
![Diagonal motor test](https://media.giphy.com/media/7MUJxguDErxrx9e7n8/giphy.gif)

*Sadly didn't take any videos with all motors running. Also, drone is definitely not suitable to fly unteathered as it will likely end up with some dead*

## Components:

- Raspberry Pi
- Hobbywing X-Rotor 60A 4 in 1 ESC
- X-Power 4S 5000mAh Battery
- Multistar 2834-800KV
- Multirotor Carbon Fiber Propeller 14x4.7
- AdaFruit 16 Channel PWM Hat
- DX6 RC Controller 

*Note: PID control may be erratic due to a noisy derivative.*
