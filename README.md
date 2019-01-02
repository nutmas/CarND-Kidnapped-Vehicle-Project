# Kidnapped Vehicle
## Localisation with a 2 dimensional particle filter.

---

[//]: # (Image References)

[image1]: ./support/SimulatorStartup.png "Simulator Startup Window"
[image2]: ./support/ParticleFilter.png " Particle Filter Start"
[image3]: ./support/ParticleFilterResult.png " Simulator Results"


## Overview
Implementation of a 2 dimensional particle filter in C++, which localises a vehicles position by being provided some initial localisation from GPS information, and then at each time step a varying of measurements to observed landmarks. The particle filter ties these observations and a known map of landmarks to understand where the vehicles location is at each cycle.

The basic process of the particle filter is:

1. Initialise Vehicle position to a number of possible positions.
2. Prediction
    * Transform car sensor coordinates to landmark coordinates
    * Associate transformed observations to nearest map landmarks
3. Update Particle weight
    * Apply multivariate gaussian probability for each measurement
    * Combine probabilities into a normalised weight
    * Calculated weight is posterior probability for each particle


The simulated data is provided over WebSocket from the Udacity Term 2 Simulator. The `main.cpp` calls functions within `particle_filter.cpp` which process the observations and predict the vehicles location. The prediction is passed back to the simulator and displayed with the sensor measurements and annotation.

---

## Installation steps

To run this code the following downloads are required:

1. Make a project directory `mkdir project_udacity && cd project_udacity`
2. Clone this repository into the project_udacity directory. `git clone https://github.com/nutmas/CarND-Kidnapped-Vehicle-Project.git`
3. Setup environment. `cd CarND-Kidnapped-Vehicle-Project\` and launch the script to install uWebSocketIO `./install-mac.sh`. Alternatively for Ubuntu installs launch `./install-ubuntu.sh`. The environment will be installed with these scripts.
4. Download Term 2 Simulator: 
      * Download from here: [Udacity Simulator](https://github.com/udacity/self-driving-car-sim/releases).
      * Place the extracted folder into the project directory. 
      * Within the folder run simulator `term2_sim`; if successful the following window should appear:
      ![alt text][image1]

---

## Other Important Dependencies

* cmake >= 3.5
* make >= 4.1 
* gcc/g++ >= 5.4

---

## Build the code

1. From the project_udacity folder change to folder created from cloning `cd CarND-Kidnapped-Vehicle-Project`
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make` 

---

## Usage

After running the script, confirming the operation of the simulator and building the code the Particle Filter program is ready to be run.

1. From terminal window; change to build folder of project `cd ~/project_udacity/CarND-Kidnapped-Vehicle-Project/build/`
2. Run Particle Filter: `./particle_filter `
3. Message appears `Listening to port 4567` the Particle filter is now running and waiting for connection from the simulator
4. Run the simulator `term2_sim`
5. In the simulator window, press 'Next' arrow twice to navigate to kidnapped vehicle then press 'Select'.
6. In the Particle Filter terminal window `Connected!!!` will now appear.
7. The Particle Filter is now fully operational.

        * Press Start

The Animation will start and the vehicle can be seen moving around the screen.

![alt text][image2]

* Blue lines are predicted measurements from particle filter
* Green lines are ground truth observations
* The ego vehicle is the blue car
* Map landmarks are shown with a cross in a circle symbol.

---

## Results

The Simulator evaluates the performance of the filter and after a number of cycles it will display a message informing the filter implementation success.

The success criteria is:

1. **Accuracy**: Particle filter localises the vehicle position and yaw to within the values specified in the parameters `max_translation_error` and `max_yaw_error` in `src/main.cpp`.

2. **Performance**: Particle filter should complete execution within the time of 100 seconds.

Message displayed with successful implementation:
```
Success! Your particle filter passed!
```

![alt text][image3]

---

## Streamline Build and Execution

By running these scripts form the top directory of the project the compiling and running of the project can be simplified:

1. ./clean.sh
2. ./build.sh
3. ./run.sh


---

## License

For License information please see the [LICENSE](./LICENSE) file for details

---

