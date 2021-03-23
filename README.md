# Cloth-Simulator

[![Build Status](https://travis-ci.com/Gan-Tu/Cloth-Simulator.svg?branch=master)](https://travis-ci.com/Gan-Tu/Cloth-Simulator)

A real-time cloth simulation engine

## Code Protection

Upon request of the course staff, the source code will not be shown publicly on GitHub. The source code is currently password protected, zipped in `src.zip`. To request for the password for unzipping the source file, send me an email.

## Write-Up
You can read my detailed [writeup](https://michael-tu.github.io/Cloth-Simulator/) to understand what features I have implemented, how I developed them, see many of the cool results, and view side-by-side comparisons of the performance by different techniques.

## Glimpse of Results

![Demo](docs/assets/img/demo.png)

**Animation**

_Note: the following animation may take a while to load before it starts moving_

![Animated Demo](docs/assets/img/demo2.gif)

## Running Cloth Simulator

Running the program with zero arguments will load up the default scene (`scene/pinned2.json`). 

```
./clothsim
```

Otherwise, to load a specific scene, run the program as follows:

```
./clothsim -f ../scene/<my_scene>.json
```

### Specific Animations

To load a hanging cloth, run:

```
./clothsim -f ../scene/pinned2.json
```

To load a pinned cloth, run:

```
./clothsim -f ../scene/pinned4.json
```

To load a cloth falling on to a sphere, run:

```
./clothsim -f ../scene/sphere.json
```

To load a cloth falling on to a plane, run:

```
./clothsim -f ../scene/plane.json
```

To load a cloth falling onto itself, run:

```
./clothsim -f ../scene/selfCollision.json
```

## Using the GUI

Besides the actual graphical user interface that you can work with using your mouse to tweak cloth parameters and dynamically change external forces, there are several **keyboard commands** that you might find useful.

Command | Key
------- | -----
Pause simulation    | P
(while paused) Go forward 1 timestep   |  N
Restart simulation  | R
End simulation |  ESCAPE
Reset camera to default position   |  SPACE
Rotate camera  |  (click and drag, or right click)
Pan camera  | (hold control + click and drag, or right click)

## Build System

If you don't have [CMake](https://cmake.org) (version >= 2.8) on your personal computer, you can install it using `apt-get` on Linux or `Macports/Homebrew` on OS X. Alternatively, you can download it directly from the CMake website.

To build the code, start in the folder that GitHub made or that was created when you unzipped the download. 

#####
You will need to unzip the `src.zip` to get the source code first!

Upon request of the course staff, the source code will not be shown publicly on GitHub. The source code is currently password protected, zipped in `src.zip`. To request for the password for unzipping the source file, send me an email.
#####

Run
```
mkdir build; cd build
```

to create a build directory and enter it, then

```
cmake ..
```

to have CMake generate the appropriate Makefiles for your system, then

```
make 
```

to make the executable, which will be deposited in the build directory.
