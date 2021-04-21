# Oh-My-LOAM

# How to run
## BUILD

Install dependences (listed below).\
Clone this repository\
Compile: 
```bash
mkdir build && cd build
cmake ..
make -j6
```
# Dependences

### OS
Tested on ubuntu 16.04/18.04/20.04.

### C++17
If cannot find *std::filesystem* error is encountered during your compiling, please upgrade your compiler. 
We recommend `g++-9` (or higher version).

### ROS

### Eigen: linear algebra, quaternion
```
sudo apt install libeigen3-dev
```

### g3log: logging
Follow [g3log](https://github.com/KjellKod/g3log) to install.

### yaml-cpp: yaml parsing
```
sudo apt install libyaml-cpp-dev
```

### ceres: non-linear optimization
```
sudo apt install libceres-dev
```