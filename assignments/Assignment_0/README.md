# Assignment 0 Setup Tutorial

> James Raphael Tiovalen / 1004555

## For Mac

### Step 0 -- Preparation

- Download and install CMake for Mac OS: https://cmake.org/download/

- Download and install Homebrew: https://brew.sh/ 

- Install the GLFW libraries using Homebrew: `brew install GLFW`

### Step 1 -- Compile the project

Input the following commands on the Terminal: 
1. cd to the folder: `Assignment_0_mac/build`
2. `cmake ..`
3. `make`
4. `./Assignment_0`

## For Linux (Ubuntu)

### Step 0 -- Preparation

Simply run these commands to install all the necessary dependencies:

```bash
sudo apt update
sudo apt install cmake libglfw3 libglfw3-dev
```

### Step 1 -- Compile the project

Simply run these commands:

```bash
cd Assignment_0_linux/
mkdir build
cd build/
cmake ..
make
./Assignment_0
```

After modification, just re-run `make` from inside the `build` directory and re-execute the `Assignment_0` binary executable.

## For Windows

### Step 0 -- Preparation

- Download and install Visual Studio: https://visualstudio.microsoft.com/

- Download the pre-compiled GLFW (click "32-bit Windows binaries"): https://www.glfw.org/download.html. 

- Choose glfw32.lib file according to your Visual Studio version.

- Copy the glfw32.lib file to the libs folder in your project.

### Step 1 --- Visual Studio Setup

1. Open the `Assignment_0_win.sln` file in `Assignment_0_win` folder with your Visual Studio.
2. Go to Project Properties -> Configuration Properties -> C/C++ -> General -> Additional Include Directories, select the includes folder in your project.
3. Go to Project Properties -> Configuration Properties -> Linker -> General -> Additional Library Directories, select the libs folder in your project.
4. Go to Project Properties -> Configuration Properties -> Linker -> Input -> Additional Dependencies and type: glfw3.lib and opengl32.lib. (We have already done it for you.)

### Step 2 --- Compile and run the project

- In Visual Studio, click `Build -> Build Solution` to compile the project.

- Then, click `Debug -> Start Without Debugging` to run the project.
