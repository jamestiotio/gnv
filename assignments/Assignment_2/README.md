# Assignment 2

> James Raphael Tiovalen / 1004555

## Setup Instructions

To setup the environment, simply run the following commands:

```bash
cd Assignment_2_linux/
rm -r build/
mkdir build/
cd build/
cmake ..
```

Then, to execute the program, simply run the following commands from the `build` directory (where `filename` is the filename of the SKEL file of interest):

```bash
make
./Assignment_2
Please enter filename.skel: ../data/filename.skel
```

## Demo and Description of Features

The three features implemented in this assignment are:

- Load Skeleton File
- Draw Skeleton
- Change Pose of Skeleton

### Load Skeleton File and Draw Skeleton

All included skeleton files in the [data](./Assignment_2_linux/data/) directory contains 13 joints each.

`Model1.skel`:

![model1_draw](./assets/draw_skeleton/model1_draw.gif)

`Model2.skel`:

![model2_draw](./assets/draw_skeleton/model2_draw.gif)

`Model3.skel`:

![model3_draw](./assets/draw_skeleton/model3_draw.gif)

`Model4.skel`:

![model4_draw](./assets/draw_skeleton/model4_draw.gif)

### Change Pose of Skeleton

`Model1.skel`:

![model1_draw](./assets/change_skeleton_pose/model1_change_pose.gif)

`Model2.skel`:

![model2_draw](./assets/change_skeleton_pose/model2_change_pose.gif)

`Model3.skel`:

![model3_draw](./assets/change_skeleton_pose/model3_change_pose.gif)

`Model4.skel`:

![model4_draw](./assets/change_skeleton_pose/model4_change_pose.gif)

