# Assignment 3

> James Raphael Tiovalen / 1004555

## Setup Instructions

To setup the environment, simply run the following commands:

```bash
cd Assignment_3_linux/
rm -r build/
mkdir build/
cd build/
cmake ..
```

Then, to execute the program, simply run the following commands from the `build` directory (where `texture_filename` is the filename of the texture image and `obj_filename` is the filename of the OBJ file):

```bash
make
./Assignment_3 ../data/texture_filename.png
Please enter filename.obj: ../data/obj_filename.obj
```

## Demo and Description of Features

The three texture mapping features implemented in this assignment are:

- Planar Parameterization
- Cylindrical Parameterization
- Spherical Parameterization

### `sphere.obj`

Using `texture.png`:

![sphere_checkerboard](./assets/sphere/sphere_checkerboard.gif)

Using `texture_wood.png`:

![sphere_wood](./assets/sphere/sphere_wood.gif)

Using `texture_stone.png`:

![sphere_stone](./assets/sphere/sphere_stone.gif)

Using `texture_scene.png`:

![sphere_scene](./assets/sphere/sphere_scene.gif)

### `bunny.obj`

Using `texture.png`:

![bunny_checkerboard](./assets/bunny/bunny_checkerboard.gif)

Using `texture_wood.png`:

![bunny_wood](./assets/bunny/bunny_wood.gif)

Using `texture_stone.png`:

![bunny_stone](./assets/bunny/bunny_stone.gif)

Using `texture_scene.png`:

![bunny_scene](./assets/bunny/bunny_scene.gif)

### `garg.obj`

Using `texture.png`:

![garg_checkerboard](./assets/garg/garg_checkerboard.gif)

Using `texture_wood.png`:

![garg_wood](./assets/garg/garg_wood.gif)

Using `texture_stone.png`:

![garg_stone](./assets/garg/garg_stone.gif)

Using `texture_scene.png`:

![garg_scene](./assets/garg/garg_scene.gif)
