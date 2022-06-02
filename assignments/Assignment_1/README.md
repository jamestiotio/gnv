# Assignment 1

> James Raphael Tiovalen / 1004555

## Setup Instructions

To setup the environment, simply run the following commands:

```bash
cd Assignment_1_linux/
rm -r build/
mkdir build/
cd build/
cmake ..
```

Then, to execute the program, simply run the following commands from the `build` directory (where `filename` is the filename of the OBJ file of interest):

```bash
make
./Assignment_1 ../data/filename.obj
```

## Demo and Description of Features

### Mesh Loading

`garg.obj`:

- 21278 vertices
- 21278 normals
- 42552 faces

`mickey.obj`:

- 3502 vertices
- 3502 normals
- 7000 faces

`sphere.obj`:

- 382 vertices
- 382 normals
- 760 faces

### Mesh Display

`garg.obj`:

![mesh_display_garg](./assets/mesh_display/mesh_display_garg.png)

`mickey.obj`:

![mesh_display_mickey](./assets/mesh_display/mesh_display_mickey.png)

`sphere.obj`:

![mesh_display_sphere](./assets/mesh_display/mesh_display_sphere.png)

### Mesh Coloring

`garg.obj`:

![mesh_coloring_garg](./assets/mesh_coloring/mesh_coloring_garg.gif)

`mickey.obj`:

![mesh_coloring_mickey](./assets/mesh_coloring/mesh_coloring_mickey.gif)

`sphere.obj`:

![mesh_coloring_sphere](./assets/mesh_coloring/mesh_coloring_sphere.gif)

### Mesh Transformation

`garg.obj`:

![mesh_transformation_garg](./assets/mesh_transformation/mesh_transformation_garg.gif)

`mickey.obj`:

![mesh_transformation_mickey](./assets/mesh_transformation/mesh_transformation_mickey.gif)

`sphere.obj`:

![mesh_transformation_sphere](./assets/mesh_transformation/mesh_transformation_sphere.gif)
