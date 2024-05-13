# Approximate Boxes

Approximate Boxes is a small C++ project that intends to approximate a volume with cubes using octrees. 
It leverages the power of the Computational Geometry Algorithms Library (CGAL) to perform this task.

## Features

- Approximation of a volume provided by an OFF file with cubes using octrees
- Saving the boxes as hexahedral elements in the Gmsh file format

## Prerequisites

- C++ compiler with C++17 support
- CMake (version 3.10 or higher)
- CGAL library (Download and install from [here](https://www.cgal.org/download.html))

## Building the Project

1. Clone the repository:
    ```bash
    git clone https://github.com/paulotto/approximate_boxes.git
    ```

2. Navigate to the project directory:
    ```bash
    cd approximate_boxes
    ```
3. Create a build directory and navigate to it:
    ```bash
    mkdir build && cd build
    ```

4. Run CMake and build the project:
    ```bash
    cmake ..
    make
    ```
## Usage

After building the project, you can run the example program with the following command:
```bash
./bin/ApproximateBoxesExample "../resources/ellipsoid.off"
```
This command will generate a Gmsh file with the boxes approximating the volume of the ellipsoid, 
placed in the [resources](./resources) folder. 

>**Note**: The degree of approximation is primarily determined by the number of nodes used for the surface in the 
>OFF file. The more nodes, the more accurate the approximation will be and the smaller the boxes.
>Furthermore, the approximation can be influenced by the 
>``` ApproximateBoxes<>::SetOctreeMaxDepth(unsigned int depth) ``` and 
>``` ApproximateBoxes<>::SetOctreeMaxInliers(unsigned int inliers) ``` functions.

To have a completely connected mesh, it is necessary to divide the larger boxes into smaller ones so that every box
has the same dimensions. This can be done by setting the 
``` ApproximateBoxes<>::SetDivideBoxes(bool divide) ``` function to true (default: true). Below you can see an example
where the ellipsoid is approximated with and without dividing the boxes.

>**Note**: If the boxes are divided, the number of boxes will increase significantly, 
>leading to longer computation times.

### Ellipsoid Approximation without divided Boxes
| ![](resources/ellipsoid_orig.png) | ![](resources/ellipsoid_orig_approx.png) | ![](resources/ellipsoid_approx.png) |
|:---------------------------------:|:----------------------------------------:|:-----------------------------------:|  
|             Ellipsoid             |        Ellipsoid + Approximation         |          Approximation Cut          |

### Ellipsoid Approximation with divided Boxes
| ![](resources/ellipsoid_orig.png) | ![](resources/ellipsoid_orig_approx_div.png) | ![](resources/ellipsoid_approx_div.png) |
|:---------------------------------:|:--------------------------------------------:|:---------------------------------------:|  
|             Ellipsoid             |          Ellipsoid + Approximation           |            Approximation Cut            |

## License
This project is licensed under the LGPL-3.0 License. See the [LICENSE](LICENSE) file for more details.
