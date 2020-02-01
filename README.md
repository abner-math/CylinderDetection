## About

This is the implementation of the following article:

```
@article{AraujoOliveira_2020c,
  author={Abner M. Araujo and Manuel M. Oliveira},
  title={Connectivity-based Cylinder Detection in Unorganized Point Clouds},
  journal={Pattern Recognition},
  volume={100},
  pages={107161},
  year={2020},
  publisher={Elsevier}
}
```

If case you use it in our research, please cite our work.

## Install 

This is a regular Qt project with no additional dependencies. We use Eigen, but this is bundled in our code. Open the project on Qt design and you should be able to compile it right away.

The actual article implementation is located here:

`DetectionLib/cylinderdetector.cpp`

## Project structure

Basically, this project is structured in four main subprojects:
- CoreLib (contains the main classes such as Point, PointCloud, Cylinder...)
- DetectionLib (this is where the cylinder detection algorithm is implemented, see class CylinderDetector)
- GraphicsLib (OpenGL utilities used by the UI)
- PointCloudEditor (the UI. Run it to Load a point cloud, visualize it, detect cylinders and visualize detected cylinders, among many other possibilities...) 

## Evaluating technique performance 

Besides the Qt project, this repository also contains a small cpp file used to calculate metrics (F1 score, recall, precision, etc.) for a given detection. Please, refer to: 

`compare_cylinder_detector`

