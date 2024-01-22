## About

![](https://www.inf.ufrgs.br/%7Eoliveira/pubs_files/CD/teaser.png)

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

Access our homepage to get access to the paper and datasets:

https://www.inf.ufrgs.br/%7Eoliveira/pubs_files/CD/CD.html

## Install 

This is a regular Qt project with no external dependencies. We use Eigen, but this is bundled in our code. Once the project is opened in Qt design you should be able to compile it right away.

The actual article implementation is located here:

`DetectionLib/cylinderdetector.cpp`

### Graphical Interface 

#### !! Important !!

**Before estimating a plane (in Plane Detector > Detect planes), you need to estimate normals (Plane Detector > Estimate normals).**

## Project structure

This project is structured into four main subprojects:
- CoreLib (contains the main classes such as Point, PointCloud, Cylinder...)
- DetectionLib (this is where the cylinder detection algorithm is implemented, see class CylinderDetector)
- GraphicsLib (OpenGL utilities used by the GUI)
- PointCloudEditor (the GUI. Run it to Load a point cloud, visualize it, detect cylinders and visualize detected cylinders, among many other possibilities...) 

## Evaluating technique performance 

Besides the Qt project, this repository also contains a project used to calculate some metrics (F1 score, recall, precision, etc.) for a given cylinder detection. Please, refer to: 

`CompareCylinderDetector`

