#include "cylinderdetectorworker.h"

CylinderDetectorWorker::CylinderDetectorWorker(const PointCloud3d *pointCloud)
    : mDetector(new CylinderDetector(pointCloud))
{

}

CylinderDetectorWorker::~CylinderDetectorWorker()
{
    delete mDetector;
}

void CylinderDetectorWorker::pointCloud(const PointCloud3d *pointCloud)
{
    mDetector->pointCloud(pointCloud);
}

void CylinderDetectorWorker::actions()
{
    emit workerStatus(QString("Detecting cylinders..."));
    std::set<Cylinder*> cylinders = mDetector->detect();
    emit workerStatus(QString("Done."));
    mCylinders = std::vector<Cylinder*>(cylinders.begin(), cylinders.end());
}

