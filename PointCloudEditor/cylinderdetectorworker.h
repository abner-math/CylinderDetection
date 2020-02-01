#ifndef PLANEDETECTORWORKER_H
#define PLANEDETECTORWORKER_H

#include "worker.h"
#include "cylinderdetector.h"

class CylinderDetectorWorker : public Worker
{
public:
    CylinderDetectorWorker(const PointCloud3d *pointCloud);

    ~CylinderDetectorWorker();

    void pointCloud(const PointCloud3d *pointCloud);

    std::vector<Cylinder*> cylinders() const
    {
        return mCylinders;
    }

    CylinderDetector* detector()
    {
        return mDetector;
    }

private:
    CylinderDetector *mDetector;
    std::vector<Cylinder*> mCylinders;

    void actions() override;

};

#endif // PLANEDETECTORWORKER_H
