#ifndef CYLINDERDETECTOR_H
#define CYLINDERDETECTOR_H

#include "circledetector.h"
#include "cylinder.h"

class CylinderDetector : public PrimitiveDetector<3, Cylinder>
{
public:
    CylinderDetector(const PointCloud3d *pointCloud)
        : PrimitiveDetector<3, Cylinder>(pointCloud)
    {

    }

    Cylinder* detectCylinder(const std::vector<size_t> &points, bool leastSquares = true);

    std::set<Cylinder*> detect() override;

private:
    struct ConnectedComponent
    {
        Eigen::Vector3f direction;
        std::vector<size_t> indices;
        std::vector<Eigen::Vector2f> positions;
        std::vector<Eigen::Vector2f> normals;
    };

    Eigen::Vector3f getMedianVector(const std::vector<Eigen::Vector3f> &v);

    Eigen::Vector3f getMedianNormal(const std::vector<size_t> &indices);

    void getEvenlyDistributedDirections(std::vector<Eigen::Vector3f> &directions);

    PointCloud2d* getProjectedPointCloud(const ConnectedComponent *component);

    Eigen::Vector3f getMeanDirection(const std::vector<size_t> &indices);

    void findConnectedComponents(const Eigen::Vector3f &direction, std::vector<ConnectedComponent*> &components);

    bool findSubcomponents(ConnectedComponent *component, std::vector<ConnectedComponent*> &subcomponents);

    void updateComponents(size_t begin, std::vector<ConnectedComponent*> &components);

    void fitCylinder(Cylinder *cylinder);

    void removePoints(Cylinder *cylinder);

    void mergeCylinders(std::vector<Cylinder*> &cylinders);

    bool isCylinderValid(Cylinder *cylinder);

};

#endif // CYLINDERDETECTOR_H
