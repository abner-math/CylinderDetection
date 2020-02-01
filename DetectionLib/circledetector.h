#ifndef NEWALGORITHM_H
#define NEWALGORITHM_H

#include "primitivedetector.h"
#include "circle.h"
#include "rect.h"

class CircleDetector : public PrimitiveDetector<2, Circle>
{
public:
    CircleDetector(const PointCloud2d *pointCloud)
        : PrimitiveDetector<2, Circle>(pointCloud)
    {

    }

    std::set<Circle*> detect() override;

    static bool isValid(std::map<int, int> &angles);

private:
    struct EstimatedCenter
    {
        Rect2d estimatedCenter;
        float maxRadius;
        float minRadius;
        float radius;
    };

    Rect2d getExtension();

    Circle* estimateCircle(size_t p1, size_t p2, size_t p3);

    EstimatedCenter* estimateCenter(const std::vector<size_t> &indices);

    Circle* estimateCircle();

};

#endif // NEWALGORITHM_H
