#include "circledetector.h"
#include "angleutils.h"
#include "collisiondetector.h"
#include "statisticsutils.h"

#include <iostream>

#define ANGLE_RANGE 5
#define MIN_NUM_ANGLES 18
#define MIN_ARC_LENGTH 90
#define MIN_DEPTH 5
#define MAX_R_SCORE 3
#define NUM_SAMPLES 720

class QuadtreeDetector
{
public:
    QuadtreeDetector(const PointCloud2d *pointCloud, const Eigen::Vector2f &center, float size, QuadtreeDetector *parent = NULL)
        : mPointCloud(pointCloud)
        , mParent(parent)
        , mCenter(center)
        , mSize(size)
        , mRect(Eigen::Vector2f(center.x() - size, center.y() - size), Eigen::Vector2f(center.x() + size, center.y() + size))
        , mLeaf(true)
        , mDepth(0)
        , mArcLength(0)
    {
        for (size_t i = 0; i < 4; i++)
        {
            mChild[i] = NULL;
        }
    }

    ~QuadtreeDetector()
    {
        for (size_t i = 0; i < 4; i++)
        {
            if (mChild[i] != NULL)
            {
                delete mChild[i];
            }
        }
    }

    void clear()
    {
        mIndices.clear();
        mAngles.clear();
        mArcLength = 0;
    }

    const QuadtreeDetector* detect()
    {
        clear();
        for (size_t index = 0; index < mPointCloud->size(); index++)
        {
            addIndex(index);
        }
        return subdivide();
    }

    const Eigen::Vector2f& center() const
    {
        return mCenter;
    }

    float size() const
    {
        return mSize;
    }

    bool isLeaf() const
    {
        return mLeaf;
    }

    const QuadtreeDetector* parent() const
    {
        return mParent;
    }

    const QuadtreeDetector* child(size_t index) const
    {
        return mChild[index];
    }

    const Rect2d& rect() const
    {
        return mRect;
    }

    const std::vector<size_t>& indices() const
    {
        return mIndices;
    }

    static inline int minDistAngles(int angle1, int angle2)
    {
        return 180 - std::abs(std::abs(angle1 - angle2) - 180);
    }

    static void filterAngles(std::map<int, int> &angles)
    {
        size_t numAngles = angles.size();
        size_t numPoints = 0;
        for (const std::pair<int, int> &angle : angles)
        {
            numPoints += angle.second;
        }
        float mean = numPoints / float(numAngles);
        float std = 0;
        for (const std::pair<int, int> &angle : angles)
        {
            std += std::pow(angle.second - mean, 2);
        }
        std = std::sqrt(std / float(numAngles));
        for (auto it = angles.begin(); it != angles.end(); )
        {
            if (it->second < mean - 1.5f * std || it->second > mean + 1.5f * std)
            {
                it = angles.erase(it);
            }
            else
            {
                ++it;
            }
        }
    }

    static int getArcLength(const std::map<int, int> &angles)
    {
        int maxDiff = 0;
        for (const std::pair<int, int> &angle1 : angles)
        {
            for (const std::pair<int, int> &angle2 : angles)
            {
                int diff = minDistAngles(angle1.first, angle2.first);
                if (diff > maxDiff)
                {
                    maxDiff = diff;
                }
            }
        }
        return maxDiff;
    }

private:
    const PointCloud2d *mPointCloud;
    QuadtreeDetector *mParent;
    QuadtreeDetector *mChild[4];
    Eigen::Vector2f mCenter;
    float mSize;
    Rect2d mRect;
    bool mLeaf;
    int mDepth;
    std::vector<size_t> mIndices;
    std::map<int, int> mAngles;
    int mArcLength;

    void createChildren()
    {
        if (!mLeaf)
        {
            for (size_t i = 0; i < 4; i++)
            {
                mChild[i]->clear();
            }
            return;
        }
        float halfSize = mSize / 2;
        mChild[0] = new QuadtreeDetector(mPointCloud, Eigen::Vector2f(mCenter.x() - halfSize, mCenter.y() - halfSize), halfSize, this);
        mChild[1] = new QuadtreeDetector(mPointCloud, Eigen::Vector2f(mCenter.x() - halfSize, mCenter.y() + halfSize), halfSize, this);
        mChild[2] = new QuadtreeDetector(mPointCloud, Eigen::Vector2f(mCenter.x() + halfSize, mCenter.y() - halfSize), halfSize, this);
        mChild[3] = new QuadtreeDetector(mPointCloud, Eigen::Vector2f(mCenter.x() + halfSize, mCenter.y() + halfSize), halfSize, this);
        mLeaf = false;
        for (size_t i = 0; i < 4; i++)
        {
            mChild[i]->mDepth = mDepth + 1;
        }
    }

    void addIndex(size_t index)
    {
        mIndices.push_back(index);
        int angle = int(mPointCloud->at(index).angle() / ANGLE_RANGE) * ANGLE_RANGE;
        auto it = mAngles.find(angle);
        if (it == mAngles.end())
        {
            mAngles.insert(it, std::make_pair(angle, 1));
        }
        else
        {
            ++it->second;
        }
    }

    const QuadtreeDetector* subdivide()
    {
        if (mAngles.size() >= MIN_NUM_ANGLES && getArcLength(mAngles) >= MIN_ARC_LENGTH)
        {
            if (mDepth >= MIN_DEPTH)
            {
                StatisticsUtils statistics(mIndices.size());
                statistics.size(mIndices.size());
                for (size_t i = 0; i < mIndices.size(); i++)
                {
                    statistics.dataBuffer()[i] = (mCenter - mPointCloud->at(mIndices[i]).position()).norm();
                }
                float median = statistics.getMedian();
                float mad = statistics.getMAD(median);
                std::vector<size_t> newIndices;
                for (size_t i = 0; i < mIndices.size(); i++)
                {
                    if (statistics.getRScore(statistics.dataBuffer()[i], median, mad) < MAX_R_SCORE)
                    {
                        newIndices.push_back(mIndices[i]);
                    }
                }
                clear();
                for (const size_t &index : newIndices)
                {
                    addIndex(index);
                }
                filterAngles(mAngles);
                if (mAngles.size() >= MIN_NUM_ANGLES) // && getArcLength(mAngles) >= MIN_ARC_LENGTH)
                {
                    return this;
                }
            }
            else
            {
                createChildren();
                for (const size_t &index : mIndices)
                {
                    for (size_t i = 0; i < 4; i++)
                    {
                        if (CollisionDetector::rayBoxCollision2d(mPointCloud->at(index), mChild[i]->mRect))
                        {
                            mChild[i]->addIndex(index);
                        }
                    }
                }
                clear();
                for (size_t i = 0; i < 4; i++)
                {
                    filterAngles(mChild[i]->mAngles);
                    mChild[i]->mArcLength = getArcLength(mChild[i]->mAngles);
                }
                std::sort(mChild, mChild + 4, [](const QuadtreeDetector *a, const QuadtreeDetector *b)
                {
                    if (a->mArcLength == b->mArcLength)
                    {
                        if (a->mAngles.size() == b->mAngles.size())
                        {
                            return a->mIndices.size() > b->mIndices.size();
                        }
                        return a->mAngles.size() > b->mAngles.size();
                    }
                    return a->mArcLength > b->mArcLength;
                });
                return mChild[0]->subdivide();
            }
        }
        return NULL;
    }

};

std::set<Circle*> CircleDetector::detect()
{
    std::set<Circle*> circles;
    Circle *circle = estimateCircle();
    if (circle != NULL)
    {
        circles.insert(circle);
    }
    return circles;
}

Circle* CircleDetector::estimateCircle()
{
    Rect2d extension = getExtension();
    Eigen::Vector2f center = extension.center();
    float size = extension.maxSize() * 4;
    QuadtreeDetector quadtreeDetector(pointCloud(), center, size);
    const QuadtreeDetector *focus = quadtreeDetector.detect();
    if (focus != NULL)
    {
        EstimatedCenter *EstimatedCenter = estimateCenter(focus->indices());
        if (EstimatedCenter != NULL)
        {
            if (EstimatedCenter->radius < 2 * size)
            {
                std::map<int, int> angles;
                std::vector<size_t> inliers;
                for (size_t index = 0; index < pointCloud()->size(); index++)
                {
                    float radius = (pointCloud()->at(index).position() - EstimatedCenter->estimatedCenter.center()).norm();
                    if (radius > EstimatedCenter->minRadius && radius < EstimatedCenter->maxRadius &&
                            CollisionDetector::rayBoxCollision2d(pointCloud()->at(index), EstimatedCenter->estimatedCenter))
                    {
                        angles[int(pointCloud()->at(index).angle() / ANGLE_RANGE) * ANGLE_RANGE] += 1;
                        inliers.push_back(index);
                    }
                }
                QuadtreeDetector::filterAngles(angles);
                if (angles.size() >= MIN_NUM_ANGLES && QuadtreeDetector::getArcLength(angles) >= MIN_ARC_LENGTH)
                {
                    Circle *circle = new Circle(EstimatedCenter->estimatedCenter.center(), EstimatedCenter->radius);
                    Eigen::Matrix2Xf matrix(2, inliers.size());
                    for (size_t i = 0; i < inliers.size(); i++)
                    {
                        matrix.col(i) = pointCloud()->at(inliers[i]).position();
                    }
                    circle->leastSquares(matrix);
                    circle->inliers(inliers);
                    delete EstimatedCenter;
                    return circle;
                }
            }
            delete EstimatedCenter;
        }
    }
    return NULL;
}

CircleDetector::EstimatedCenter* CircleDetector::estimateCenter(const std::vector<size_t> &indices)
{
    StatisticsUtils xs(NUM_SAMPLES);
    xs.size(NUM_SAMPLES);
    StatisticsUtils ys(NUM_SAMPLES);
    ys.size(NUM_SAMPLES);
    StatisticsUtils radiuses(NUM_SAMPLES);
    radiuses.size(NUM_SAMPLES);
    size_t numSamples = 0;
    for (size_t i = 0; i < NUM_SAMPLES; i++)
    {
        size_t p1 = indices[rand() % indices.size()];
        size_t p2 = indices[rand() % indices.size()];
        size_t p3 = indices[rand() % indices.size()];
        Circle *circle = estimateCircle(p1, p2, p3);
        if (circle != NULL)
        {
            xs.dataBuffer()[numSamples] = circle->center().x();
            ys.dataBuffer()[numSamples] = circle->center().y();
            radiuses.dataBuffer()[numSamples] = circle->radius();
            ++numSamples;
            delete circle;
        }
    }
    xs.size(numSamples);
    ys.size(numSamples);
    radiuses.size(numSamples);

    Eigen::Vector2f estimatedCenterPosition(xs.getMedian(), ys.getMedian());
    Eigen::Vector2f estimateCenterSize(3 * xs.getMAD(estimatedCenterPosition.x()), 3 * ys.getMAD(estimatedCenterPosition.y()));
    Eigen::Vector2f bottomLeft = estimatedCenterPosition - estimateCenterSize;
    Eigen::Vector2f topRight = estimatedCenterPosition + estimateCenterSize;
    Rect2d rect(bottomLeft, topRight);

    float medianRadius = radiuses.getMedian();
    float madRadius = radiuses.getMAD(medianRadius);
    float minRadius = std::max(0.0f, medianRadius - 3 * madRadius);
    float maxRadius = medianRadius + 3 * madRadius;

    EstimatedCenter *estimatedCenter = new EstimatedCenter;
    estimatedCenter->estimatedCenter = rect;
    estimatedCenter->maxRadius = maxRadius;
    estimatedCenter->minRadius = minRadius;
    estimatedCenter->radius = medianRadius;
    return estimatedCenter;
}

Rect2d CircleDetector::getExtension()
{
    Eigen::Vector2f min = Eigen::Vector2f::Constant(std::numeric_limits<float>::max());
    Eigen::Vector2f max = -min;
    for (size_t index = 0; index < pointCloud()->size(); index++)
    {
        const Eigen::Vector2f& position = pointCloud()->at(index).position();
        min.x() = std::min(min.x(), position.x());
        min.y() = std::min(min.y(), position.y());
        max.x() = std::max(max.x(), position.x());
        max.y() = std::max(max.y(), position.y());
    }
    return Rect2d(min, max);
}

bool CircleDetector::isValid(std::map<int, int> &angles)
{
    QuadtreeDetector::filterAngles(angles);
    return angles.size() >= MIN_NUM_ANGLES && QuadtreeDetector::getArcLength(angles) >= MIN_ARC_LENGTH;
}

Circle* CircleDetector::estimateCircle(size_t p1, size_t p2, size_t p3)
{
    Eigen::Vector2f a = pointCloud()->at(p1).position();
    Eigen::Vector2f b = pointCloud()->at(p2).position();
    Eigen::Vector2f c = pointCloud()->at(p3).position();

    float offset = std::pow(b.x(), 2) + std::pow(b.y(), 2);
    float ab = (std::pow(a.x(), 2) + std::pow(a.y(), 2) - offset) / 2;
    float bc = (offset - std::pow(c.x(),  2) - std::pow(c.y(),  2)) / 2;
    float det = (a.x() - b.x()) * (b.y() - c.y()) - (b.x() - c.x())* (a.y() - b.y());

    if (std::abs(det) < std::numeric_limits<float>::epsilon()) return NULL; // there is no intersection

    Eigen::Vector2f center;
    center.x() = (ab * (b.y() - c.y()) - bc * (a.y() - b.y())) / det;
    center.y() = (bc * (a.x() - b.x()) - ab * (b.x() - c.x())) / det;
    float radius = (center - a).norm();

    return new Circle(center, radius);
}
