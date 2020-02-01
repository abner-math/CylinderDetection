#include "cylinderdetector.h"
#include "angleutils.h"
#include "segmentator.h"
#include "pcacalculator.h"
#include "geometryutils.h"
#include "circledetector.h"
#include "unionfind.h"

#include <iostream>

#define NUM_INITIAL_DIRECTIONS 100
#define ANGULAR_THRESHOLD 0.07f
#define MAX_RADIUS_DIFF 2.0f
#define MIN_NUM_POINTS_CYLINDER 1000

std::set<Cylinder*> CylinderDetector::detect()
{
    std::vector<Cylinder*> cylinders;

    clearRemovedPoints();
    for (size_t i = 0; i < pointCloud()->geometry()->numPlanes(); i++)
    {
        for (const size_t &inlier : pointCloud()->geometry()->plane(i)->inliers())
        {
            removePoint(inlier);
        }
    }
    for (size_t i = 0; i < pointCloud()->geometry()->numCylinders(); i++)
    {
        for (const size_t &inlier : pointCloud()->geometry()->cylinder(i)->inliers())
        {
            removePoint(inlier);
        }
    }

    std::vector<Eigen::Vector3f> directions;
    getEvenlyDistributedDirections(directions);
    std::vector<ConnectedComponent*> components;
    for (const Eigen::Vector3f &direction : directions)
    {
        findConnectedComponents(direction, components);
    }
    updateComponents(0, components);
    for (size_t i = 0; i < components.size(); i++)
    {
        PointCloud2d *projectedPointCloud = getProjectedPointCloud(components[i]);
        CircleDetector *detector = new CircleDetector(projectedPointCloud);
        std::set<Circle*> circles = detector->detect();
        if (!circles.empty())
        {
            for (Circle *circle : circles)
            {
                Eigen::Vector3f center = GeometryUtils::unproject(circle->center(), components[i]->direction);
                Eigen::Vector3f axis = components[i]->direction;
                float radius = circle->radius();
                Cylinder *cylinder = new Cylinder(center, axis, radius);
                for (size_t j = 0; j < circle->inliers().size(); j++)
                {
                    size_t index = components[i]->indices[circle->inliers()[j]];
                    cylinder->addInlier(index);
                }
                fitCylinder(cylinder);
                if (isCylinderValid(cylinder))
                {
                    cylinders.push_back(cylinder);
                    removePoints(cylinder);
                }
                else
                {
                    delete cylinder;
                }
                delete circle;
            }
            updateComponents(i + 1, components);
        }
        delete detector;
        delete projectedPointCloud;
        delete components[i];
    }
    mergeCylinders(cylinders);
    return std::set<Cylinder*>(cylinders.begin(), cylinders.end());
}

Eigen::Vector3f CylinderDetector::getMedianVector(const std::vector<Eigen::Vector3f> &v)
{
    Eigen::Vector3f median;
    for (size_t i = 0; i < 3; i++)
    {
        std::vector<float> values;
        for (const Eigen::Vector3f &vec : v)
        {
            values.push_back(vec(i));
        }
        std::nth_element(values.begin(), values.begin() + values.size() / 2, values.end());
        median(i) = values[values.size() / 2];
    }
    return median;
}

Eigen::Vector3f CylinderDetector::getMedianNormal(const std::vector<size_t> &points)
{
    std::vector<Eigen::Vector3f> normals;
    for (size_t i = 0; i < points.size() / 10; i++)
    {
        Eigen::Vector3f n1 = pointCloud()->at(points[rand() % points.size()]).normal();
        Eigen::Vector3f n2 = pointCloud()->at(points[rand() % points.size()]).normal();
        Eigen::Vector3f normal = n1.cross(n2);
        if (std::abs(normal.norm()) > std::numeric_limits<float>::epsilon())
        {
            normal = normal.normalized();
            if (normals.size() > 0 && normals[0].dot(normal) < 0) normal = -normal;
            normals.push_back(normal);
        }
    }
    return getMedianVector(normals).normalized();
}

Cylinder* CylinderDetector::detectCylinder(const std::vector<size_t> &points, bool leastSquares)
{
    std::vector<Eigen::Vector3f> positions;
    for (const size_t &point : points)
    {
        positions.push_back(pointCloud()->at(point).position());
    }
    Eigen::Vector3f center = getMedianVector(positions);
    Eigen::Vector3f normal = getMedianNormal(points);
    Eigen::Vector3f basisU, basisV;
    GeometryUtils::orthogonalBasis(normal, basisU, basisV);
    Eigen::Vector2f centerProjected = GeometryUtils::projectOntoOrthogonalBasis(center, basisU, basisV);
    std::vector<float> radi;
    for (const size_t &point : points)
    {
        Eigen::Vector3f position = pointCloud()->at(point).position();
        Eigen::Vector2f positionProjected = GeometryUtils::projectOntoOrthogonalBasis(position, basisU, basisV);
        float dist = std::abs((positionProjected - centerProjected).norm());
        radi.push_back(dist);
    }
    std::nth_element(radi.begin(), radi.begin() + radi.size() / 2, radi.end());
    float radius = radi[radi.size() / 2];
    Cylinder *cylinder = new Cylinder(center, normal, radius);
    cylinder->inliers(points);
    Eigen::Matrix3Xf matrix(3, points.size());
    for (size_t i = 0; i < points.size(); i++)
    {
        matrix.col(i) = pointCloud()->at(points[i]).position();
    }
    if (leastSquares)
        cylinder->leastSquares(matrix);
    else
        cylinder->setHeight(matrix);
    return cylinder;
}

//https://stackoverflow.com/questions/9600801/evenly-distributing-n-points-on-a-sphere
//fibonacci sphere
void CylinderDetector::getEvenlyDistributedDirections(std::vector<Eigen::Vector3f> &directions)
{
    float offset = 2.0f / NUM_INITIAL_DIRECTIONS;
    float increment = M_PI * (3 - std::sqrt(5.0f));
    for (size_t i = 0; directions.size() < NUM_INITIAL_DIRECTIONS; i++)
    {
        float y = ((i * offset) - 1) + (offset / 2.0f);
        float r = std::sqrt(1 - y*y);
        float phi = ((i + 1) % NUM_INITIAL_DIRECTIONS) * increment;
        float x = std::cos(phi) * r;
        float z = std::sin(phi) * r;
        if (z < 0) continue; // half sphere
        directions.push_back(Eigen::Vector3f(x, y, z).normalized());
    }
}

PointCloud2d* CylinderDetector::getProjectedPointCloud(const ConnectedComponent *component)
{
    Eigen::Vector3f basisU, basisV;
    GeometryUtils::orthogonalBasis(component->direction, basisU, basisV);
    PointCloud2d *projectedPointCloud = new PointCloud2d(component->indices.size(), PointCloud2d::Mode::ALL);
    for (size_t i = 0; i < component->indices.size(); i++)
    {
        Eigen::Vector2f position = GeometryUtils::projectOntoOrthogonalBasis(pointCloud()->at(component->indices[i]).position(), basisU, basisV);
        Eigen::Vector2f normal = GeometryUtils::projectOntoOrthogonalBasis(pointCloud()->at(component->indices[i]).normal(), basisU, basisV).normalized();
        (*projectedPointCloud)[i].position(position);
        (*projectedPointCloud)[i].normal(normal);
    }
    return projectedPointCloud;
}

Eigen::Vector3f CylinderDetector::getMeanDirection(const std::vector<size_t> &indices)
{
    Eigen::Matrix3Xf matrix(3, indices.size() * 2);
    for (size_t i = 0; i < indices.size(); i++)
    {
        matrix.col(i) = pointCloud()->at(indices[i]).normal();
        matrix.col(indices.size() + i) = -pointCloud()->at(indices[i]).normal();
    }
    Eigen::Matrix3f eigenVectors;
    PCACalculator3d::calculate(matrix, eigenVectors);
    return eigenVectors.col(2);
}

void CylinderDetector::findConnectedComponents(const Eigen::Vector3f &direction, std::vector<ConnectedComponent*> &components)
{
    std::vector<bool> projected(pointCloud()->size(), false);
    std::vector<bool> visited(pointCloud()->size(), false);
    for (size_t i = 0; i < pointCloud()->size(); i++)
    {
        if (!isRemoved(i) && std::abs(pointCloud()->at(i).normal().dot(direction)) < ANGULAR_THRESHOLD)
        {
            projected[i] = true;
        }
    }
    UnionFind uf(pointCloud()->size());
    std::queue<size_t> q;
    for (size_t i = 0; i < pointCloud()->size(); i++)
    {
        if (projected[i] && !visited[i])
        {
            visited[i] = true;
            q.push(i);
            while (!q.empty())
            {
                size_t front = q.front();
                q.pop();
                for (const size_t &neighbor : pointCloud()->connectivity()->neighbors(front))
                {
                    if (projected[neighbor])
                    {
                        uf.join(front, neighbor);
                        if (!visited[neighbor])
                        {
                            q.push(neighbor);
                            visited[neighbor] = true;
                        }
                    }
                }
            }
        }
    }
    std::map<int, std::vector<size_t> > groups;
    for (size_t i = 0; i < pointCloud()->size(); i++)
    {
        if (!projected[i]) continue;
        int root = uf.root(i);
        if (root != -1)
        {
            groups[root].push_back(i);
        }
    }
    for (auto it = groups.begin(); it != groups.end(); ++it)
    {
        if (it->second.size() < MIN_NUM_POINTS_CYLINDER) continue;
        ConnectedComponent *component = new ConnectedComponent;
        component->indices = it->second;
        component->direction = getMeanDirection(it->second);
        std::fill(visited.begin(), visited.end(), false);
        for (const size_t &index : it->second)
        {
            q.push(index);
            visited[index] = true;
        }
        while (!q.empty())
        {
            size_t front = q.front();
            q.pop();
            for (const size_t &neighbor : pointCloud()->connectivity()->neighbors(front))
            {
                if (!isRemoved(neighbor) && !visited[neighbor])
                {
                    visited[neighbor] = true;
                    if (std::abs(pointCloud()->at(neighbor).normal().dot(component->direction)) < ANGULAR_THRESHOLD)
                    {
                        uf.join(front, neighbor);
                        q.push(neighbor);
                        component->indices.push_back(neighbor);
                    }
                }
            }
        }
        component->direction = getMeanDirection(it->second);
        components.push_back(component);
        it->second.clear();
    }
}

bool CylinderDetector::findSubcomponents(ConnectedComponent *component, std::vector<ConnectedComponent*> &subcomponents)
{
    std::vector<bool> projected(pointCloud()->size(), false);
    size_t countProjected = 0;
    for (const size_t &index : component->indices)
    {
        if (!isRemoved(index))
        {
            projected[index] = true;
            ++countProjected;
        }
    }
    if (countProjected == component->indices.size())
    {
        return false;
    }
    std::vector<bool> visited(pointCloud()->size(), false);
    UnionFind uf(pointCloud()->size());
    std::queue<size_t> q;
    for (const size_t &index : component->indices)
    {
        if (projected[index] && !visited[index])
        {
            visited[index] = true;
            q.push(index);
            while (!q.empty())
            {
                size_t front = q.front();
                q.pop();
                for (const size_t &neighbor : pointCloud()->connectivity()->neighbors(front))
                {
                    if (projected[neighbor])
                    {
                        uf.join(front, neighbor);
                        if (!visited[neighbor])
                        {
                            q.push(neighbor);
                            visited[neighbor] = true;
                        }
                    }
                }
            }
        }
    }
    std::map<int, std::vector<size_t> > groups;
    for (size_t i = 0; i < pointCloud()->size(); i++)
    {
        if (!projected[i]) continue;
        int root = uf.root(i);
        if (root != -1)
        {
            groups[root].push_back(i);
        }
    }
    for (auto it = groups.begin(); it != groups.end(); ++it)
    {
        if (it->second.size() < MIN_NUM_POINTS_CYLINDER) continue;
        ConnectedComponent *subcomponent = new ConnectedComponent;
        subcomponent->indices = it->second;
        subcomponent->direction = getMeanDirection(subcomponent->indices);
        std::fill(visited.begin(), visited.end(), false);
        for (const size_t &index : subcomponent->indices)
        {
            q.push(index);
            visited[index] = true;
        }
        while (!q.empty())
        {
            size_t front = q.front();
            q.pop();
            for (const size_t &neighbor : pointCloud()->connectivity()->neighbors(front))
            {
                if (!isRemoved(neighbor) && !visited[neighbor])
                {
                    visited[neighbor] = true;
                    if (std::abs(pointCloud()->at(neighbor).normal().dot(subcomponent->direction)) < ANGULAR_THRESHOLD)
                    {
                        subcomponent->indices.push_back(neighbor);
                        q.push(neighbor);
                    }
                }
            }
        }
        subcomponent->direction = getMeanDirection(subcomponent->indices);
        subcomponents.push_back(subcomponent);
        it->second.clear();
    }
    return true;
}

void CylinderDetector::updateComponents(size_t begin, std::vector<ConnectedComponent*> &components)
{
    std::vector<ConnectedComponent*> newComponents;
    components.erase(std::remove_if(components.begin() + begin, components.end(), [&](ConnectedComponent *component) {
        if (findSubcomponents(component, newComponents))
        {
            delete component;
            return true;
        }
        return false;
    }), components.end());
    if (!newComponents.empty())
    {
        components.insert(components.end(), newComponents.begin(), newComponents.end());
        std::sort(components.begin() + begin, components.end(), [](const ConnectedComponent *a, const ConnectedComponent *b) {
           return a->indices.size() > b->indices.size();
        });
    }
}

void CylinderDetector::fitCylinder(Cylinder *cylinder)
{
    Eigen::Matrix3Xf matrix(3, cylinder->inliers().size());
    for (size_t i = 0; i < cylinder->inliers().size(); i++)
    {
        matrix.col(i) = pointCloud()->at(cylinder->inliers()[i]).position();
    }
    cylinder->leastSquares(matrix);
}

void CylinderDetector::removePoints(Cylinder *cylinder)
{
    std::vector<bool> visited(pointCloud()->size(), false);
    std::queue<size_t> q;
    for (const size_t &inlier : cylinder->inliers())
    {
        q.push(inlier);
        visited[inlier] = true;
    }
    while (!q.empty())
    {
        size_t front = q.front();
        q.pop();
        removePoint(front);
        for (const size_t &neighbor : pointCloud()->connectivity()->neighbors(front))
        {
            if (!visited[neighbor])
            {
                visited[neighbor] = true;
                if (std::abs(pointCloud()->at(neighbor).normal().dot(cylinder->axis())) < ANGULAR_THRESHOLD)
                {
                    Eigen::Vector3f position = pointCloud()->at(neighbor).position();
                    Eigen::Vector3f normal = pointCloud()->at(neighbor).normal();
                    float dist = std::abs(cylinder->getSignedDistanceFromSurface(position) + cylinder->radius());
                    if (std::max(dist, cylinder->radius()) / std::min(dist, cylinder->radius()) < MAX_RADIUS_DIFF &&
                            std::abs(normal.dot(cylinder->normalAt(position))) > (1 - ANGULAR_THRESHOLD))
                    {
                        cylinder->addInlier(neighbor);
                    }
                    q.push(neighbor);
                }
            }
        }
    }
}

void CylinderDetector::mergeCylinders(std::vector<Cylinder*> &cylinders)
{
    std::vector<std::set<size_t> > inliers;
    for (Cylinder *cylinder : cylinders)
    {
        inliers.push_back(std::set<size_t>(cylinder->inliers().begin(), cylinder->inliers().end()));
    }
    UnionFind uf(cylinders.size());
    for (size_t i = 0; i < cylinders.size(); i++)
    {
        for (size_t j = i + 1; j < cylinders.size(); j++)
        {
            if (std::abs(cylinders[i]->axis().dot(cylinders[j]->axis())) > (1 - ANGULAR_THRESHOLD) &&
                    std::max(cylinders[i]->radius(), cylinders[j]->radius()) / std::min(cylinders[i]->radius(), cylinders[j]->radius()) < MAX_RADIUS_DIFF)
            {
                std::set<size_t> intersect;
                std::set_intersection(inliers[i].begin(), inliers[i].end(),
                                      inliers[j].begin(), inliers[j].end(), std::inserter(intersect, intersect.begin()));
                if (intersect.size() > std::min(inliers[i].size(), inliers[j].size()) / 2)
                {
                    uf.join(i, j);
                }
            }
        }
    }
    for (size_t i = 0; i < cylinders.size(); i++)
    {
        int root = uf.root(i);
        if (root != -1 && root != i)
        {
            inliers[root].insert(inliers[i].begin(), inliers[i].end());
            inliers[i].clear();
        }
    }
    for (int i = cylinders.size() - 1; i >= 0; i--)
    {
        if (inliers[i].empty())
        {
            delete cylinders[i];
            cylinders.erase(cylinders.begin() + i);
        }
        else
        {
            cylinders[i]->inliers(std::vector<size_t>(inliers[i].begin(), inliers[i].end()));
            fitCylinder(cylinders[i]);
            if (!isCylinderValid(cylinders[i]))
            {
                delete cylinders[i];
                cylinders.erase(cylinders.begin() + i);
            }
        }
    }
}

bool CylinderDetector::isCylinderValid(Cylinder *cylinder)
{
    if (cylinder->radius() / cylinder->height() > 5) return false;
    size_t countInliers = 0;
    std::map<int, int> angles;
    Eigen::Vector3f basisU, basisV;
    GeometryUtils::orthogonalBasis(cylinder->axis(), basisU, basisV);
    for (const size_t &inlier : cylinder->inliers())
    {
        const Point3d &point = pointCloud()->at(inlier);
        if (std::abs(cylinder->normalAt(point.position()).dot(point.normal())) > (1 - ANGULAR_THRESHOLD))
        {
            Eigen::Vector2f normalProjected = GeometryUtils::projectOntoOrthogonalBasis(point.normal(), basisU, basisV).normalized();
            int angle = int(AngleUtils::rad2deg(std::atan2(normalProjected.y(), normalProjected.x()) + M_PI) / 5) * 5;
            angles[angle] += 1;
            ++countInliers;
        }
    }
    if (countInliers / (float)cylinder->inliers().size() < 0.5f) return false;
    return CircleDetector::isValid(angles);
}
