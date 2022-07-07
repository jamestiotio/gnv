#ifndef SPHERE_H
#define SPHERE_H

#include "Object3D.h"
#include "vecmath.h"
#include <cmath>

#include <iostream>
using namespace std;

class Sphere : public Object3D
{
public:
    Sphere()
    {
        // unit ball at the center
        this->center = vec3(0, 0, 0);
        this->radius = 1.0;
    }

    Sphere(vec3 center, float radius, Material *material) : Object3D(material)
    {
        this->center = center;
        this->radius = radius;
    }

    ~Sphere() {}

    /// TODO: implement this function for ray-sphere intersection test.
    virtual bool intersect(const Ray &r, Hit &h, float tmin)
    {
        vec3 rayOriginToCircleCenter = r.getOrigin() - this->center;

        const float a = dot(r.getDirection(), r.getDirection());
        const float b = 2.0f * dot(r.getDirection(), rayOriginToCircleCenter);
        const float c = dot(rayOriginToCircleCenter, rayOriginToCircleCenter) - (this->radius * this->radius);

        bool hasIntersect = false;

        if (b * b >= 4 * a * c)
        {
            float temp = sqrt(b * b - 4 * a * c);
            float t1 = (-b + temp) / (2.0f * a);
            float t2 = (-b - temp) / (2.0f * a);

            if (t1 > tmin && t1 < h.getT())
            {
                hasIntersect = true;
                updateHit(t1, r, h);
            }

            if (t2 > tmin && t2 < h.getT())
            {
                hasIntersect = true;
                updateHit(t2, r, h);
            }
        }

        return hasIntersect;
    }

protected:
    vec3 center;
    float radius;

private:
    // This function is used to update the Hit using the ray parameter, t, the surface normal vector of the intersection point, and the object material
    void updateHit(float rayParam, const Ray &r, Hit &h)
    {
        vec3 pointOnSurface = r.pointAtParameter(rayParam);
        vec3 pointNormal = glm::normalize(pointOnSurface - this->center);
        h.set(rayParam, this->material, pointNormal);
    }
};

#endif
