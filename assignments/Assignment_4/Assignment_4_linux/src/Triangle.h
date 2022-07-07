#ifndef TRIANGLE_H
#define TRIANGLE_H

#include "Object3D.h"
#include "vecmath.h"
#include <cmath>
#include <iostream>

using namespace std;

class Triangle : public Object3D
{
public:
    Triangle();
    ///@param a b c are three vertex positions of the triangle
    Triangle(const vec3 &a, const vec3 &b, const vec3 &c, Material *m) : Object3D(m)
    {
        this->a = a;
        this->b = b;
        this->c = c;
        hasTex = false;
    }

    /// TODO: implement this function for ray-triangle intersection test.
    virtual bool intersect(const Ray &ray, Hit &hit, float tmin)
    {
        vec3 constCol = this->a - ray.getOrigin();
        vec3 betaCol = this->a - this->b;
        vec3 gammaCol = this->a - this->c;
        vec3 tCol = ray.getDirection();

        float coefDet = determinant(glm::mat3(betaCol, gammaCol, tCol));
        float betaDet = determinant(glm::mat3(constCol, gammaCol, tCol));
        float gammaDet = determinant(glm::mat3(betaCol, constCol, tCol));
        float tDet = determinant(glm::mat3(betaCol, gammaCol, constCol));

        float beta = betaDet / coefDet;
        float gamma = gammaDet / coefDet;
        float t = tDet / coefDet;

        if (t > tmin && t < hit.getT() && beta + gamma <= 1.0f && beta >= 0.0f && gamma >= 0.0f) {
            float alpha = 1.0f - beta - gamma;
            vec3 normal = alpha * normals[0] + beta * normals[1] + gamma * normals[2];
            hit.set(t, this->material, glm::normalize(normal));
            return true;
        }

        return false;
    }

    bool hasTex;
    vec3 normals[3];
    vec2 texCoords[3];

protected:
    vec3 a;
    vec3 b;
    vec3 c;
};

#endif // TRIANGLE_H
