#ifndef TRANSFORM_H
#define TRANSFORM_H

#include "vecmath.h"
#include "Object3D.h"

class Transform : public Object3D
{
public:
    Transform() {}

    Transform(const mat4 &m, Object3D *obj) : o(obj)
    {
        this->m = m;
        // this->o = obj;
    }

    ~Transform()
    {
    }
    /// TODO: implement this function so that the intersect function first transforms the ray into the object's local coordinate frame
    virtual bool intersect(const Ray &r, Hit &h, float tmin)
    {
        mat4 matInv = inverse(this->m);
        vec3 transRayOrigin = vec3(matInv * vec4(r.getOrigin(), 1.0f));
        vec3 transRayDir = vec3(matInv * vec4(r.getDirection(), 0.0f));
        Ray transRay = Ray(transRayOrigin, transRayDir);

        if (o->intersect(transRay, h, tmin)) {
            vec3 transNormal = normalize(vec3(transpose(matInv) * vec4(h.getNormal(), 0.0f)));
            h.set(h.getT(), h.getMaterial(), transNormal);
            return true;
        }

        return false;
    }

protected:
    Object3D *o; // un-transformed object
    mat4 m;
};

#endif // TRANSFORM_H
