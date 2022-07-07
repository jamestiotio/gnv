#ifndef TRANSFORM_H
#define TRANSFORM_H

#include "vecmath.h"
#include "Object3D.h"

class Transform: public Object3D
{
public: 
    Transform(){}

    Transform( const mat4& m, Object3D* obj ):o(obj)
    {
      this->m = m;
      // this->o = obj;
    }

    ~Transform()
    {

    }
    /// TODO: implement this function so that the intersect function first transforms the ray into the object's local coordindate frame
    virtual bool intersect( const Ray& r , Hit& h , float tmin)
    {
        return false;   // remove this sentence in your implementation
    }

 protected:
    Object3D* o; //un-transformed object
    mat4 m;
};

#endif //TRANSFORM_H
