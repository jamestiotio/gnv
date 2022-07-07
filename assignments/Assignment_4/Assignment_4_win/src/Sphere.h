#ifndef SPHERE_H
#define SPHERE_H

#include "Object3D.h"
#include "vecmath.h"
#include <cmath>

#include <iostream>
using namespace std;

class Sphere: public Object3D
{
public:
	Sphere()
    { 
		//unit ball at the center
        this->center = vec3(0,0,0);
        this->radius = 1.0;
	}

	Sphere( vec3 center , float radius , Material* material ):Object3D(material)
    {
        this->center = center;
        this->radius = radius;
	}
	

	~Sphere(){}

    /// TODO: implement this function for ray-sphere intersection test.
	virtual bool intersect( const Ray& r , Hit& h , float tmin)
    {
        return false;   // remove this sentence in your implementation
	}

protected:
    vec3 center;
    float radius;
  

};


#endif
