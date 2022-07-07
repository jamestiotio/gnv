#ifndef TRIANGLE_H
#define TRIANGLE_H

#include "Object3D.h"
#include "vecmath.h"
#include <cmath>
#include <iostream>

using namespace std;

class Triangle: public Object3D
{
public:
	Triangle();
    ///@param a b c are three vertex positions of the triangle
	Triangle( const vec3& a, const vec3& b, const vec3& c, Material* m):Object3D(m)
    {
        this->a = a;
        this->b = b;
        this->c = c;
        hasTex = false;
	}

    /// TODO: implement this function for ray-triangle intersection test.
	virtual bool intersect( const Ray& ray,  Hit& hit , float tmin)
    {
        return false;   // remove this sentence in your implementation
    }
    
	bool hasTex;
	vec3 normals[3];
	vec2 texCoords[3];
    
protected:
    vec3 a;
    vec3 b;
    vec3 c;

};

#endif //TRIANGLE_H
