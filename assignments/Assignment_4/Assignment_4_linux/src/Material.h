#ifndef MATERIAL_H
#define MATERIAL_H

#include <cassert>
#include "vecmath.h"

#include "Ray.h"
#include "Hit.h"
#include "texture.hpp"

class Material
{
public:
    Material(const vec3 &d_color, const vec3 &s_color = vec3(0, 0, 0), float s = 0) : diffuseColor(d_color), specularColor(s_color), shininess(s)
    {
    }

    virtual ~Material()
    {
    }

    virtual vec3 getDiffuseColor() const
    {
        return diffuseColor;
    }

    /// TODO: Implement this function to compute diffuse and specular components of Phong lighting
    vec3 Shade(const Ray &ray, const Hit &hit, const vec3 &dirToLight, const vec3 &lightColor)
    {
        vec3 shadeColor = vec3(0.0f, 0.0f, 0.0f);  // black color
        vec3 surfaceNormal = normalize(hit.getNormal());

        float diffuseLightIntensity = dot(dirToLight, surfaceNormal);

        if (diffuseLightIntensity > 0) {
            shadeColor += this->diffuseColor * diffuseLightIntensity * lightColor;
        }

        vec3 reflectedRayDir = 2.0f * surfaceNormal * dot(surfaceNormal, dirToLight) - dirToLight;
        // direction to camera is the opposite of the ray direction
        float specularLightIntensity = dot(reflectedRayDir, -normalize(ray.getDirection()));

        if (diffuseLightIntensity > 0 && specularLightIntensity > 0) {
            shadeColor += this->specularColor * pow(specularLightIntensity, this->shininess) * lightColor;
        }
        return shadeColor;
    }

    void loadTexture(const char *filename)
    {
        t.load(filename);
    }

    Texture t;

protected:
    vec3 diffuseColor;
    vec3 specularColor;
    float shininess;
};

#endif // MATERIAL_H
