#pragma once
#include "gx_vector.h"

class GeometryHit 
{
public:
    Vector P;
    Vector N;
    double tHit;
};

class BoundingBox
{
public:
    double tx0, ty0, tz0;
    double tx1, ty1, tz1;
};

class Geometry 
{
public:
    virtual bool intersect(const Ray& ray, GeometryHit& gHit) const = 0;
};

class Sphere : public Geometry
{
public:
    Sphere(const Vector& C, double R);

    Vector C;
    double R;

    bool intersect(const Ray& ray, GeometryHit& gHit) const;
};