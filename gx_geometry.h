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
    BoundingBox() = default;
    BoundingBox(double x0, double y0, double z0, double x1, double y1, double z1);
    double x0, y0, z0;
    double x1, y1, z1;

    bool intersect(const Ray& ray) const;
    bool intersect(const Ray& ray, double& t_lower_bound) const;
};

class Geometry 
{
public:
    virtual ~Geometry() {};
    virtual bool intersect(const Ray& ray, GeometryHit& gHit) const = 0;
    virtual bool updateIntersect(const Ray& ray, GeometryHit& gHit) const = 0;
    virtual Geometry* clone() const = 0;
};

class Sphere : public Geometry
{
public:
    Sphere(const Vector& C, double R);
    ~Sphere() = default;

    Geometry* clone() const;

    Vector C;
    double R;
    BoundingBox bbox;

    bool intersect(const Ray& ray, GeometryHit& gHit) const;
    bool updateIntersect(const Ray& ray, GeometryHit& gHit) const;
};