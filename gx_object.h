#pragma once

#include "gx_geometry.h"
#include "gx_material.h"

class ObjectHit;

class Object
{
public:
    Object(Geometry* g, const Material& m);
    Geometry * const geometry_ptr;
    Material material;

    bool intersect(const Ray& ray, ObjectHit& hitInfo) const;
};

class ObjectHit : public GeometryHit
{
public:
    const Object *object_ptr;
};
