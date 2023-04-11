#pragma once

#include "gx_geometry.h"
#include "gx_material.h"

class ObjectHit;

class Object
{
public:
    Object() = delete;
    Object(const Geometry& g, const Material& m);
    Object(const Object& other) = delete; // Ideally delete later...
    Object(Object&& other);
    Object& operator=(const Object& other);

    ~Object();

    Geometry *geometry_ptr;
    Material material;

    bool intersect(const Ray& ray, ObjectHit& hitInfo) const;
    bool updateIntersect(const Ray& ray, ObjectHit& hitInfo) const;
};

class ObjectHit : public GeometryHit
{
public:
    const Object *object_ptr;
};
