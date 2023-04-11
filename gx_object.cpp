#include "gx_object.h"

Object::Object(const Geometry& g, const Material& m)
: material(m)
{
    geometry_ptr = g.clone();
}

Object::Object(Object&& other)
: material(std::move(other.material))
{
    geometry_ptr = other.geometry_ptr;
    other.geometry_ptr = nullptr;
}

Object& Object::operator=(const Object& other)
{   
    if (this == &other)
        return *this;

    material = other.material;
    delete geometry_ptr;
    geometry_ptr = other.geometry_ptr->clone();
    return *this;
}

Object::~Object()
{
    delete geometry_ptr;
}

bool Object::intersect(const Ray& ray, ObjectHit& hitInfo) const 
{
    if (!geometry_ptr->intersect(ray, hitInfo))
        return false;
    hitInfo.object_ptr = this;
    return true;
}

bool Object::updateIntersect(const Ray& ray, ObjectHit& hitInfo) const
{
    if (!geometry_ptr->updateIntersect(ray, hitInfo))
        return false;
    hitInfo.object_ptr = this;
    return true;
}
