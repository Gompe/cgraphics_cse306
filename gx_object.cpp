#include "gx_object.h"

Object::Object(Geometry* g, const Material& m)
: geometry_ptr(g), material(m)
{}

bool Object::intersect(const Ray& ray, ObjectHit& hitInfo) const 
{
    if (!geometry_ptr->intersect(ray, hitInfo))
        return false;
    hitInfo.object_ptr = this;
    return true;
}

