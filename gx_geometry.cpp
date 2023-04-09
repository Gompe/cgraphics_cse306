#include "gx_geometry.h"

static inline double squareDouble(double& x) {return x*x;}

Sphere::Sphere(const Vector& C, double R)
: C(C), R(R)
{}

bool Sphere::intersect(const Ray& ray, GeometryHit& gHit) const {
    Vector d = ray.O-C;
    double a = dot(ray.u, d);

    double delta = squareDouble(a)-d.norm2()+R*R;
    if (delta < 0)
        return false;
    double t2 = sqrt(delta) - a;
    if (t2 < 0)
        return false;
        
    double t1 = -sqrt(delta) - a;
    gHit.tHit = (t1 > 0) ? t1 : t2;
    gHit.P = ray.O + gHit.tHit*ray.u;
    gHit.N = (gHit.P - C).normalized();

    return true;
}