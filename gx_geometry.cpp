#include "gx_geometry.h"

static inline double squareDouble(double x) {return x*x;}

Sphere::Sphere(const Vector& C, double R)
: C(C), R(R)
{
    bbox = BoundingBox(C[0]-R, C[1]-R, C[2]-R, C[0]+R, C[1]+R, C[2]+R);
}

Geometry* Sphere::clone() const
{
    return new Sphere(*this);
}

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

bool Sphere::updateIntersect(const Ray& ray, GeometryHit& gHit) const {
    Vector d = ray.O-C;
    double a = dot(ray.u, d);

    double delta = squareDouble(a)-d.norm2()+R*R;
    if (delta < 0)
        return false;

    double t2 = sqrt(delta) - a;
    if (t2 < 0)
        return false;
        
    double t1 = -sqrt(delta) - a;
    double t = (t1 > 0) ? t1 : t2;

    if (t >= gHit.tHit)
        return false;

    gHit.tHit = t;
    gHit.P = ray.O + gHit.tHit*ray.u;
    gHit.N = (gHit.P - C).normalized();

    return true;
}

// ----- Bounding Box ----- //
static inline bool hasSameSign(double a, double b)
{   
    return (a<0)?(b<0):(b>0);
}

BoundingBox::BoundingBox(double x0, double y0, double z0, double x1, double y1,
double z1)
: x0(x0), y0(y0), z0(z0), x1(x1), y1(y1), z1(z1)
{}

bool BoundingBox::intersect(const Ray& ray) const
{   
    // To avoid division by 0
    static const double eps = 1E-6;

    Vector u = ray.u;
    const Vector& O = ray.O;

    double xprime0 = -eps + x0 - O[0];
    double xprime1 = eps + x1 - O[0];
    if (u[0] < 0) {
        u[0] = -u[0];
        std::swap(xprime0, xprime1);
        xprime0 *= -1;
        xprime1 *= -1;
    }    

    double yprime0 = -eps + y0 - O[1];
    double yprime1 = eps + y1 - O[1];
    if (u[1] < 0){
        u[1] = -u[1];
        std::swap(yprime0, yprime1);
        yprime0 *= -1;
        yprime1 *= -1;
    }

    double zprime0 = -eps + z0 - O[2];
    double zprime1 = eps + z1 - O[2];
    if (u[2] < 0){
        u[2] = -u[2];
        std::swap(zprime0, zprime1);
        zprime0 *= -1;
        zprime1 *= -1;
    }

    // Centralized and Reflected so ray starts at (0, 0, 0) and (ux, uy, uz)
    // are all non-negative
    if (xprime1<0 || yprime1<0 || zprime1<0)
        return false;
    
    double t = 0;
    t = std::max(t, xprime0/(u[0]+eps));
    t = std::max(t, yprime0/(u[1]+eps));
    t = std::max(t, zprime0/(u[2]+eps));

    u *= t;
    return !(u[0] > xprime1 || u[1] > yprime1 || u[2] > zprime1);
}

bool BoundingBox::intersect(const Ray& ray, double& t_lower_bound) const
{   
    // To avoid division by 0
    static const double eps = 1E-6;

    Vector u = ray.u;
    const Vector& O = ray.O;

    double xprime0 = -eps + x0 - O[0];
    double xprime1 = eps + x1 - O[0];
    if (u[0] < 0) {
        u[0] = -u[0];
        std::swap(xprime0, xprime1);
        xprime0 *= -1;
        xprime1 *= -1;
    }    

    double yprime0 = -eps + y0 - O[1];
    double yprime1 = eps + y1 - O[1];
    if (u[1] < 0){
        u[1] = -u[1];
        std::swap(yprime0, yprime1);
        yprime0 *= -1;
        yprime1 *= -1;
    }

    double zprime0 = -eps + z0 - O[2];
    double zprime1 = eps + z1 - O[2];
    if (u[2] < 0){
        u[2] = -u[2];
        std::swap(zprime0, zprime1);
        zprime0 *= -1;
        zprime1 *= -1;
    }

    // Centralized and Reflected so ray starts at (0, 0, 0) and (ux, uy, uz)
    // are all non-negative
    if (xprime1<0 || yprime1<0 || zprime1<0)
        return false;
    
    double& t = t_lower_bound;
    t = std::max(t, xprime0/(u[0]+eps));
    t = std::max(t, yprime0/(u[1]+eps));
    t = std::max(t, zprime0/(u[2]+eps));

    u *= t;
    return !(u[0] > xprime1 || u[1] > yprime1 || u[2] > zprime1);
}

// Transformations
static inline Matrix rotationMatrix(const Vector& axis, double theta)
{   
    // https://en.wikipedia.org/wiki/Rotation_matrix
    // Assumes axis is normalized
    double sin_theta = sin(theta);
    double cos_theta = cos(theta);

    const Vector& u = axis;

    double out[3][3] = {
        {cos_theta+squareDouble(u[0])*(1-cos_theta), u[0]*u[1]*(1-cos_theta)-u[2]*sin_theta, u[0]*u[2]*(1-cos_theta)+u[1]*sin_theta},
        {u[1]*u[0]*(1-cos_theta)+u[2]*sin_theta, cos_theta+squareDouble(u[1])*(1-cos_theta), u[1]*u[2]*(1-cos_theta)-u[0]*sin_theta},
        {u[2]*u[0]*(1-cos_theta)-u[1]*sin_theta, u[2]*u[1]*(1-cos_theta)+u[0]*sin_theta, cos_theta+squareDouble(u[2])*(1-cos_theta)}
    };

    return Matrix(out);
}

static inline Matrix rotationMatrix(int axis, double theta)
{
    switch (axis) {
        case 1: return rotationMatrix(Vector(1,0,0), theta);
        case 2: return rotationMatrix(Vector(0,1,0), theta);
        case 3: return rotationMatrix(Vector(0,0,1), theta);
    }
    throw std::logic_error("Axis must be 0, 1, 2 in rotationMatrix");
}

void Sphere::transformTranslate(const Vector& delta)
{
    C+=delta;
}
void Sphere::transformScale(double r){
    R*=r;
    C*=r;
}
void Sphere::transformRotate(const Vector& axis, double theta)
{
    C=rotationMatrix(axis, theta)*C;
}
void Sphere::transformRotate(int axis, double theta)
{
    C=rotationMatrix(axis, theta)*C;
}