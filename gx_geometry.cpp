#include "gx_geometry.h"

#include <iostream> 
#include<assert.h>

template <typename T>
static inline void print(const T& x, bool endLine=true) {
    if (endLine)
        std::cout << x << std::endl;
    else
        std::cout << x << " ";
}

static inline double squareDouble(double x)
{
    return x*x;
}

static inline double max3(double a, double b, double c)
{
    return std::max(a, std::max(b, c));
}

static inline double min3(double a, double b, double c)
{
    return std::min(a, std::min(b, c));
}


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
    print("intersect bbox");
    // To avoid division by 0
    static const double eps = 1E-6;

    Vector u = ray.u;
    const Vector& O = ray.O;
    print("got ray");

    double xprime0 = -eps + x0 - O[0];
    double xprime1 = eps + x1 - O[0];
    if (u[0] < 0) {
        u[0] = -u[0];
        std::swap(xprime0, xprime1);
        xprime0 *= -1;
        xprime1 *= -1;
    }    
    print("intersect bbox write x");

    double yprime0 = -eps + y0 - O[1];
    double yprime1 = eps + y1 - O[1];
    if (u[1] < 0){
        u[1] = -u[1];
        std::swap(yprime0, yprime1);
        yprime0 *= -1;
        yprime1 *= -1;
    }
    print("intersect bbox write y");

    double zprime0 = -eps + z0 - O[2];
    double zprime1 = eps + z1 - O[2];
    if (u[2] < 0){
        u[2] = -u[2];
        std::swap(zprime0, zprime1);
        zprime0 *= -1;
        zprime1 *= -1;
    }
    print("intersect bbox write z");

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

void BoundingBox::resizeAddVector(const Vector& v)
{
    x0 = std::min(v[0], x0); y0 = std::min(v[1], y0); z0 = std::min(v[2], z0);
    x1 = std::max(v[0], x1); y1 = std::max(v[1], y1); z1 = std::max(v[2], z1);
}

void BoundingBox::merge(const BoundingBox& other)
{
    x0 = std::min(x0, other.x0);
    y0 = std::min(y0, other.y0);
    z0 = std::min(z0, other.z0);
    x1 = std::max(x1, other.x1);
    y1 = std::max(y1, other.y1);
    z1 = std::max(z1, other.z1);
}

BoundingBox BoundingBox::merge(const BoundingBox& a, const BoundingBox& b)
{
    return BoundingBox(
        std::min(a.x0, b.x0), std::min(a.y0, b.y0), std::min(a.z0, b.z0),
        std::max(a.x1, b.x1), std::max(a.y1, b.y1), std::max(a.z1, b.z1)
    );
}

BoundingBox BoundingBox::initWithLimits()
{   
    double x0,y0,z0,x1,y1,z1;
    x0 = y0 = z0 = std::numeric_limits<double>::max();
    x1 = y1 = z1 = std::numeric_limits<double>::min();
    return BoundingBox(x0,y0,z0,x1,y1,z1);
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

// ----- Triangle meshes ------
static bool intersectTriangle(const Ray& ray, const Vector& A, 
const Vector& B, const Vector& C, double data[4])
{
    Vector e1 = B-A;
    Vector e2 = C-A;
    Vector d = A - ray.O;
    const Vector& u = ray.u;
    
    Vector N = cross(e1, e2);
    double Z = 1/dot(u, N);

    double beta = dot(e2, cross(d, u)) * Z;
    double gamma = -dot(e1, cross(d, u)) * Z;
    double alpha = 1 - beta - gamma;
    double t = dot(d, N) * Z;

    if (alpha >= 0 && beta >= 0 && gamma >= 0 && t >= 0) {
        // Maybe change the way I do this with a struct
        data[0] = alpha;
        data[1] = beta;
        data[2] = gamma;
        data[3] = t;
        return true;
    }

    return false;
}

static inline BoundingBox triangleBoundingBox(const Vector& A, const Vector& B, const Vector& C)
{
    double x0,y0,z0,x1,y1,z1;

    x0 = min3(A[0], B[0], C[0]);
    y0 = min3(A[1], B[1], C[1]);
    z0 = min3(A[2], B[2], C[2]);

    x1 = max3(A[0], B[0], C[0]);
    y1 = max3(A[1], B[1], C[1]);
    z1 = max3(A[2], B[2], C[2]);

    return BoundingBox(x0,y0,z0,x1,y1,z1);
}

Geometry* TriangleMesh::clone() const
{
    return new TriangleMesh(*this);
}

TriangleMesh::TriangleMesh(const char* obj)
{
    readOBJ(obj);

    if (vertices.size() == 0)
        throw std::logic_error("Triangle Mesh cannot be empty");

    buildBVH();

    bbox = BoundingBox::initWithLimits();
    for (const auto& v : vertices) {
        bbox.resizeAddVector(v);
    }
}

bool TriangleMesh::intersect(const Ray& ray, GeometryHit& gHit) const
{
    throw std::logic_error("Not implemented error");
}

bool TriangleMesh::updateIntersect(const Ray& ray, GeometryHit& gHit) const
{   
    // CRASH!
    fastUpdateIntersect(ray, gHit);

    double t_lower_bound;

    if (!bbox.intersect(ray, t_lower_bound)) {
        return false;
    }

    if (t_lower_bound > gHit.tHit)
        return false;

    bool hasUpdate = false;

    for(const auto& tIndex : indices) {
        double data[4];
        const Vector& A = vertices[tIndex.vtxi];
        const Vector& B = vertices[tIndex.vtxj];
        const Vector& C = vertices[tIndex.vtxk];
        if (intersectTriangle(ray, A, B, C, data)) {
            if (data[4] < gHit.tHit) {
                hasUpdate = true;
                gHit.tHit = data[4];
                gHit.P = data[0]*A + data[1]*B + data[2]*C;
                gHit.N = (data[0]*normals[tIndex.ni] + data[1]*normals[tIndex.nj] + data[2]*normals[tIndex.nk]).normalized();
            }
        }
    }

    return hasUpdate;
}

bool TriangleMesh::fastUpdateIntersect(const Ray& ray, GeometryHit& gHit) const
{
    return BVHIntersectRoutine(ray, gHit, treeBVH.root);
}

bool TriangleMesh::BVHIntersectRoutine(const Ray& ray, GeometryHit& gHit, binary_tree::Node<dataBVH> *node) const
{
    assert(node != nullptr);

    print("BVH Routine 0");
    double t_lower_bound;

    if (!node->data.bbox.intersect(ray, t_lower_bound)) {
        return false;
    }

    if (t_lower_bound > gHit.tHit)
        return false;
    
    print(" -- may hit");
    bool hasUpdate = false;

    // Intersection is possible
    if (node->left != nullptr)
        hasUpdate = BVHIntersectRoutine(ray, gHit, node->left) || hasUpdate;
    
    if (node->right != nullptr)
        hasUpdate = BVHIntersectRoutine(ray, gHit, node->right) || hasUpdate;

    if (node->is_leaf()) {
        for(int i=node->data.start; i != node->data.end; i++) {
            const auto& tIndex = indices[i];
            double data[4];
            const Vector& A = vertices[tIndex.vtxi];
            const Vector& B = vertices[tIndex.vtxj];
            const Vector& C = vertices[tIndex.vtxk];
            if (intersectTriangle(ray, A, B, C, data)) {
                if (data[4] < gHit.tHit) {
                    hasUpdate = true;
                    gHit.tHit = data[4];
                    gHit.P = data[0]*A + data[1]*B + data[2]*C;
                    gHit.N = (data[0]*normals[tIndex.ni] + data[1]*normals[tIndex.nj] + data[2]*normals[tIndex.nk]).normalized();
                }
            }
        }
    }

    return hasUpdate;
}

void TriangleMesh::transformTranslate(const Vector& delta) {
    for (auto& vertex : vertices)
        vertex+=delta;
}
void TriangleMesh::transformScale(double r){
    for (auto& vertex : vertices)
        vertex *= r;
}
void TriangleMesh::transformRotate(const Vector& axis, double theta){
    Matrix Q = rotationMatrix(axis, theta);
    for (auto& vertex : vertices)
        vertex = Q*vertex;
    for (auto& normal : normals)
        normal = Q*normal;
}
void TriangleMesh::transformRotate(int axis, double theta){
    Matrix Q = rotationMatrix(axis, theta);
    for (auto& vertex : vertices)
        vertex = Q*vertex;
    for (auto& normal : normals)
        normal = Q*normal;
}

Vector TriangleMesh::triangleCentroid(int idx) const {
    const TriangleIndices& tIndex = indices[idx];
    return (vertices[tIndex.vtxi] + vertices[tIndex.vtxj] + vertices[tIndex.vtxk])/3;
}

// Acceleration - BVH
void TriangleMesh::buildBVH() 
{      
    print("Build BVH");
    treeBVH = binary_tree::BinaryTree(buildBVH(0, indices.size()));
    print("Tree completed");
}

binary_tree::Node<dataBVH>* TriangleMesh::buildBVH(int start, int end)
{
    // print("Building Interval: ",0); print(start,0); print(end);

    // Compute bounding box
    BoundingBox bbox = BoundingBox::initWithLimits();
    for (int i=start; i<end; i++) {
        const auto& tidx = indices[i];
        bbox.merge(triangleBoundingBox(vertices[tidx.vtxi], vertices[tidx.vtxj], vertices[tidx.vtxk]));
    }

    // Create current Node
    dataBVH data;
    data.start = start;
    data.end = end;
    data.bbox = bbox;

    binary_tree::Node<dataBVH> *node = new binary_tree::Node<dataBVH>(data);

    // initial condition
    if (end - start <= BVH_STOP) {
        return node;
    }

    // Compute longest axis to perform partition
    Vector lbBbox = Vector(bbox.x0, bbox.y0, bbox.z0);
    Vector ubBbox = Vector(bbox.x1, bbox.y1, bbox.z1);

    int axis = (ubBbox - lbBbox).argmax();
    double mid = (ubBbox[axis] + lbBbox[axis])/2.;

    // Partition scheme
    int ptr = start;
    int pivot = start;

    for (ptr = start; ptr != end; ptr++) {
        if (triangleCentroid(ptr)[axis] < mid) {
            std::swap(indices[ptr], indices[pivot]);
            pivot++;
        }
    }

    // pivot - start = Number of elements left to mid
    if (pivot == start || pivot == end)
        return node;

    // Will continue
    node->left = buildBVH(start, pivot);
    node->right = buildBVH(pivot, end);

    return node;
}