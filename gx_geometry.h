#pragma once
#include "gx_vector.h"
#include "binary_tree.h"

class GeometryHit 
{
public:
    Vector P;
    Vector N;
    double tHit = std::numeric_limits<double>::max();;
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

	void merge(const BoundingBox& other);
	void resizeAddVector(const Vector& v);

    static BoundingBox merge(const BoundingBox& a, const BoundingBox& b);
	static BoundingBox initWithLimits();
};

class Geometry 
{
public:
    virtual ~Geometry() {};
    virtual bool intersect(const Ray& ray, GeometryHit& gHit) const = 0;
    virtual bool updateIntersect(const Ray& ray, GeometryHit& gHit) const = 0;
    virtual Geometry* clone() const = 0;

    virtual void transformTranslate(const Vector& delta) = 0;
    virtual void transformScale(double r) = 0;
    virtual void transformRotate(const Vector& axis, double theta) = 0;
    virtual void transformRotate(int axis, double theta) = 0;
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

    void transformTranslate(const Vector& delta);
    void transformScale(double r);
    void transformRotate(const Vector& axis, double theta);
    void transformRotate(int axis, double theta);
};

// ----- Triangle meshes ------

class TriangleIndices {
public:
	TriangleIndices(int vtxi = -1, int vtxj = -1, int vtxk = -1, int ni = -1,
					int nj = -1, int nk = -1, int uvi = -1, int uvj = -1,
					int uvk = -1, int group = -1, bool added = false)
		: vtxi(vtxi), vtxj(vtxj), vtxk(vtxk), uvi(uvi), uvj(uvj), uvk(uvk),
		ni(ni), nj(nj), nk(nk), group(group){};
		
	int vtxi, vtxj, vtxk; // indices within the vertex coordinates array
	int uvi, uvj, uvk;    // indices within the uv coordinates array
	int ni, nj, nk;       // indices within the normals array
	int group;            // face group
};

struct dataBVH {
	int start; // Index of first triangle
	int end;  // Index of last triangle
	BoundingBox bbox; // Bounding Box
};

class TriangleMesh : public Geometry{
public:
	TriangleMesh() = delete;
	TriangleMesh(const char *obj);
	~TriangleMesh() = default;

	Geometry* clone() const;

	void readOBJ(const char *obj);

	static const int BVH_STOP = 3;

	BoundingBox bbox;

	std::vector<TriangleIndices> indices;
	std::vector<Vector> vertices;
	std::vector<Vector> normals;
	std::vector<Vector> uvs;
	std::vector<Vector> vertexcolors;

	bool intersect(const Ray& ray, GeometryHit& gHit) const;
	bool updateIntersect(const Ray& ray, GeometryHit& gHit) const;

	bool fastUpdateIntersect(const Ray& ray, GeometryHit& gHit) const;

	Vector triangleCentroid(int idx) const;

	void transformTranslate(const Vector& delta);
	void transformScale(double r);
	void transformRotate(const Vector& axis, double theta);
	void transformRotate(int axis, double theta);

private:
	// BVH Optimization
	binary_tree::BinaryTree<dataBVH> treeBVH;
	void buildBVH ();
	binary_tree::Node<dataBVH> * buildBVH (int start, int end);

	bool BVHIntersectRoutine(const Ray& ray, GeometryHit& gHit, binary_tree::Node<dataBVH> *node) const;
};