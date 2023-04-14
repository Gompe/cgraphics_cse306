#define _CRT_SECURE_NO_WARNINGS 1

#include <iostream>
#include <vector>
#include <algorithm>
#include <cmath>
#include <chrono>
#include <atomic>

#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "./stb/stb_image_write.h"

#define STB_IMAGE_IMPLEMENTATION
#include "./stb/stb_image.h"

#define EPSILON 0.001
#define RANDOM_SEED 10

#include "gx_vector.h"
#include "gx_random.h"
#include "gx_camera.h"
#include "gx_geometry.h"


template <typename T>
static inline void print(const T& x, bool endLine=true) {
    if (endLine)
        std::cout << x << std::endl;
    else
        std::cout << x << " ";
}

static inline double squareDouble(double x) {return x*x; }

Vector random_cos(const Vector& N){
	// Generate random x, y ,z
	double r1 = random_uniform();
	double r2 = random_uniform();

	double x, y, z;
	x = cos(2*M_PI*r1)*sqrt(1 - r2);
	y = sin(2*M_PI*r1)*sqrt(1 - r2);
	z = sqrt(r2);

	// Generate local frame around N
	Vector T1 = N.orthogonal();
	T1 = T1.normalized();

	Vector T2 = cross(T1, N);
	T2 = T2.normalized();

	return x*T1 + y*T2 + z*N;
}


class Scene {
public:
	// ...
	Scene(const Vector& L, const double& I): L(L), I(I) {};

	Vector L;
	double I;

	std::vector<const Object *> objects;
	void insertObject(const Object* ptr) {
		objects.push_back(ptr);
	}

	// Physics -- Maybe move to another file later
	double schlick_reflexivity(const Vector& omega_i, const Vector &N, const double &n1, const double& n2){
		// Reflection Coefficient Approximation
		double k0 = ((n1-n2)*(n1-n2))/((n1+n2)*(n1+n2)); 

		return k0 + (1-k0)*std::pow(1 - dot(omega_i, N), 5);
	}

	bool intersect(const Ray& ray, ObjectHit& hitInfo) const
	{	
		hitInfo = ObjectHit();

		for(size_t i = 0; i < objects.size(); i++) {
			ObjectHit possibleHit;
			if (objects[i]->intersect(ray, possibleHit)) {
				if (possibleHit.tHit < hitInfo.tHit) {
					hitInfo = possibleHit;
				}
			}
		}
		return (hitInfo.object_ptr != nullptr);
	}

	bool updateIntersect(const Ray& ray, ObjectHit& hitInfo) const
	{	
		bool hasIntersection = false;
		for (const Object *object_ptr : objects) {
			hasIntersection = object_ptr->updateIntersect(ray, hitInfo) || hasIntersection;
		}
		return hasIntersection;
	}

	bool isVisible(const Vector& P, const Vector& N, const Vector& x) const
	{
		// Returns if point P with normal N can see x directly.
		ObjectHit hitInfo;
		Ray xray = Ray(P+EPSILON*N, (x-P).normalized());

		if (!updateIntersect(xray, hitInfo))
			return true;
		
		return (x-P).norm2() <= squareDouble(hitInfo.tHit);
	}

	Vector directLighting(const ObjectHit& hitInfo) 
	{
		Vector Lo = Vector(0, 0, 0);
		if (!isVisible(hitInfo.P, hitInfo.N, L))
			return Lo;

		Vector omega_i = L - hitInfo.P;
		Vector albedo = hitInfo.object_ptr->material.albedo;

		Lo = (
			(I/(4*M_PI*omega_i.norm2())) *
			(albedo/M_PI) * 
			std::max(0.0, cosine_similarity(hitInfo.N, omega_i))
		);

		return Lo;
	}

	Vector sphereDirectLighting(const ObjectHit& hitInfo, const Sphere *object_ptr)
	{	
		const Material& luminousMaterial = object_ptr->material;

		// Sample point on the light sphere
		const Vector& C = object_ptr->C;
		double R = object_ptr->R;

		Vector D = (hitInfo.P - C).normalized();
		Vector V = random_cos(D);
		Vector x_sphere = C + R * V;

		Vector Lo(0., 0., 0.);
		if (!isVisible(hitInfo.P, hitInfo.N, x_sphere))
			return Lo;

		Vector omega_i = (x_sphere - hitInfo.P).normalized();
		Vector N_sphere = (x_sphere - C).normalized();
		double R2 = (x_sphere - hitInfo.P).norm2();

		double form_factor = (
			std::max(0., dot(omega_i, hitInfo.N)) *
			std::max(0., dot(-omega_i, N_sphere)) /
			(x_sphere - hitInfo.P).norm2()
		);

		double pdf = std::max(0., dot(N_sphere, D))/(M_PI*R2);

		Lo = (
			(luminousMaterial.albedo/M_PI) *
			(luminousMaterial.light_intensity/(4*M_PI*M_PI*R2)) *
			form_factor /
			pdf
		);

		return Lo;
	}

	Vector indirectLighting(const ObjectHit& hitInfo, int ray_depth){
		if (ray_depth <= 0) return Vector(0,0,0);

		Ray random_ray = Ray(hitInfo.P + EPSILON*hitInfo.N, random_cos(hitInfo.N));
		return hitInfo.object_ptr->material.albedo * getColor(random_ray, ray_depth-1, true);
	}

	Vector getColor(const Ray& ray, int ray_depth, bool last_bounce_diffuse=false){
		if (ray_depth <= 0) return Vector(0,0,0);

		ObjectHit hitInfo;
		if (!updateIntersect(ray, hitInfo))
			return Vector(0.,0.,0.);

		// Hit happens
		Vector& P = hitInfo.P;
		Vector& N = hitInfo.N;
		const Material& materialHit = hitInfo.object_ptr->material;

		if (materialHit.is_light) {
			if (last_bounce_diffuse)
				return Vector(0.,0.,0.);
			else {
				double R2 = squareDouble(hitInfo.tHit);
				Vector light = Vector::initOnesVector() * materialHit.light_intensity;
				return light / (4*M_PI*M_PI*R2);
			}
		}

		if (materialHit.is_mirror) {
			Vector mirror_direction = ray.u - 2 * dot(ray.u, N) * N;
			Ray mirror_ray = Ray(P + EPSILON * N, mirror_direction);
			return getColor(mirror_ray, ray_depth - 1);
		}

		if (materialHit.is_transparent) {
			double n1 = 1; 
			double n2 = materialHit.refractive_index;

			double cos_incidence = dot(ray.u, N);
			bool is_inside_material = (cos_incidence > 0);

			if (is_inside_material) {
				std::swap(n1, n2);
				N *= -1;
				cos_incidence *= -1;
			}
			double n_ratio = n1/n2;
			double rad = 1 - n_ratio*n_ratio*(1 - cos_incidence*cos_incidence);

			if (rad < 0) {
				// total reflection
				Vector mirror_direction = ray.u - 2 * dot(ray.u, N) * N;
				Ray mirror_ray = Ray(P + EPSILON * N, mirror_direction);
				return getColor(mirror_ray, ray_depth - 1);
			}
			Vector transmitted_direction = n_ratio * (ray.u - cos_incidence * N) - sqrt(rad) * N;
			return getColor(Ray(P - EPSILON*N, transmitted_direction), ray_depth-1);
		}

		Vector Lo_direct = directLighting(hitInfo);

		// Check for direct lighting from spheres
		for (auto object_ptr : objects) {
			if (dynamic_cast<const Sphere *> (object_ptr) && object_ptr->material.is_light)
				Lo_direct += sphereDirectLighting(hitInfo, dynamic_cast<const Sphere *> (object_ptr));
		}

		return Lo_direct + indirectLighting(hitInfo, ray_depth);
	}

};

int main() {
	int W = 512;
	int H = 512;

	Object *sphere_1 = new Sphere(Vector(10, 7, 5), 7);
	sphere_1->material.is_mirror = true;

	Object *sphere_2 = new Sphere(Vector(-15, 7, 5), 2.7);
	sphere_2->material.set_light(1E7);

	Object *s3 = new Sphere(Vector(+14, 7, +20), 7, Material(Vector(0.8, 0.4, 0.8)));

	Object *s4 = new Sphere(Vector(-14, 5, +12), 5);

	Object *s5 = new Sphere(Vector(+1, 0, +30), 1, Material(Vector(0.01, 0.05, 0.01)));


	Object *left_wall = new Sphere(Vector(-1000, 0, 0), 940, Material(Vector(0.5, 0.8, 0.1)));
	Object *right_wall = new Sphere(Vector(1000, 0, 0), 940, Material(Vector(0.9, 0.2, 0.3)));
	Object *ceiling = new Sphere(Vector(0, 1000, 0), 940, Material(Vector(0.3, 0.5, 0.3)));
	Object *floor = new Sphere(Vector(0, -1000, 0), 1000, Material((Vector(0.8, 0.8, 0.8))));
	Object *front_wall = new Sphere(Vector(0, 0, -1000), 910, Material(Vector(0.1, 0.6, 0.7)));
	Object *behind_wall = new Sphere(Vector(0, 0, 1000), 940, Material(Vector(0.8, 0.6, 0.8)));

	Vector camera_center(0, 10, 50);	

	double alpha = 60.*M_PI/180.;
	double I = 4E6;

	Vector L(0, 30, 20);
	Scene scene(L, I);

	Object *mesh = new TriangleMesh("./data/cat/cat.obj", Material(Vector(0.2, 0.8, 0.8)));

	mesh->transformScale(0.8);
	mesh->transformTranslate(Vector(0,0,-25));
	mesh->transformRotate(2, -1.5);
	mesh->transformTranslate(Vector(-30, 0, 0));

	Object *mesh2 = new TriangleMesh("./data/cat/cat.obj", Material(Vector(0.1, 0.1, 0.1)));

	mesh2->transformScale(0.8);
	mesh2->transformTranslate(Vector(0,0,-25));
	mesh2->transformRotate(2, -1.5);
	mesh2->transformTranslate(Vector(-30, 0, 0));
	mesh2->transformScale(0.2);
	mesh2->transformTranslate(Vector(+5, 0, +20));

	scene.insertObject(sphere_1);
	scene.insertObject(sphere_2);
	scene.insertObject(s3);
	scene.insertObject(s4);
	scene.insertObject(s5);

	scene.insertObject(left_wall);
	scene.insertObject(right_wall);
	scene.insertObject(ceiling);
	scene.insertObject(floor);
	scene.insertObject(front_wall);
	scene.insertObject(behind_wall);

	// Most complex object at the end of intersection loop...
	scene.insertObject(mesh);
	scene.insertObject(mesh2);

	PinholeCamera camera(camera_center, W, H, alpha);

	while(true){
	std::vector<unsigned char> image(W * H * 3, 0);

	int NUMBER_OF_RAYS;
	std::cout << "Number of rays:";
	std::cin >> NUMBER_OF_RAYS;
	
	const int N_BOUNCES = 5;

	auto start = std::chrono::high_resolution_clock::now();

	#pragma omp parallel for
	for (int i = 0; i < H; i++) {
		for (int j = 0; j < W; j++) {
			Vector color = Vector(0,0,0);

			for(int _counter=0; _counter<NUMBER_OF_RAYS; _counter++){
				Ray r = camera.generate_ray(i, j);

				Vector vec = scene.getColor(r, N_BOUNCES);
				color+= vec;
			}

			color = color / NUMBER_OF_RAYS;
			color = color.clip(0., 255.);
			color /= 255.;
			
			color[0] = std::pow(color[0], 0.45);
			color[1] = std::pow(color[1], 0.45);
			color[2] = std::pow(color[2], 0.45);

			color *= 255.;

			image[(i * W + j) * 3 + 0] = std::min(color[0], 255.);
			image[(i * W + j) * 3 + 1] = std::min(color[1], 255.);
			image[(i * W + j) * 3 + 2] = std::min(color[2], 255.);
		}
	}

	auto end = std::chrono::high_resolution_clock::now();
	auto time_elapsed = std::chrono::duration<double>(end - start).count();
	std::cout << "waited " << 1000*time_elapsed << std::endl;

	stbi_write_png("image.png", W, H, 3, &image[0], 0);

	}

	return 0;
}