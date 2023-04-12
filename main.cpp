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
#include "gx_mesh.h"
#include "gx_geometry.h"

#include "gx_object.h"

double sqr(double x) {
	return x * x;
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

	std::vector<Object> objects;

	// Physics -- Maybe move to another file later
	double schlick_reflexivity(const Vector& omega_i, const Vector &N, const double &n1, const double& n2){
		// Reflection Coefficient Approximation
		double k0 = ((n1-n2)*(n1-n2))/((n1+n2)*(n1+n2)); 

		return k0 + (1-k0)*std::pow(1 - dot(omega_i, N), 5);
	}

	bool intersect(const Ray& ray, ObjectHit& hitInfo) const
	{	
		bool hasIntersection = false;
		double first_hit_distance = std::numeric_limits<double>::max();
		for(size_t i = 0; i < objects.size(); i++) {
			ObjectHit possibleHit;
			if (objects[i].intersect(ray, possibleHit)) {
				if (possibleHit.tHit < first_hit_distance) {
					first_hit_distance = possibleHit.tHit;
					hitInfo = possibleHit;
					hasIntersection = true;
				}
			}
		}
		return hasIntersection;
	}

	bool updateIntersect(const Ray& ray, ObjectHit& hitInfo) const
	{	
		bool hasIntersection = false;
		for (const auto& object : objects) {
			// NOTE: Try to understand why (const auto object : objects) by value fails
			// reason: pointer to object in hitInfo is meaningless...

			// NOTE: hasIntersection = hasIntersection || object.updateIntersect(ray, hitInfo); WILL NOT WORK!!!
			hasIntersection = object.updateIntersect(ray, hitInfo) || hasIntersection;
		}
		return hasIntersection;
	}

	bool isVisible(const Vector& P, const Vector& N, const Vector& x) const
	{
		// Returns if point P with normal N can see x directly.
		ObjectHit hitInfo;
		Ray xray = Ray(P+EPSILON*N, (x-P).normalized());

		if (!intersect(xray, hitInfo))
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

	Vector directLighting(const ObjectHit& hitInfo, const Sphere& lightSphere,
							const Material& luminousMaterial) 
	{
		// Sample point on the light sphere
		const Vector& C = lightSphere.C;
		double R = lightSphere.R;

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
		if (ray_depth < 0) return Vector(0,0,0);

		Ray random_ray = Ray(hitInfo.P + EPSILON*hitInfo.N, random_cos(hitInfo.N));
		return hitInfo.object_ptr->material.albedo * getColor(random_ray, ray_depth-1, true);
	}

	Vector getColor(const Ray& ray, int ray_depth, bool last_bounce_diffuse=false){
		if (ray_depth < 0) return Vector(0,0,0);

		// CHANGE SO THAT INIT SETS TO DOUBLE MAX
		ObjectHit hitInfo;
		hitInfo.tHit = std::numeric_limits<double>::max();

		// if (!updateIntersect(ray, hitInfo))
		// 	return Vector(0.,0.,0.);

		if (!intersect(ray, hitInfo))
			return Vector();

		// Hit happens
		Vector& P = hitInfo.P;
		Vector& N = hitInfo.N;
		const Material& materialHit = hitInfo.object_ptr->material;

		if (materialHit.is_light) {
			if (last_bounce_diffuse)
				return Vector(0.,0.,0.);
			else {
				double R2 = squareDouble(hitInfo.tHit);

				// different from lecture notes. Maybe change later
				Vector coloredLight = materialHit.albedo * materialHit.light_intensity;

				return coloredLight / (4*M_PI*M_PI*R2);
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
		for (size_t i=0; i<objects.size(); i++) {
			if (dynamic_cast<Sphere*> (objects[i].geometry_ptr)) {
				// objects[i].geometry is a sphere
				const Material& luminousMaterial = objects[i].material;
				if (luminousMaterial.is_light) 
					Lo_direct += directLighting(hitInfo, 
						*(dynamic_cast<Sphere*> (objects[i].geometry_ptr)), luminousMaterial);
			}
		}

		return Lo_direct + indirectLighting(hitInfo, ray_depth);
	}

	void insertObject(Object&& object) {
		objects.push_back(std::move(object));
	}

};

int main() {
	int W = 512;
	int H = 512;

	Object sphere_1(
		Sphere(Vector(0, 7, 5), 7),
		Material()
	);

	sphere_1.material.is_mirror = true;

	Object sphere_2(
		Sphere(Vector(-14, 7, 5), 3),
		Material()
	);

	sphere_2.material.set_light(1E7);

	Object s3(
		Sphere(Vector(+14, 7, +20), 7),
		Material(Vector(0.8, 0.4, 0.8))
	);

	Object s4(
		Sphere(Vector(-14, 5, +12), 5),
		Material()
	);

	Object s5(
		Sphere(Vector(+1, 0, +45), 1),
		Material(Vector(0.5, 1., 0.5))
	);

	Object left_wall(
		Sphere(Vector(-1000, 0, 0), 940),
		Material(Vector(0.5, 0.8, 0.1))
	);

	Object right_wall(
		Sphere(Vector(1000, 0, 0), 940),
		Material(Vector(0.9, 0.2, 0.3))
	);

	Object ceiling(
		Sphere(Vector(0, 1000, 0), 940),
		Material(Vector(0.3, 0.5, 0.3))
	);

	Object floor(
		Sphere(Vector(0, -1000, 0), 1000),
		Material((Vector(0.1, 0.1, 0.8)))
	);	

	Object front_wall(
		Sphere(Vector(0, 0, -1000), 940),
		Material(Vector(0., 0.8, 0.4))
	);

	Object behind_wall(
		Sphere(Vector(0, 0, 1000), 940),
		Material(Vector(1., 0.2, 0.1))
	);

	Vector camera_center(0, 0, 50);	

	double alpha = 60.*M_PI/180.;
	// double I = 8E9;
	double I = 1E7;

	Vector L(-10, 20, 50);
	Scene scene(L, I);

	scene.insertObject(std::move(sphere_1));
	scene.insertObject(std::move(sphere_2));
	scene.insertObject(std::move(s3));
	scene.insertObject(std::move(s4));
	scene.insertObject(std::move(s5));

	scene.insertObject(std::move(left_wall));
	scene.insertObject(std::move(right_wall));
	scene.insertObject(std::move(ceiling));
	scene.insertObject(std::move(floor));
	scene.insertObject(std::move(front_wall));
	scene.insertObject(std::move(behind_wall));

	// Input
	double focal_distance;
	double radius_aperture;

	// std::cout << "Focal distance: ";
	// std::cin >> focal_distance;
	focal_distance = 40;

	// std::cout << "Aperture radius: ";
	// std::cin >> radius_aperture;
	radius_aperture = 5E-2;

	ProjectiveCamera camera(camera_center, W, H, alpha, focal_distance, radius_aperture);

	while(true){
	std::vector<unsigned char> image(W * H * 3, 0);

	double theta;
	std::cout << "Theta: ";
	std::cin >> theta;

	scene.objects[3].geometry_ptr->transformTranslate(-camera_center);
	scene.objects[3].geometry_ptr->transformRotate(2, theta);
	scene.objects[3].geometry_ptr->transformTranslate(camera_center);

	int NUMBER_OF_RAYS;
	std::cout << "Number of rays:";
	std::cin >> NUMBER_OF_RAYS;

	
	auto start = std::chrono::high_resolution_clock::now();

	#pragma omp parallel for
	for (int i = 0; i < H; i++) {
		for (int j = 0; j < W; j++) {
			Vector color = Vector(0,0,0);

			for(int _counter=0; _counter<NUMBER_OF_RAYS; _counter++){
				Ray r = camera.generate_ray(i, j);

				Vector vec = scene.getColor(r, 5);
				// std::cout << vec << std::endl;
				// std::cout << vec.clip(0., 255.) << std::endl;
				color+= vec.clip(0., 255.);
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

			// image[(i * W + j) * 3 + 0] = std::min(std::pow(color[0], 0.45), 255.);
			// image[(i * W + j) * 3 + 1] = std::min(std::pow(color[1], 0.45), 255.);
			// image[(i * W + j) * 3 + 2] = std::min(std::pow(color[2], 0.45), 255.);
		}
	}

	auto end = std::chrono::high_resolution_clock::now();
	auto time_elapsed = std::chrono::duration<double>(end - start).count();
	std::cout << "waited " << 1000*time_elapsed << std::endl;

	stbi_write_png("image.png", W, H, 3, &image[0], 0);

	}

	return 0;
}