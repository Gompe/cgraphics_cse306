#define _CRT_SECURE_NO_WARNINGS 1

#include <iostream>
#include <vector>
#include <algorithm>
#include <cmath>

#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "./stb/stb_image_write.h"

#define STB_IMAGE_IMPLEMENTATION
#include "./stb/stb_image.h"

#define EPSILON 0.001
#define RANDOM_SEED 10

#include "gx_vector.h"
#include "gx_random.h"


double sqr(double x) {
	return x * x;
}

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

class Ray {
public:
	Ray(const Vector& O, const Vector& u) : O(O), u(u) {};
	Vector O;
	Vector u;
};

class Material {
public:
	bool is_mirror = false;
	bool is_transparent = false;
	bool is_light = false;

	double refractive_index = 1.4;
	double light_intensity = 0;

	// White as default color
	Vector albedo = Vector(1., 1. ,1.); 

	Material() = default;
	Material(const Vector& albedo) : albedo(albedo){}

	void set_is_mirror(bool is_mirror) {this->is_mirror = is_mirror;}
	void set_is_transparent(bool is_transparent) {this->is_transparent = is_transparent;}
	void set_refractive_index(double refractive_index) {this->refractive_index = refractive_index;}
	void set_light(double light_intensity) {
		is_light = (light_intensity != 0.);
		this->light_intensity = light_intensity;
	}

	bool operator==(const Material& other) {
		return (
			is_mirror == other.is_mirror &&
			is_transparent == other.is_transparent &&
			refractive_index == other.refractive_index &&
			albedo[0] == other.albedo[0] &&
			albedo[1] == other.albedo[1] &&
			albedo[2] == other.albedo[2]
		);
	}
};

class HitInfo {
public:
	bool is_hit = false;
	Vector P; 
	Vector N;
	Material material;

	void set_is_hit(bool is_hit){this->is_hit = is_hit;}
	void store_hit(const Vector& P, const Vector& N, const Material& material){
		is_hit = true;
		this->P = P;
		this->N = N;
		this->material = material;
	}
};

class Sphere {
public:
	// ...
	Sphere(const Material& m, const Vector& C, double R) {
		this->C = C;
		this->R = R;
		material = m;
	}	
	
	Vector C;
	double R;

	Material material;

	// Getters
	bool is_mirror() const {
		return material.is_mirror;
	}

	bool is_transparent() const {
		return material.is_transparent;
	}

	const Vector& albedo() const {
		return material.albedo;
	}

	bool is_light() const {
		return material.is_light;
	}

	double light_intensity() const {
		return material.light_intensity;
	}

	void intersect(const Ray& r, HitInfo& hit_info) const {
		hit_info.set_is_hit(false);
		double delta = sqr(dot(r.u, r.O - C)) - dot(r.O - C, r.O - C) + R * R;
		if (delta >= 0) {
			double t2 = dot(r.u, C - r.O) + sqrt(delta);
			if (t2 >= 0){
				double t1 = dot(r.u, C - r.O) - sqrt(delta);
				double t = (t1 > 0) ? t1 : t2;

				Vector P = r.O + t * r.u;
				Vector N = P - C;
				N = N.normalized();
				
				hit_info.store_hit(P, N, material);
			}
		}
	}


};

class Scene {
public:
	// ...
	Scene(const Vector& L, const double& I): L(L), I(I) {};

	Vector L;
	double I;

	// Physics -- Maybe move to another file later
	double schlick_reflexivity(const Vector& omega_i, const Vector &N, const double &n1, const double& n2){
		// Reflection Coefficient Approximation
		double k0 = ((n1-n2)*(n1-n2))/((n1+n2)*(n1+n2)); 

		return k0 + (1-k0)*std::pow(1 - dot(omega_i, N), 5);
	}


	void intersect(const Ray& ray, HitInfo& hit_info) const {
		hit_info.set_is_hit(false);
		double first_hit_distance_2 = std::numeric_limits<double>::max();

		for(size_t i = 0; i < objects.size(); i++) {
			HitInfo possible_hit;
			objects[i].intersect(ray, possible_hit);

			if (possible_hit.is_hit) {
				double hit_distance_2 = (ray.O - possible_hit.P).norm2();
				if (hit_distance_2 < first_hit_distance_2) {
					first_hit_distance_2 = hit_distance_2;
					hit_info = possible_hit;
				}
			}
		}
	}


	double compute_visibility(const Vector& P, const Vector& N, const Vector& x) const {
		// Returns the visibility at point P with normal N.

		HitInfo hit_info;

		// Make ray to the light source
		Vector light_vector = x - P;
		Ray ray_to_source(P + EPSILON*N, light_vector.normalized());

		intersect(ray_to_source, hit_info);
		if (hit_info.is_hit && light_vector.norm2() > (hit_info.P - P).norm2())
			return 0.;
		return 1.;
	}

	double compute_visibility(const Vector& P, const Vector& N) const{
		return compute_visibility(P, N, L);
	}

	Vector directLighting(const HitInfo& hit_info) {
		double visibility = compute_visibility(hit_info.P, hit_info.N);
		Vector Lo = Vector(0, 0, 0);

		if (visibility == 0.0) {
			return Lo;
		}

		Vector omega_i = L - hit_info.P;

		Lo = (
			(I/(4*M_PI*omega_i.norm2())) *
			(hit_info.material.albedo/M_PI) * 
			visibility *
			std::max(0.0, cosine_similarity(hit_info.N, omega_i))
		);

		return Lo;
	}

	Vector directLighting(const HitInfo& hit_info, const Sphere& light_sphere) {
		// Sample point on the light sphere
		const Vector& C = light_sphere.C;
		double R = light_sphere.R;

		Vector D = (hit_info.P - C).normalized();
		Vector V = random_cos(D);
		Vector x_sphere = C + R * V;

		Vector Lo(0., 0., 0.);
		if (compute_visibility(hit_info.P, hit_info.N, x_sphere) == 0.) 
			return Lo;
		
		Vector omega_i = (x_sphere - hit_info.P).normalized();
		Vector N_sphere = (x_sphere - C).normalized();
		double R2 = (x_sphere - hit_info.P).norm2();

		double form_factor = (
			std::max(0., dot(omega_i, hit_info.N)) *
			std::max(0., dot(-omega_i, N_sphere)) /
			(x_sphere - hit_info.P).norm2()
		);

		double pdf = std::max(0., dot(N_sphere, D))/(M_PI*R2);

		Lo = (
			(light_sphere.albedo()/M_PI) *
			(light_sphere.light_intensity()/(4*M_PI*M_PI*R2)) *
			form_factor /
			pdf
		);

		return Lo;
	}

	Vector indirectLighting(const HitInfo& hit_info, int ray_depth){
		if (ray_depth < 0) return Vector(0,0,0);

		Ray random_ray = Ray(hit_info.P + EPSILON*hit_info.N, random_cos(hit_info.N));
		return hit_info.material.albedo * getColor(random_ray, ray_depth-1, true);
	}

	Vector getColor(const Ray& ray, int ray_depth, bool last_bounce_diffuse=false){
		if (ray_depth < 0) return Vector(0,0,0);

		HitInfo hit_info;

		intersect(ray, hit_info);
		if (!hit_info.is_hit) return Vector(0,0,0);
		// Hit happens
		Vector& P = hit_info.P;
		Vector& N = hit_info.N;

		if (hit_info.material.is_light) {
			if (last_bounce_diffuse)
				return Vector(0.,0.,0.);
			else {
				// different from lecture notes. Maybe change later
				double R2 = (hit_info.P - ray.O).norm2();
				return hit_info.material.albedo * hit_info.material.light_intensity / (4*M_PI*M_PI*R2);
			}
		}

		if (hit_info.material.is_mirror) {
			Vector mirror_direction = ray.u - 2 * dot(ray.u, N) * N;
			Ray mirror_ray = Ray(P + EPSILON * N, mirror_direction);
			return getColor(mirror_ray, ray_depth - 1);
		}

		if (hit_info.material.is_transparent) {
			double n1 = 1; 
			double n2 = hit_info.material.refractive_index;

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

		// Demo code
		Vector Lo_direct = directLighting(hit_info);
		for (size_t i=0; i<objects.size(); i++) {
			if (objects[i].is_light()) {
				Lo_direct += directLighting(hit_info, objects[i]);
			}
		}

		return Lo_direct + indirectLighting(hit_info, ray_depth);
	}

	void add_sphere(const Sphere& s) {
		objects.push_back(s);
	}
	std::vector<Sphere> objects;
};


class Camera{
public:
	// Create Better Design with Inheritance Later...

	// Pinhole Camera
	Camera(Vector camera_center, int width, int height, double alpha){
		this->camera_center = camera_center;
		this->width = width;
		this->height = height;
		this->alpha = alpha;

		// precompute distance from aperture to sensor
		z_film = -width/(2.*tan(alpha/2.));
	}

	// Thin Lens Camera
	Camera(Vector camera_center, int width, int height, double alpha, double focal_distance, double radius_aperture){
		this->camera_center = camera_center;
		this->width = width;
		this->height = height;
		this->alpha = alpha;

		this->focal_distance = focal_distance;
		this->radius_aperture = radius_aperture;
		is_thin_lens = true;

		// precompute distance from aperture to sensor
		z_film = -width/(2.*tan(alpha/2.));
	}

	Vector camera_center;
	int width;
	int height;
	double alpha;
	double z_film;

	// Thin Lens Camera
	double focal_distance;
	double radius_aperture;
	bool is_thin_lens = false;

	Ray pixel_ray_center(int i, int j) const{
		/* Returns the ray from the camera_center to pixel at (row, col) = (i,j) 
		The ray points directly from the camera to the pixel center.
		*/
		Vector ray_direction;
		ray_direction[0] = j - width/2. + 0.5;
		ray_direction[1] = -i + height/2. + 0.5;
		ray_direction[2] = z_film;
		ray_direction = ray_direction.normalized();

		return Ray(camera_center, ray_direction);
	}

	Ray pixel_ray_uniform(int i, int j) const{
		/* Returns a ray from the camera_center to pixel at (row, col) = (i,j) 
		The ray points from the camera to a random position in the pixel.
		*/

		Vector ray_direction;
		ray_direction[0] = j - width/2. + random_uniform();
		ray_direction[1] = -i + height/2. + random_uniform();
		ray_direction[2] = z_film;
		ray_direction = ray_direction.normalized();

		return Ray(camera_center, ray_direction);
	}

	Ray pixel_ray_gaussian(int i, int j) const{
		Vector ray_direction(0,0,0);
		// Random displacement from center
		boxMuller(0.5, ray_direction[0], ray_direction[1]);

		ray_direction[0] += j - width/2. + 0.5;
		ray_direction[1] += -i + height/2. + 0.5;
		ray_direction[2] = z_film;
		ray_direction = ray_direction.normalized();

		return Ray(camera_center, ray_direction);
	}

	Ray pixel_ray_dof(int i, int j) const {
		// Returns random ray to simulate DoF and Antialiasing
		Ray center_ray(pixel_ray_gaussian(i, j));
		Vector P_focus = camera_center + (focal_distance/std::abs(center_ray.u[2]))*center_ray.u;

		// Generate a random point in the lens - later change this to gx_random
		double r = random_uniform() * sqrt(radius_aperture);
		double theta = random_uniform() * 2 * M_PI;

		Vector random_point_lens = camera_center + Vector(r*sin(theta), r*cos(theta), 0.);

		return Ray(random_point_lens, (P_focus-random_point_lens).normalized());
	}
};

int main() {
	int W = 512;
	int H = 512;

	Sphere s1(Material(), Vector(0, 7, 5), 7);
	Sphere s2(Material(Vector(0.5,0.5,1.)), Vector(-14, 7, 5), 3);

	// Try to play with this value later
	s2.material.set_light(1E10); 

	Sphere s3(Material(Vector(0.8, 0.4, 0.8)), Vector(+14, 7, +20), 7);

	Sphere s4(Material(), Vector(-14, 5, +12), 5);
	Sphere s5(Material(Vector(0.5, 1., 0.5)), Vector(+1, 0, +45), 1);

	Sphere left_wall(Material(Vector(0.5, 0.8, 0.1)), Vector(-1000, 0, 0), 940);
	Sphere right_wall(Material(Vector(0.9, 0.2, 0.3)), Vector(1000, 0, 0), 940);
	Sphere ceiling(Material(Vector(0.3, 0.5, 0.3)), Vector(0, 1000, 0), 940);
	Sphere floor(Material(Vector(0.1, 0.1, 0.8)), Vector(0, -1000, 0), 1000);
	Sphere front_wall(Material(Vector(0., 0.8, 0.4)), Vector(0, 0, -1000), 940);
	Sphere behind_wall(Material(Vector(1., 0.2, 0.1)), Vector(0, 0, 1000), 940);

	Vector camera_center(0, 0, 50);	

	double alpha = 60.*M_PI/180.;
	// double I = 8E9;
	double I = 4E9;

	Vector L(-10, 20, 50);
	Scene scene(L, I);

	scene.add_sphere(s1);
	scene.add_sphere(s2);
	scene.add_sphere(s3);
	scene.add_sphere(s4);
	scene.add_sphere(s5);

	scene.add_sphere(left_wall);
	scene.add_sphere(right_wall);
	scene.add_sphere(ceiling);
	scene.add_sphere(floor);
	scene.add_sphere(front_wall);
	scene.add_sphere(behind_wall);

	// Input
	double focal_distance;
	double radius_aperture;

	std::cout << "Focal distance: ";
	std::cin >> focal_distance;
	//focal_distance = 80;

	std::cout << "Aperture radius: ";
	std::cin >> radius_aperture;
	// radius_aperture = 1E-5;

	Camera camera(camera_center, W, H, alpha, focal_distance, radius_aperture);

	std::vector<unsigned char> image(W * H * 3, 0);

	int NUMBER_OF_RAYS;
	std::cout << "Number of rays:";
	std::cin >> NUMBER_OF_RAYS;

	#pragma omp parallel for
	for (int i = 0; i < H; i++) {
		for (int j = 0; j < W; j++) {
			Vector color = Vector(0,0,0);

			for(int _counter=0; _counter<NUMBER_OF_RAYS; _counter++){
				Ray r = camera.pixel_ray_dof(i, j);
				// Ray r = camera.pixel_ray_gaussian(i, j);
				color = color + scene.getColor(r, 5);
			}

			color = color / NUMBER_OF_RAYS;


			image[(i * W + j) * 3 + 0] = std::min(color[0], 255.);
			image[(i * W + j) * 3 + 1] = std::min(color[1], 255.);
			image[(i * W + j) * 3 + 2] = std::min(color[2], 255.);

			image[(i * W + j) * 3 + 0] = std::min(std::pow(color[0], 0.45), 255.);
			image[(i * W + j) * 3 + 1] = std::min(std::pow(color[1], 0.45), 255.);
			image[(i * W + j) * 3 + 2] = std::min(std::pow(color[2], 0.45), 255.);
		}
	}

	stbi_write_png("image.png", W, H, 3, &image[0], 0);

	return 0;
}