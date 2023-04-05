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


class Sphere {
public:
	// ...
	Sphere(const Vector& C, double R, const Vector& rho, bool is_mirror = false, bool is_transparent = false) : C(C), R(R), rho(rho), is_mirror(is_mirror), is_transparent(is_transparent) {};
	Vector C;
	double R;
	Vector rho;
	bool is_mirror, is_transparent;
	bool intersect(const Ray& r, Vector& P, Vector& N, double& t) const {

		double delta = sqr(dot(r.u, r.O - C)) - dot(r.O - C, r.O - C) + R * R;
		if (delta >= 0) {
			double t1 = dot(r.u, C - r.O) - sqrt(delta);
			double t2 = dot(r.u, C - r.O) + sqrt(delta);
			if (t2 < 0) return false;
			if (t1 > 0) {
				t = t1;
			}
			else {
				t = t2;
			}
			P = r.O + t * r.u;
			N = P - C;
			N = N.normalized();
			return true;
		}

		return false;
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

	bool intersect(const Ray& r, Vector& P, Vector& N, double& t, size_t& sphere_idx) const {
		bool has_intersection = false;
		t = std::numeric_limits<double>::max();
		for(size_t i = 0; i < objects.size(); i++) {
			double t_tmp;
			Vector P_tmp, N_tmp;
			if (objects[i].intersect(r, P_tmp, N_tmp, t_tmp)) {
				if (t_tmp < t) {
					t = t_tmp;
					P = P_tmp;
					N = N_tmp;
					sphere_idx = i;
					has_intersection = true;
				}
			}
		}

		return has_intersection;
	}


	double compute_visibility(const Vector& P, const Vector& N) const{
		/* Returns the visibility at point P with normal N. */
		Vector light_vector = L - P;
		double distance_to_light_squared = light_vector.norm2();

		light_vector = light_vector.normalized();
		Ray light_ray(P + EPSILON*N, light_vector);

		double t;

		// Placeholder variables
		Vector _0;
		size_t _1;

		if (intersect(light_ray, _0, _0, t, _1)) {
			if (t*t < distance_to_light_squared) {
				return 0.;
			}
		}

		// No intersection
		return 1.;
	}

	Vector directLighting(const Vector& P, const Vector& N, size_t sphere_id){
		/* Returns direct lighting at point P of object sphre_id (normal also
		passed for efficiency)*/
		double visibility = compute_visibility(P, N);
		Vector Lo = Vector(0, 0, 0);

		if (visibility == 0.0) {
			return Lo;
		}

		Vector omega_i = L - P;

		Lo = (
			(I/(4*M_PI*omega_i.norm2())) *
			(objects[sphere_id].rho/M_PI) * 
			visibility *
			std::max(0.0, cosine_similarity(N, omega_i))
		);

		return Lo;
	}

	Vector indirectLighting(const Vector& P, const Vector& N, size_t sphere_id, int ray_depth){
		if (ray_depth < 0) return Vector(0,0,0);

		Ray random_ray = Ray(P + EPSILON*N, random_cos(N));
		return objects[sphere_id].rho * getColor(random_ray, ray_depth-1);
	}

	Vector getColor(const Ray& ray, int ray_depth){
		if (ray_depth < 0) return Vector(0,0,0);

		Vector P, N;
		double t;
		size_t sphere_id;
		
		if (!intersect(ray, P, N, t, sphere_id)){
			return Vector(0,0,0);
		}

		// Intersection happens
		if (objects[sphere_id].is_mirror) {
			Vector mirror_direction = ray.u - 2 * dot(ray.u, N) * N;
				Ray mirror_ray = Ray(P + EPSILON * N, mirror_direction);
				return getColor(mirror_ray, ray_depth - 1);
		}

		if (objects[sphere_id].is_transparent) {
				double n1 = 1;
				double n2 = 1.4;
				double n = n1 / n2;
				Vector Ntransp = N;
				if (dot(ray.u, N) > 0) {
					std::swap(n1, n2);
					n = 1/n;
					Ntransp = -Ntransp;
				}
				Vector tTangent, tNormal;
				double cos = dot(ray.u, Ntransp);
				tTangent = n * (ray.u - cos * Ntransp);
				double rad = 1 - sqr(n) * (1 - sqr(cos));
				if (rad < 0) {
					Vector mirror_direction = ray.u - 2 * dot(ray.u, Ntransp) * Ntransp;
					Ray mirror_ray = Ray(P - EPSILON * N, mirror_direction);
					return getColor(mirror_ray, ray_depth - 1);
				}
				tNormal = -sqrt(rad) * Ntransp;
				Ray refractiveRay(P - EPSILON * Ntransp, tTangent + tNormal);
				return getColor(refractiveRay, ray_depth - 1); 
		}

		// return directLighting(P, N, sphere_id);
		return directLighting(P, N, sphere_id) + indirectLighting(P, N, sphere_id, ray_depth);
	}

	void add_sphere(const Sphere& s) {
		objects.push_back(s);
	}
	std::vector<Sphere> objects;
};

class Camera{
public:
	Camera(Vector camera_center, int width, int height, double alpha){
		this->camera_center = camera_center;
		this->width = width;
		this->height = height;
		this->alpha = alpha;

		// precompute z direction of ray
		_z_direction = -width/(2.*tan(alpha/2.));
	}

	Vector camera_center;
	int width;
	int height;
	double alpha;
	double _z_direction;

	Ray pixel_ray_center(int i, int j) const{
		/* Returns the ray from the camera_center to pixel at (row, col) = (i,j) 
		The ray points directly from the camera to the pixel center.
		*/
		Vector ray_direction;
		ray_direction[0] = j - width/2. + 0.5;
		ray_direction[1] = -i + height/2. + 0.5;
		ray_direction[2] = _z_direction;
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
		ray_direction[2] = _z_direction;
		ray_direction = ray_direction.normalized();

		return Ray(camera_center, ray_direction);
	}

	Ray pixel_ray_gaussian(int i, int j) const{
		Vector ray_direction(0,0,0);
		// Random displacement from center
		boxMuller(0.5, ray_direction[0], ray_direction[1]);

		ray_direction[0] += j - width/2. + 0.5;
		ray_direction[1] += -i + height/2. + 0.5;
		ray_direction[2] = _z_direction;
		ray_direction = ray_direction.normalized();

		return Ray(camera_center, ray_direction);
	}
};

int main() {
	int W = 512;
	int H = 512;

	Sphere s1(Vector(0, 7, 5), 7, Vector(1., 0.5, 1.), false, true);
	Sphere s2(Vector(0, 7, 5), 6, Vector(1., 0.5, 1), true, false);

	Sphere left_wall(Vector(-1000, 0, 0), 940, Vector(0.5, 0.8, 0.1));
	Sphere right_wall(Vector(1000, 0, 0), 940, Vector(0.9, 0.2, 0.3));
	Sphere ceiling(Vector(0, 1000, 0), 940, Vector(0.3, 0.5, 0.3));
	Sphere floor(Vector(0, -1000, 0), 1000, Vector(0.6, 0.8, 0.7));
	Sphere front_wall(Vector(0, 0, -1000), 940, Vector(0.1, 0.6, 0.7));
	Sphere behind_wall(Vector(0, 0, 1000), 940, Vector(0.0, 0.2, 0.9));

	Vector camera_center(0, 0, 50);	
	double alpha = 60.*M_PI/180.;
	double I = 2E9;
	Vector L(-10, 20, 40);
	Scene scene(L, I);

	scene.add_sphere(s1);
	scene.add_sphere(s2);

	scene.add_sphere(left_wall);
	scene.add_sphere(right_wall);
	scene.add_sphere(ceiling);
	scene.add_sphere(floor);
	scene.add_sphere(front_wall);
	scene.add_sphere(behind_wall);


	Camera camera(camera_center, W, H, alpha);

	std::vector<unsigned char> image(W * H * 3, 0);

	int NUMBER_OF_RAYS;
	std::cout << "Number of rays:";
	std::cin >> NUMBER_OF_RAYS;

	#pragma omp parallel for
	for (int i = 0; i < H; i++) {
		for (int j = 0; j < W; j++) {
			Vector color = Vector(0,0,0);

			for(int _counter=0; _counter<NUMBER_OF_RAYS; _counter++){
				Ray r = camera.pixel_ray_gaussian(i, j);
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