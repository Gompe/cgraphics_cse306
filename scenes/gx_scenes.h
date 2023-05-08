#pragma once

#include "../gx_camera.h"
#include "../gx_scene.h"

#include "functional"

struct RefScene {
    Camera *camera_ptr;
    Scene *scene_ptr;
    int W;
    int H;

    RefScene(Camera *camera_ptr, Scene *scene_ptr, int W, int H)
    : camera_ptr(camera_ptr), scene_ptr(scene_ptr), W(W), H(H)
    {}

};

// All functions will have signature 
// RefScene(int W, int H)

static RefScene getScene0(int W, int H) {
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

    // Meshes -- The two cats
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

    // Light Position and Intensity (set intensity to 0 if only spherical light is desired)
    Vector L(0, 30, 20);
    double I = 4E6;
    Scene *scene_ptr = new Scene(L, I);

	scene_ptr->insertObject(sphere_1);
	scene_ptr->insertObject(sphere_2);
	scene_ptr->insertObject(s3);
	scene_ptr->insertObject(s4);
	scene_ptr->insertObject(s5);

	scene_ptr->insertObject(left_wall);
	scene_ptr->insertObject(right_wall);
	scene_ptr->insertObject(ceiling);
	scene_ptr->insertObject(floor);
	scene_ptr->insertObject(front_wall);
	scene_ptr->insertObject(behind_wall);

	// Most complex object at the end of intersection loop...
	scene_ptr->insertObject(mesh);
	scene_ptr->insertObject(mesh2);

    double alpha = 60.*M_PI/180.;
    Vector camera_center(0, 10, 50);
	Camera *camera_ptr = new PinholeCamera(camera_center, W, H, alpha);

    return RefScene(camera_ptr, scene_ptr, W, H);
}

static RefScene getScene1(int W, int H) {
    Object *sphere_1 = new Sphere(Vector(15, 7, 5), 7);
	sphere_1->material.is_mirror = true;

	Object *sphere_2 = new Sphere(Vector(0, 7, 5), 7);
	sphere_2->material.is_transparent = true;

	Object *sphere_3 = new Sphere(Vector(-15, 7, 5), 7);

	Object *left_wall = new Sphere(Vector(-1000, 0, 0), 940, Material(Vector(0.5, 0.8, 0.1)));
	Object *right_wall = new Sphere(Vector(1000, 0, 0), 940, Material(Vector(0.9, 0.2, 0.3)));
	Object *ceiling = new Sphere(Vector(0, 1000, 0), 940, Material(Vector(0.3, 0.5, 0.3)));
	Object *floor = new Sphere(Vector(0, -1000, 0), 1000, Material((Vector(0.8, 0.8, 0.8))));
	Object *front_wall = new Sphere(Vector(0, 0, -1000), 910, Material(Vector(0.1, 0.6, 0.7)));
	Object *behind_wall = new Sphere(Vector(0, 0, 1000), 940, Material(Vector(0.8, 0.6, 0.8)));


    // Light Position and Intensity (set intensity to 0 if only spherical light is desired)
    Vector L(0, 30, 20);
    double I = 8E6;
    Scene *scene_ptr = new Scene(L, I);
	scene_ptr->setIndirectLighting(false);

	scene_ptr->insertObject(sphere_1);
	scene_ptr->insertObject(sphere_2);
	scene_ptr->insertObject(sphere_3);

	scene_ptr->insertObject(left_wall);
	scene_ptr->insertObject(right_wall);
	scene_ptr->insertObject(ceiling);
	scene_ptr->insertObject(floor);
	scene_ptr->insertObject(front_wall);
	scene_ptr->insertObject(behind_wall);

    double alpha = 60.*M_PI/180.;
    Vector camera_center(0, 10, 50);
	Camera *camera_ptr = new PinholeCamera(camera_center, W, H, alpha);

    return RefScene(camera_ptr, scene_ptr, W, H);
}

static RefScene getScene2(int W, int H) {
    Object *sphere_1 = new Sphere(Vector(15, 7, 5), 7);
	sphere_1->material.is_mirror = true;

	Object *sphere_2 = new Sphere(Vector(0, 7, 5), 7);
	sphere_2->material.is_transparent = true;

	Object *sphere_3 = new Sphere(Vector(-15, 7, 5), 7);

	Object *left_wall = new Sphere(Vector(-1000, 0, 0), 940, Material(Vector(0.5, 0.8, 0.1)));
	Object *right_wall = new Sphere(Vector(1000, 0, 0), 940, Material(Vector(0.9, 0.2, 0.3)));
	Object *ceiling = new Sphere(Vector(0, 1000, 0), 940, Material(Vector(0.3, 0.5, 0.3)));
	Object *floor = new Sphere(Vector(0, -1000, 0), 1000, Material((Vector(0.8, 0.8, 0.8))));
	Object *front_wall = new Sphere(Vector(0, 0, -1000), 910, Material(Vector(0.1, 0.6, 0.7)));
	Object *behind_wall = new Sphere(Vector(0, 0, 1000), 940, Material(Vector(0.8, 0.6, 0.8)));


    // Light Position and Intensity (set intensity to 0 if only spherical light is desired)
    Vector L(0, 30, 20);
    double I = 8E6;
    Scene *scene_ptr = new Scene(L, I);

	scene_ptr->insertObject(sphere_1);
	scene_ptr->insertObject(sphere_2);
	scene_ptr->insertObject(sphere_3);

	scene_ptr->insertObject(left_wall);
	scene_ptr->insertObject(right_wall);
	scene_ptr->insertObject(ceiling);
	scene_ptr->insertObject(floor);
	scene_ptr->insertObject(front_wall);
	scene_ptr->insertObject(behind_wall);

    double alpha = 60.*M_PI/180.;
    Vector camera_center(0, 10, 50);
	Camera *camera_ptr = new PinholeCamera(camera_center, W, H, alpha);

    return RefScene(camera_ptr, scene_ptr, W, H);
}

static RefScene getScene3(int W, int H) {
    Object *sphere_1 = new Sphere(Vector(23, 15, 5), 15);

	Object *sphere_2 = new Sphere(Vector(0, 4, 5), 4, Material(Vector(0.5, 0.5, 1)));

	Object *sphere_3 = new Sphere(Vector(-16, 9, 5), 2);
	sphere_3->material.set_light(8E6);

	Object *sphere_4 = new Sphere(Vector(7, 2, 25), 2);
	sphere_4->material.is_mirror = true;

	Object *left_wall = new Sphere(Vector(-1000, 0, 0), 940, Material(Vector(0.5, 0.8, 0.1)));
	Object *right_wall = new Sphere(Vector(1000, 0, 0), 940, Material(Vector(0.9, 0.2, 0.3)));
	Object *ceiling = new Sphere(Vector(0, 1000, 0), 940, Material(Vector(0.3, 0.5, 0.3)));
	Object *floor = new Sphere(Vector(0, -1000, 0), 1000, Material((Vector(0.5, 0.5, 0.8))));
	Object *front_wall = new Sphere(Vector(0, 0, -1000), 910, Material(Vector(0.1, 0.6, 0.7)));
	Object *behind_wall = new Sphere(Vector(0, 0, 1000), 940, Material(Vector(0.8, 0.6, 0.8)));


    // Light Position and Intensity (set intensity to 0 if only spherical light is desired)
    Vector L(0, 30, 20);
    double I = 0;
    Scene *scene_ptr = new Scene(L, I);

	scene_ptr->insertObject(sphere_1);
	scene_ptr->insertObject(sphere_2);
	scene_ptr->insertObject(sphere_3);
	scene_ptr->insertObject(sphere_4);

	scene_ptr->insertObject(left_wall);
	scene_ptr->insertObject(right_wall);
	scene_ptr->insertObject(ceiling);
	scene_ptr->insertObject(floor);
	scene_ptr->insertObject(front_wall);
	scene_ptr->insertObject(behind_wall);

    double alpha = 60.*M_PI/180.;
    Vector camera_center(0, 10, 50);
	Camera *camera_ptr = new PinholeCamera(camera_center, W, H, alpha);

    return RefScene(camera_ptr, scene_ptr, W, H);
}

static RefScene getScene4(int W, int H) {
    Object *sphere_1 = new Sphere(Vector(0, 7, 5), 7, Material(Vector(0.5, 0.5, 1)));

	Object *left_wall = new Sphere(Vector(-1000, 0, 0), 940, Material(Vector(0.5, 0.8, 0.1)));
	Object *right_wall = new Sphere(Vector(1000, 0, 0), 940, Material(Vector(0.9, 0.2, 0.3)));
	Object *ceiling = new Sphere(Vector(0, 1000, 0), 940, Material(Vector(0.3, 0.5, 0.3)));
	Object *floor = new Sphere(Vector(0, -1000, 0), 1000, Material((Vector(0.5, 0.5, 0.8))));
	Object *front_wall = new Sphere(Vector(0, 0, -1000), 910, Material(Vector(0.1, 0.6, 0.7)));
	Object *behind_wall = new Sphere(Vector(0, 0, 1000), 940, Material(Vector(0.8, 0.6, 0.8)));

    Vector L(0, 30, 20);
    double I = 2E7;
    Scene *scene_ptr = new Scene(L, I);

	scene_ptr->insertObject(sphere_1);

	scene_ptr->insertObject(left_wall);
	scene_ptr->insertObject(right_wall);
	scene_ptr->insertObject(ceiling);
	scene_ptr->insertObject(floor);
	scene_ptr->insertObject(front_wall);
	scene_ptr->insertObject(behind_wall);

    double alpha = 60.*M_PI/180.;
    Vector camera_center(0, 10, 50);
	Camera *camera_ptr = new AliasedCamera(camera_center, W, H, alpha);

    return RefScene(camera_ptr, scene_ptr, W, H);
}

static RefScene getScene5(int W, int H) {
    Object *sphere_1 = new Sphere(Vector(0, 7, 5), 7, Material(Vector(0.5, 0.5, 1)));

	Object *left_wall = new Sphere(Vector(-1000, 0, 0), 940, Material(Vector(0.5, 0.8, 0.1)));
	Object *right_wall = new Sphere(Vector(1000, 0, 0), 940, Material(Vector(0.9, 0.2, 0.3)));
	Object *ceiling = new Sphere(Vector(0, 1000, 0), 940, Material(Vector(0.3, 0.5, 0.3)));
	Object *floor = new Sphere(Vector(0, -1000, 0), 1000, Material((Vector(0.5, 0.5, 0.8))));
	Object *front_wall = new Sphere(Vector(0, 0, -1000), 910, Material(Vector(0.1, 0.6, 0.7)));
	Object *behind_wall = new Sphere(Vector(0, 0, 1000), 940, Material(Vector(0.8, 0.6, 0.8)));

    Vector L(0, 30, 20);
    double I = 2E7;
    Scene *scene_ptr = new Scene(L, I);

	scene_ptr->insertObject(sphere_1);

	scene_ptr->insertObject(left_wall);
	scene_ptr->insertObject(right_wall);
	scene_ptr->insertObject(ceiling);
	scene_ptr->insertObject(floor);
	scene_ptr->insertObject(front_wall);
	scene_ptr->insertObject(behind_wall);

    double alpha = 60.*M_PI/180.;
    Vector camera_center(0, 10, 50);
	Camera *camera_ptr = new PinholeCamera(camera_center, W, H, alpha);

    return RefScene(camera_ptr, scene_ptr, W, H);
}

static RefScene getScene6(int W, int H) {
    Object *sphere_1 = new Sphere(Vector(0, 7, 5), 7, Material(Vector(0.5, 0.5, 1)));
	Object *sphere_2 = new Sphere(Vector(3, 10, 40), 1, Material(Vector(0.5, 1, 0.5)));
	Object *sphere_3 = new Sphere(Vector(-20, 7, -20), 7, Material(Vector(1, 0.5, 0.5)));

	Object *left_wall = new Sphere(Vector(-1000, 0, 0), 940, Material(Vector(0.5, 0.8, 0.1)));
	Object *right_wall = new Sphere(Vector(1000, 0, 0), 940, Material(Vector(0.9, 0.2, 0.3)));
	Object *ceiling = new Sphere(Vector(0, 1000, 0), 940, Material(Vector(0.3, 0.5, 0.3)));
	Object *floor = new Sphere(Vector(0, -1000, 0), 1000, Material((Vector(0.5, 0.5, 0.8))));
	Object *front_wall = new Sphere(Vector(0, 0, -1000), 910, Material(Vector(0.1, 0.6, 0.7)));
	Object *behind_wall = new Sphere(Vector(0, 0, 1000), 940, Material(Vector(0.8, 0.6, 0.8)));

    Vector L(0, 30, 20);
    double I = 2E7;
    Scene *scene_ptr = new Scene(L, I);

	scene_ptr->insertObject(sphere_1);
	scene_ptr->insertObject(sphere_2);
	scene_ptr->insertObject(sphere_3);

	scene_ptr->insertObject(left_wall);
	scene_ptr->insertObject(right_wall);
	scene_ptr->insertObject(ceiling);
	scene_ptr->insertObject(floor);
	scene_ptr->insertObject(front_wall);
	scene_ptr->insertObject(behind_wall);

    double alpha = 60.*M_PI/180.;
    Vector camera_center(0, 10, 50);

	double focalDistance, radiusAperture;
	focalDistance = 10;
	radiusAperture = 5E-2;

	Camera *camera_ptr = new ProjectiveCamera(camera_center, W, H, alpha, focalDistance, radiusAperture);

    return RefScene(camera_ptr, scene_ptr, W, H);
}

static RefScene getScene7(int W, int H) {

	Object *left_wall = new Sphere(Vector(-1000, 0, 0), 940, Material(Vector(0.5, 0.8, 0.1)));
	Object *right_wall = new Sphere(Vector(1000, 0, 0), 940, Material(Vector(0.9, 0.2, 0.3)));
	Object *ceiling = new Sphere(Vector(0, 1000, 0), 940, Material(Vector(0.3, 0.5, 0.3)));
	Object *floor = new Sphere(Vector(0, -1000, 0), 1000, Material((Vector(0.8, 0.8, 0.8))));
	Object *front_wall = new Sphere(Vector(0, 0, -1000), 910, Material(Vector(0.1, 0.6, 0.7)));
	Object *behind_wall = new Sphere(Vector(0, 0, 1000), 940, Material(Vector(0.8, 0.6, 0.8)));

    // Meshes -- The two cats
	Object *mesh = new TriangleMesh("./data/cat/cat.obj", Material(Vector(0.8, 0.4, 0.2)));
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

	mesh2->material.is_transparent = true;

	Object *mesh3 = new TriangleMesh("./data/cat/cat.obj");

	mesh3->transformScale(0.8);
	mesh3->transformTranslate(Vector(0,0,-25));
	// mesh3->transformRotate(2, -1.5);
	mesh3->transformTranslate(Vector(-30, 0, 0));
	mesh3->transformScale(0.2);
	mesh3->transformTranslate(Vector(-20, 0, 0));
	mesh3->transformScale(1.4);
	mesh3->transformTranslate(Vector(+14, 0, 0));

	mesh3->material.is_mirror = true;

    // Light Position and Intensity (set intensity to 0 if only spherical light is desired)
    Vector L(0, 30, 20);
    double I = 4E6;
    Scene *scene_ptr = new Scene(L, I);


	scene_ptr->insertObject(left_wall);
	scene_ptr->insertObject(right_wall);
	scene_ptr->insertObject(ceiling);
	scene_ptr->insertObject(floor);
	scene_ptr->insertObject(front_wall);
	scene_ptr->insertObject(behind_wall);

	// Most complex object at the end of intersection loop...
	scene_ptr->insertObject(mesh);
	scene_ptr->insertObject(mesh2);
	scene_ptr->insertObject(mesh3);

    double alpha = 60.*M_PI/180.;
    Vector camera_center(0, 10, 50);
	Camera *camera_ptr = new PinholeCamera(camera_center, W, H, alpha);

    return RefScene(camera_ptr, scene_ptr, W, H);
}


namespace my_scenes {

static std::vector<std::function<RefScene(int, int)>> listGetScenes {
	getScene0,
	getScene1,
	getScene2,
	getScene3,
	getScene4,
	getScene5,
	getScene6,
	getScene7,
};
RefScene getScene(size_t sceneIdx, int W, int H) {
	if (sceneIdx >= listGetScenes.size()) {
		fprintf(stderr, "Error: Scene Index out of range.\n");
		fprintf(stderr, "\tCurrently valid scene indexes are integers from 0 to %d (inclusive).\n", (int) listGetScenes.size()-1);
		exit(1);
	}

	return listGetScenes[sceneIdx](W, H);
}

}