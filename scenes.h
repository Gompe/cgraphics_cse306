void make_scene_1(Secene &scene) {
    Sphere S(Vector(0, 0, 0), 10, Vector(0., 0.5, 1.), false, true);

	Sphere left_wall(Vector(-1000, 0, 0), 940, Vector(0.5, 0.8, 0.1));
	Sphere right_wall(Vector(1000, 0, 0), 940, Vector(0.9, 0.2, 0.3));
	Sphere ceiling(Vector(0, 1000, 0), 940, Vector(0.3, 0.5, 0.3));
	Sphere floor(Vector(0, -1000, 0), 940, Vector(0.6, 0.5, 0.7));
	Sphere front_wall(Vector(0, 0, -1000), 940, Vector(0.1, 0.6, 0.7));
	Sphere behind_wall(Vector(0, 0, 1000), 940, Vector(0.0, 0.2, 0.9));

	Vector camera_center(0, 0, 55);	
	double alpha = 60.*M_PI/180.;
	double I = 2E10;
	Vector L(-10, 20, 40);

	Scene scene(L, I);

	scene.add_sphere(S);
	scene.add_sphere(left_wall);
	scene.add_sphere(right_wall);
	scene.add_sphere(ceiling);
	scene.add_sphere(floor);
	scene.add_sphere(front_wall);
	scene.add_sphere(behind_wall);
}

void make_scene_2(Scene &scene) {
    Sphere s1(Vector(0, 5, 5), 5, Vector(1., 0.5, 1.), false, true);
	Sphere s2(Vector(-10, 5, 5), 5, Vector(0, 0.5, 1.), false, false);
	Sphere s3(Vector(+10, 5, 5), 5, Vector(0.3, 1., 0.5), true, false);

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
	scene.add_sphere(s3);

	scene.add_sphere(left_wall);
	scene.add_sphere(right_wall);
	scene.add_sphere(ceiling);
	scene.add_sphere(floor);
	scene.add_sphere(front_wall);
	scene.add_sphere(behind_wall);
}