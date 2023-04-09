render: main.cpp
	g++ -O2 main.cpp gx_vector.cpp gx_random.cpp gx_camera.cpp gx_mesh.cpp gx_material.cpp gx_geometry.cpp gx_object.cpp -fopenmp -o render

clean:
	rm render

remove:
	rm image.png render