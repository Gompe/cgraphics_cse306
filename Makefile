render: main.cpp
	g++ -O2 main.cpp gx_vector.cpp gx_random.cpp gx_mesh.cpp -fopenmp -o render

clean:
	rm render

remove:
	rm image.png render