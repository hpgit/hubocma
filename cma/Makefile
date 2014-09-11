all:
	mkdir -p build
	g++ -c cma/cmaes.cpp -o build/cmaes.o
	g++ -c example_boundary/boundary_transformation.cpp -o build/boundary_transformation.o
	ar rcs usr/lib/libcma.a build/cmaes.o
	ar rcs usr/lib/libboundary_transformation.a build/boundary_transformation.o


clean:
	rm -rf build
	rm -rf usr/lib/*.a
