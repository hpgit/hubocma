.PHONY: all clean sloop debug install_dependencies_ubuntu

all: sloop

clean: 
	cd build_release;make clean;rm CMakeCache.txt;rm Makefile

sloop: build_release/Makefile
	cd build_release;make --no-print-directory
	cp build_release/sloop .

debug: build_debug/Makefile
	cd build_debug;make --no-print-directory
	cp build_debug/sloop ./sloop_d

build_release/Makefile: CMakeLists.txt
	mkdir -p build_release;cd build_release;export CC=gcc;export CXX=g++;cmake CMAKE_BUILD_TYPE=Release ..

build_debug/Makefile: CMakeLists.txt
	mkdir -p build_debug;cd build_debug;export CC=gcc;export CXX=g++;cmake -DCMAKE_BUILD_TYPE=Debug ..

install_dependencies_ubuntu:
	sudo apt-get install build-essential cmake freeglut3-dev
	
