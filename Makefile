CC=gcc
CXX=g++

6strut: 6strut.cpp
	g++ 6strut.cpp -o 6strut -lode -ldrawstuff -lX11 -lGLU -lGL -lpthread

3strut: tensegrity.cpp
	g++ tensegrity.cpp -o tensegrity -lode -ldrawstuff -lX11 -lGLU -lGL -lpthread
