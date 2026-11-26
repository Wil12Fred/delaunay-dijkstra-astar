CXX      ?= g++
CXXFLAGS ?= -std=c++11 -Wall -O2

ifeq ($(shell uname),Darwin)
LDLIBS := -framework OpenGL -framework GLUT -lobjc
else
LDLIBS := -lglut -lGLU -lGL
endif

all: main trans

main: main.cc aest.h delaunay.h grafo.h
	$(CXX) $(CXXFLAGS) -o $@ main.cc $(LDLIBS)

trans: Trans.cpp
	$(CXX) $(CXXFLAGS) -o $@ Trans.cpp

clean:
	rm -f *.o *.s main trans

.PHONY: all clean
