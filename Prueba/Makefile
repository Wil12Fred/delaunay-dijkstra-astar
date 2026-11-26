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

tests/test_algorithms: tests/test_algorithms.cc main.cc aest.h delaunay.h grafo.h
	$(CXX) $(CXXFLAGS) -o $@ tests/test_algorithms.cc $(LDLIBS)

# The program and the tests write aristas.txt, camino.txt and comprobar*.txt
# in their working directory, so the unit tests run in tests/run/ and the
# program in a scratch copy. The GUI run always uses its own xvfb-run display.
test: main tests/test_algorithms
	mkdir -p tests/run
	cd tests/run && ../test_algorithms > log.txt || { cat log.txt; exit 1; }
	tail -1 tests/run/log.txt
	./tests/run_program.sh

clean:
	rm -f *.o *.s main trans tests/test_algorithms
	rm -f tests/run/*.txt

.PHONY: all test clean
