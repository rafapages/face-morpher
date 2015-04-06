#CFLAGS=-g
OBJECTS = main.o mesh.o	\
morpher.o camera.o triangle.o \
pyramid.o

all: faceMorpher


faceMorpher: $(OBJECTS)
	g++ -o faceMorpher $(OBJECTS) $(CXXFLAGS) -lCGAL -lgmp -lmpfr

clean:
	rm -f *.o 
	rm -f faceMorpher