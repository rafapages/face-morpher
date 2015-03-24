#CFLAGS=-g
OBJECTS = main.o mesh.o	\
morpher.o camera.o

all: faceMorpher


faceMorpher: $(OBJECTS)
	g++ -o faceMorpher $(OBJECTS) $(CXXFLAGS) -lopencv_core -lopencv_highgui -lopencv_imgproc -lopencv_objdetect -lfreeimageplus

clean:
	rm -f *.o 
	rm -f faceMorpher