CC=mpicc # use this variantfor C language programs only
#CC=mpiCC  # use variant for C++ programs to ensure linking with C++ libraries
CXX=mpiCC
#CC=arm-linux-gnueabihf-gcc
CFLAGS += -g -std=c99 -pedantic -Wall
CXXFLAGS += -g -std=gnu++11 -pedantic -Wall
#LDFLAGS += -lrt -lpthread

#CFLAGS += -fopenmp
#LDFLAGS += -fopenmp

CFLAGS += -lmpi
LDFLAGS += -lmpi

all: program

program: program.o

.PHONY : depend dep all run

run: program
	mpirun --host localhost,localhost,localhost -np 3 ./program

dep:
	$(CC) $(CFLAGS) $(CPPFLAGS) -w -E -M *.[Cc] \
	  > depend

depend:
	@touch depend

clean:
	rm -f *.o *.a program depend

-include depend
