CC=g++
DLIBS=-lbcm2835 -lrt -lpthread
OBJS= TCP.o ../lib/arduPi.o ../lib/sim7x00.o
SMS:$(OBJS) 
	$(CC) -Wall -o TCP $(OBJS)  $(DLIBS) 

%.o:%.c
	$(CC) -c -o $@ $<

clean:
	rm -f *.o TCP

