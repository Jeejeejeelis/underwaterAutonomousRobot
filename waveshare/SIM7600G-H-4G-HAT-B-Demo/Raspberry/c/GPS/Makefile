CC=g++
DLIBS=-lbcm2835 -lrt -lpthread
OBJS= GPS.o ../lib/arduPi.o ../lib/sim7x00.o
SMS:$(OBJS) 
	$(CC) -Wall -o GPS $(OBJS)  $(DLIBS) 

%.o:%.c
	$(CC) -c -o $@ $<

clean:
	rm -f *.o GPS

