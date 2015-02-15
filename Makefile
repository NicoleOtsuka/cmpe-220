CC = gcc
CFLAGS = -g -O2
OBJECTS = cpu.o

cpu: $(OBJECTS)
	$(CC) $(CFLAGS) $(OBJECTS) -o cpu

%.o: %.c
	$(CC) $(CFLAGS) -c $<

.PHONY: clean

clean:
	rm -f $(OBJECTS) cpu
