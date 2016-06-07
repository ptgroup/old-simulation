all: clean sim

sim:
	gcc -std=c99 -Wall -Wextra -o sim sim.c rs232.c serial.c script.c helper.c -lm

clean:
	rm -f sim.exe sim
