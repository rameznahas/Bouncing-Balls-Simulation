#include <iostream>
#include "bouncing_balls_sim.h"

bouncing_balls_sim sim;

void display() {
	sim.update();
}

int main(int argc, char **argv) {
	sim = bouncing_balls_sim(&argc, argv);
	sim.start(display);

	return 0;
}