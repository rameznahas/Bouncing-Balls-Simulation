#include <iostream>
#include "bouncing_balls_sim.h"

bouncing_balls_sim sim;

int main(int argc, char **argv) {
	sim = bouncing_balls_sim(&argc, argv);
	sim.start([]() {
		sim.update(); 
	});

	return 0;
}