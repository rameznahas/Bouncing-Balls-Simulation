#include <random>
#include <iostream>
#include <string>
#include "glew.h"
#include "freeglut.h"
#include "bouncing_balls_sim.h"
#define WWIDTH 800
#define WHEIGHT 800
#define UPDATE_FREQ 1.f / 30
#define BALL_COUNT 10

bouncing_balls_sim::bouncing_balls_sim(int* argc, char **argv)
	:
	balls(*argc > 1 ? std::stoi(argv[1]) : BALL_COUNT),
	t_workers(*argc > 1 ? std::stoi(argv[1]) : BALL_COUNT),
	wall_computation(*argc > 1 ? std::stoi(argv[1]) : BALL_COUNT),
	do_frame(*argc > 1 ? std::stoi(argv[1]) + 1 : BALL_COUNT + 1),
	frame_done(*argc > 1 ? std::stoi(argv[1]) + 1 : BALL_COUNT + 1),
	GRAVITY(0.f, -1.5f),
	previous_t(0),
	current_t(0),
	delta_t(UPDATE_FREQ),
	program_running(true)
{
	glutInit(argc, argv);
	glutInitWindowPosition(-1, -1);
	glutInitWindowSize(WWIDTH, WHEIGHT);
	glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE | GLUT_ALPHA);
	glutCreateWindow("Bouncing Balls Simulation");
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	glEnable(GL_BLEND);
}

/*
	The worker threads collision computation (ball-wall and ball-ball).
*/
void bouncing_balls_sim::operator()(ball& current, int start, int end) {
	while (program_running) {
		// wait for control thread to give the command to start
		do_frame.wait();
		// ball-wall computation
		wall_bounce(current);
		// wait for all workers to be done with ball-wall computation
		wall_computation.wait();
		
		// ball-ball computation
		for (int i = start; i < end; ++i) {
			ball* current = pairs[i].first;
			ball* other = pairs[i].second;

			// check for aabb overlap
			// if true, balls are close enough, computation is worth it.
			if (aabb(*current, *other)) {
				vector2d c = current->center - other->center;
				float min_dist = current->radius + other->radius;

				// balls are close enough, but it does not mean they have collided.
				// check for ball collision.
				// if true, collision occured, handle it
				if (powf(c.x, 2.f) + powf(c.y, 2.f) <= powf(min_dist, 2.f)) {
					float distance = vector2d::magnitude(c);
					float overlap = 0.5f * (distance - current->radius - other->radius);

					vector2d dir = vector2d::normalize(c);

					current->center += -overlap * dir;
					other->center += overlap * dir;

					vector2d v = current->velocity - other->velocity;
					int m = current->mass + other->mass;
					float mag = powf(distance, 2.f);
					float dot_vc = vector2d::dot(v, c);
					float ratio = 2.f * dot_vc / (m * mag);

					current->velocity -= (other->mass * ratio * c);
					other->velocity += (current->mass * ratio * c);
				}
			}
		}
		// signal to control that this thread is done.
		// once all are done, control will go ahead with drawing
		// the new frame.
		frame_done.wait();
	}
}

/*
	The control thread task.
*/
void bouncing_balls_sim::update() {
	//update current clock time
	current_t = clock();
	delta_t = (float)(current_t - previous_t) / CLOCKS_PER_SEC;

	// don't draw if delta_t is faster than 30 fps
	if (delta_t < UPDATE_FREQ) return;

	// store last draw time
	previous_t = current_t;

	// update the balls' velocity and positions
	for (ball& ball : balls) {
		ball.velocity += delta_t * GRAVITY;
		ball.center += delta_t * ball.velocity;
	}

	// notify worker threads to start
	do_frame.wait();
	// wait for workers to notifiy that they're done
	frame_done.wait();

	draw();
	glutSwapBuffers();
}

void bouncing_balls_sim::wall_bounce(ball& current) {
	float t_wall = 1.f - current.radius;
	float b_wall = current.radius - 1.f;
	float r_wall = t_wall;
	float l_wall = b_wall;

	if (current.center.x > r_wall) {
		current.center.x = r_wall;
		current.velocity.x *= -1.f;
	}
	else if (current.center.x < l_wall) {
		current.center.x = l_wall;
		current.velocity.x *= -1.f;
	}

	if (current.center.y > t_wall) {
		current.center.y = t_wall;
		current.velocity.y *= -1.f;
	}
	else if (current.center.y < b_wall) {
		current.center.y = b_wall;
		current.velocity.y *= -1.f;
	}
}

void bouncing_balls_sim::draw() {
	glClearColor(0.25f, 0.25f, 0.25f, 1.f);
	glClear(GL_COLOR_BUFFER_BIT);

	for (ball& ball : balls) {
		glBegin(GL_POLYGON);
		glColor4f(ball.color.x, ball.color.y, ball.color.z, 0.25f);

		for (int i = 0; i < NUM_POINTS; ++i) {
			float angle = i * DEGREE_TO_RAD;

			glVertex2d
			(
				ball.radius * cos(angle) + ball.center.x,	// x-coord
				ball.radius * sin(angle) + ball.center.y	// y-coord
			);
		}

		glEnd();
	}
}

void bouncing_balls_sim::start(void(*callback)()) {
	init();

	glutDisplayFunc(callback);
	glutIdleFunc(callback);
	glutMainLoop();

	// ensure worker threads are done computing and have been properly 
	// destroyed before exiting the program.
	program_running = false;
	do_frame.wait();
	frame_done.wait();

	for (std::thread& worker : t_workers) {
		worker.join();
	}
}

/*
	Initialization of simulation.
		- Creates balls with random radius, centers and velocities.
		- Caches all the unique ball pairs (optimization).
		- Creates the worker threads and assigns their tasks.
*/
void bouncing_balls_sim::init() {
	///////////////////////////init balls///////////////////////////
	std::random_device rd;
	std::mt19937 gen(rd());
	std::uniform_int_distribution<int> rad(1, 3);
	std::uniform_real_distribution<float> vel(-1.f, 1.f);

	for (unsigned int i = 0; i < balls.size(); ++i) {
		int test = rad(gen);
		float radius = MIN_RADIUS * test; // random radius

		float ur_bound = radius - 1;
		float ll_bound = 1 - radius;

		std::uniform_real_distribution<float> coord(ur_bound, ll_bound); // so we dont get balls out of bounds
		vector2d center(coord(gen), coord(gen));

		int weight = (int)(radius * 100.0f);
		vector2d velocity(vel(gen), vel(gen));
		balls[i] = ball(radius, center, velocity, weight);
	}
	////////////////////////////////////////////////////////////////
	///////////////////////////init pairs///////////////////////////
	for (unsigned int i = 0; i < balls.size(); ++i) {
		for (unsigned int j = (i + 1); j < balls.size(); ++j) {
			pairs.push_back({ &balls[i], &balls[j] });
		}
	}
	////////////////////////////////////////////////////////////////
	//////////////////////////init threads//////////////////////////
	unsigned int div = pairs.size() / t_workers.size();
	unsigned int mod = pairs.size() % t_workers.size();

	unsigned int start = 0;
	unsigned int end = mod;
	for (unsigned int i = 0; i < t_workers.size(); ++i) {
		if (i < mod) {
			t_workers[i] = std::thread(std::ref(*this), std::ref(balls[i]), start, end);
			start = end;
			end += mod;
		}
		else {
			end = start + div;
			t_workers[i] = std::thread(std::ref(*this), std::ref(balls[i]), start, end);
			start = end;
		}
	}
	////////////////////////////////////////////////////////////////
}

/*
	Axis-aligned bounding-box check (optimization).

	Checks if balls are close enough to go ahead with actual collision
	computation (which is more expensive).
	
	If true, they are close enough.
*/
bool bouncing_balls_sim::aabb(const ball& current, const ball& other) {
	float min_dist = current.radius + other.radius;

	return (current.center.x + min_dist > other.center.x
		&& current.center.y + min_dist > other.center.y
		&& other.center.x + min_dist > current.center.x
		&& other.center.y + min_dist > current.center.y);
}
