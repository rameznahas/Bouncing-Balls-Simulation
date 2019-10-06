#include <random>
#include <iostream>
#include <string>
#include "glew.h"
#include "freeglut.h"
#include "bouncing_balls_sim.h"
#define WWIDTH 800
#define WHEIGHT 800
#define UPDATE_FREQ 1.f / 30

bouncing_balls_sim::bouncing_balls_sim(int* argc, char **argv)
	:
	balls(std::stoi(argv[1])),
	t_workers(std::stoi(argv[1])),
	computation_barrier(std::stoi(argv[1])),
	GRAVITY(0.f, -0.015f),
	previous_t(0),
	current_t(0),
	delta_t(UPDATE_FREQ),
	accumulated_t(0),
	program_running(true),
	do_frame(false)
{
	glutInit(argc, argv);
	glutInitWindowPosition(-1, -1);
	glutInitWindowSize(WWIDTH, WHEIGHT);
	glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE | GLUT_ALPHA);
	glutCreateWindow("Bouncing Balls Simulation");
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	glEnable(GL_BLEND);
}

void bouncing_balls_sim::wall_bounce(ball& current) {
	float t_wall = 1 - current.radius;
	float b_wall = current.radius - 1;
	float r_wall = t_wall;
	float l_wall = b_wall;

	if (current.center.x > r_wall) {
		current.center.x = r_wall;
		current.velocity.x *= -1;
	}
	else if (current.center.x < l_wall) {
		current.center.x = l_wall;
		current.velocity.x *= -1;
	}

	if (current.center.y > t_wall) {
		current.center.y = t_wall;
		current.velocity.y *= -1;
	}
	else if (current.center.y < b_wall) {
		current.center.y = b_wall;
		current.velocity.y *= -1;
	}
}

void bouncing_balls_sim::operator()(ball& current, int start, int end) {
	while (program_running) {
		while (!do_frame) 
			std::this_thread::sleep_for(std::chrono::milliseconds(1));

		wall_bounce(current);

		computation_barrier.sync();

		for (int i = start; i < end; ++i) {
			ball* current = pairs[i].first;
			ball* other = pairs[i].second;

			vector2d c = current->center - other->center;

			float min_dist = current->radius + other->radius;

			if (aabb(*current, *other) && powf(c.x, 2) + powf(c.y, 2) <= powf(min_dist, 2)) {
				float distance = vector2d::magnitude(c);
				float overlap = 0.5f * (distance - current->radius - other->radius);

				vector2d dir = vector2d::normalize(c);

				current->center += -overlap * dir;
				other->center += overlap * dir;

				vector2d v = current->velocity - other->velocity;
				int m = current->mass + other->mass;
				float mag = powf(vector2d::magnitude(c), 2);
				float dot_vc = vector2d::dot(v, c);
				float ratio = 2.f * dot_vc / (m * mag);

				current->velocity -= (other->mass * ratio * c);
				other->velocity -= (current->mass * ratio * -1 * c);
			}
		}

		computation_barrier.sync();
		do_frame = false;
	}
}

void bouncing_balls_sim::update() {
	previous_t = current_t;
	accumulated_t += delta_t;

	if (accumulated_t < UPDATE_FREQ) return;

	accumulated_t -= UPDATE_FREQ;

	if (current_t == 0) {
		current_t = clock();
		return;
	}

	do_frame = true;
	while (do_frame) 
		std::this_thread::sleep_for(std::chrono::milliseconds(1));

	draw();

	glutSwapBuffers();
	current_t = clock();
	delta_t = (float)(current_t - previous_t) / CLOCKS_PER_SEC;
}

void bouncing_balls_sim::start(void(*callback)()) {
	init();

	glutDisplayFunc(callback);
	glutIdleFunc(callback);
	glutMainLoop();

	// ensure worker threads are done computing and have been properly 
	// destroyed before exiting the program.
	program_running = false;
	for (std::thread& worker : t_workers) {
		worker.join();
	}
}

void bouncing_balls_sim::draw() {
	glClearColor(0.25f, 0.25f, 0.25f, 1.f);
	glClear(GL_COLOR_BUFFER_BIT);

	for (ball& ball : balls) {
		ball.velocity += GRAVITY;
		ball.center += delta_t  * ball.velocity;

		glBegin(GL_POLYGON);
		glColor4f(ball.color.x, ball.color.y, ball.color.z, 0.25f);

		for (int i = 0; i < NUM_POINTS; ++i) {
			float angle = i * DEGREE_TO_RAD;
			float x = ball.radius * cos(angle) + ball.center.x;
			float y = ball.radius * sin(angle) + ball.center.y;

			glVertex2d(x, y);
		}

		glEnd();
	}
}

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

		int weight = radius * 100;
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

bool bouncing_balls_sim::aabb(const ball& current, const ball& other) {
	// axis-aligned bounding box check (optimization)
	// if true, circles are close enough. compute further.

	float min_dist = current.radius + other.radius;

	return (current.center.x + min_dist > other.center.x
		&& current.center.y + min_dist > other.center.y
		&& other.center.x + min_dist > current.center.x
		&& other.center.y + min_dist > current.center.y);
}
