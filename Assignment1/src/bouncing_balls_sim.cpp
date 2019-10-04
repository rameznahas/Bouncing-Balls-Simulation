#include <random>
#include <iostream>
#include "glew.h"
#include "freeglut.h"
#include "bouncing_balls_sim.h"
#define WWIDTH 800
#define WHEIGHT 800

bouncing_balls_sim::bouncing_balls_sim(int* argc, char **argv)
	:
	balls(250/*std::stoi(argv[1])*/),
	t_workers(250/*std::stoi(argv[1])*/),
	barrier_(250/*std::stoi(argv[1])*/),
	GRAVITY(0.f, -0.015f),
	compute(true),
	do_frame(false)
{
	std::random_device rd;
	std::mt19937 gen(rd());
	std::uniform_int_distribution<int> rad(1, 3);
	std::uniform_real_distribution<float> vel(-1.f, 1.f);

	for (int i = 0; i < balls.size(); ++i) {
		float radius = MIN_RADIUS * rad(gen); // random radius

		float ur_bound = radius - 1;
		float ll_bound = 1 - radius;

		std::uniform_real_distribution<float> coord(ur_bound, ll_bound); // so we dont get balls out of bounds
		vector2d center(coord(gen), coord(gen));

		int weight = radius * 100;
		vector2d velocity(vel(gen), vel(gen));
		balls[i] = ball(radius, center, velocity, weight);
	}

	for (int i = 0; i < balls.size(); ++i) {
		for (int j = (i + 1); j < balls.size(); ++j) {
			pairs.push_back({ balls[i], balls[j] });
		}
	}

	glutInit(argc, argv);
	glutInitWindowPosition(-1, -1);
	glutInitWindowSize(WWIDTH, WHEIGHT);
	glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE | GLUT_ALPHA);
	glutCreateWindow("Bouncing Balls Simulation");
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	glEnable(GL_BLEND);
}

void bouncing_balls_sim::operator()(ball& current, int start, int end) {
	while (compute) {
		while (!do_frame) {
			std::this_thread::sleep_for(std::chrono::milliseconds(1));
		}
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

		barrier_.sync();

		for (int i = start; i < end; ++i) {
			ball& current = pairs[i].first;
			ball& other = pairs[i].second;

			vector2d c = current.center - other.center;

			float min_dist = current.radius + other.radius;

			if (aabb(current, other) && powf(c.x, 2) + powf(c.y, 2) <= powf(min_dist, 2)) {
				float distance = vector2d::magnitude(c);
				float overlap = 0.5f * (distance - current.radius - other.radius);

				vector2d dir = vector2d::normalize(c);

				current.center += -overlap * dir;
				other.center += overlap * dir;

				vector2d v = current.velocity - other.velocity;
				int m = current.mass + other.mass;
				float mag = powf(vector2d::magnitude(c), 2);
				float dot_vc = vector2d::dot(v, c);
				float ratio = 2.f * dot_vc / (m * mag);

				current.velocity -= (other.mass * ratio * c);
				other.velocity -= (current.mass * ratio * -1 * c);
			}
		}
		
		barrier_.sync();
		do_frame = false;
	}
}

void bouncing_balls_sim::start(void(*callback)()) {
	int div = pairs.size() / t_workers.size();
	int mod = pairs.size() % t_workers.size();

	int start = 0;
	int end = mod;
	for (int i = 0; i < t_workers.size(); ++i) {
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

	glutDisplayFunc(callback);
	glutIdleFunc(callback);
	glutMainLoop();

	do_frame = true;
	compute = false;
	for (std::thread& worker : t_workers) {
		worker.join();
	}
}

void bouncing_balls_sim::update() {
	previous_t = current_t;
	if (current_t == 0) {
		current_t = clock();
		return;
	}
	current_t = clock();
	delta_t = (float)(current_t - previous_t) / CLOCKS_PER_SEC;

	glClearColor(0.25f, 0.25f, 0.25f, 1.f);
	glClear(GL_COLOR_BUFFER_BIT);
	// signal worker threads
	do_frame = true;
	
	// wait until worker threads have finished preparing the frame
	while (do_frame) { std::this_thread::sleep_for(std::chrono::milliseconds(1)); }
	draw();

	glutSwapBuffers();
}

void bouncing_balls_sim::draw() {
	for (ball& ball : balls) {
		ball.velocity += GRAVITY;
		ball.center += delta_t * ball.velocity;

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

void bouncing_balls_sim::wall_collisions() {
	for (ball& ball : balls) {
		float t_wall = 1 - ball.radius;
		float b_wall = ball.radius - 1;
		float r_wall = t_wall;
		float l_wall = b_wall;

		if (ball.center.x > r_wall) {
			ball.center.x = r_wall;
			ball.velocity.x *= -1;
		}
		else if (ball.center.x < l_wall) {
			ball.center.x = l_wall;
			ball.velocity.x *= -1;
		}

		if (ball.center.y > t_wall) {
			ball.center.y = t_wall;
			ball.velocity.y *= -1;
		}
		else if (ball.center.y < b_wall) {
			ball.center.y = b_wall;
			ball.velocity.y *= -1;
		}
	}
}

void bouncing_balls_sim::ball_collisions() {
	for (std::pair<ball&, ball&>& pair : pairs) {
		ball& current = pair.first;
		ball& other = pair.second;

		vector2d c = current.center - other.center;

		float min_dist = current.radius + other.radius;

		if (aabb(current, other) && powf(c.x, 2) + powf(c.y, 2) <= powf(min_dist, 2)) {
			float distance = vector2d::magnitude(c);
			float overlap = 0.5f * (distance - current.radius - other.radius);

			vector2d dir = vector2d::normalize(c);

			current.center += -overlap * dir;
			other.center += overlap * dir;

			vector2d v = current.velocity - other.velocity;
			int m = current.mass + other.mass;
			float mag = powf(vector2d::magnitude(c), 2);
			float dot_vc = vector2d::dot(v, c);
			float ratio = 2.f * dot_vc / (m * mag);

			current.velocity -= (other.mass * ratio * c);
			other.velocity -= (current.mass * ratio * -1 * c);
		}
	}
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