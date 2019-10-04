#pragma once
#include <vector>
#include <time.h>
#include <thread>
#include "ball.h"
#include "barrier.h"

class bouncing_balls_sim {
public:
	bouncing_balls_sim() = default;
	bouncing_balls_sim(int* argc, char **argv);
	void operator()(ball& current, int start, int end);
	void start(void(*callback)());
	void update();

private:
	void draw();
	void wall_collisions();
	void ball_collisions();
	bool aabb(const ball& current, const ball& other);

	// make mutex instead of do_frame

	barrier barrier_;
	std::vector<ball> balls;
	std::vector<std::pair<ball&, ball&>> pairs;
	std::vector<std::thread> t_workers;

	clock_t previous_t = 0;
	clock_t current_t = 0;
	float delta_t;
	bool compute;
	bool do_frame;

	vector2d GRAVITY;

	constexpr static float MIN_RADIUS = 0.05f;
	constexpr static float PI = 3.141592f;
	constexpr static float DEGREE_TO_RAD = PI / 180;
	constexpr static int NUM_POINTS = 360;
};