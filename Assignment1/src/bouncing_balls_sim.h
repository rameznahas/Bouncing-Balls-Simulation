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
	void init();
	void draw();
	bool aabb(const ball& current, const ball& other);
	void wall_bounce(ball& current);

	std::vector<ball> balls;
	std::vector<std::pair<ball*, ball*>> pairs;
	std::vector<std::thread> t_workers;

	vector2d GRAVITY;

	barrier wall_computation;
	barrier do_frame;
	barrier frame_done;

	clock_t previous_t;
	clock_t current_t;
	float delta_t;

	bool program_running;

	constexpr static float MIN_RADIUS = 0.05f;
	constexpr static float PI = 3.141592f;
	constexpr static float DEGREE_TO_RAD = PI / 180;
	constexpr static int NUM_POINTS = 360;
};
