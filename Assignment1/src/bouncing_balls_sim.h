#pragma once
#include <vector>
#include <time.h>
#include <thread>
#include "ball.h"

class bouncing_balls_sim {
public:
	bouncing_balls_sim() = default;
	bouncing_balls_sim(int* argc, char **argv);
	void start(void(*callback)());
	void update();

private:
	void draw();
	void wall_collisions();
	void ball_collisions();
	bool aabb(const ball& current, const ball& other);

	std::vector<ball> balls;
	std::vector<std::pair<ball&, ball&>> collisions;
	std::vector<std::thread> t_workers;

	clock_t previous_t = 0, current_t = 0;
	float delta_t;

	vector2d GRAVITY;

	constexpr static float MIN_RADIUS = 0.05f;
	constexpr static float PI = 3.141592f;
	constexpr static float DEGREE_TO_RAD = PI / 180;
	constexpr static int NUM_POINTS = 360;
	//constexpr static float GRAVITY = 9.8f; // make vector
};