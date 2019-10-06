#pragma once
#include <mutex>
#include <condition_variable>

class barrier {
public:
	barrier() = default;
	barrier(int num);
	barrier& operator=(const barrier& rhs);
	void sync();

private:
	mutable std::mutex mutex;
	std::condition_variable cv;
	int count;
	int total;
};