#include "barrier.h"

barrier::barrier(int num) : count(0), total(num) {}

barrier& barrier::operator=(const barrier& rhs) {
	if (&rhs != this) {
		std::lock(mutex, rhs.mutex);
		std::lock_guard<std::mutex> lhs_lock(mutex, std::adopt_lock);
		std::lock_guard<std::mutex> rhs_lock(rhs.mutex, std::adopt_lock);
		count = rhs.count;
		total = rhs.total;
	}
	return *this;
}

void barrier::sync() {
	std::unique_lock<std::mutex> lock(mutex);
	++count;

	if (count == total) {
		cv.notify_all();
		count = 0;
	}
	else {
		cv.wait(lock);
	}
}