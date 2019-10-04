#include "barrier.h"

barrier::barrier(int num) : count(0), total(num) {}

barrier& barrier::operator=(const barrier& barr) {
	if (&barr != this) {
		std::lock(mutex, barr.mutex);
		std::lock_guard<std::mutex> lhs_lock(mutex, std::adopt_lock);
		std::lock_guard<std::mutex> rhs_lock(barr.mutex, std::adopt_lock);
		count = barr.count;
		total = barr.total;
	}
	return *this;
}

void barrier::sync() {
	std::unique_lock<std::mutex> lock(mutex);
	count++;

	if (count == total) {
		cv.notify_all();
		count = 0;
	}
	else {
		cv.wait(lock);
	}
}