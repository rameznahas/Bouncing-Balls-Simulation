#pragma once
#include <shared_mutex>

class rw_mutex {
public:
	rw_mutex() : r_count(0) {}
	rw_mutex& operator=(const rw_mutex& rw_mtx) {
		if (&rw_mtx != this) {

		}
	}

	void r_lock() {
		r_mutex.lock();
		r_count++;

		if (r_count == 1)
			w_mutex.lock_shared();

		r_mutex.unlock();
	}

	void r_unlock() {
		r_mutex.lock();
		r_count--;

		if (r_count == 0)
			w_mutex.unlock_shared();

		r_mutex.unlock();
	}

	void w_lock() {
		w_mutex.lock();
	}

	void w_unlock() {
		w_mutex.unlock();
	}

private:
	int r_count;
	std::shared_mutex r_mutex;
	std::shared_mutex w_mutex;
};