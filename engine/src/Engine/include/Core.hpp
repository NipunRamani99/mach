#ifndef __CORE_HPP__
#define __CORE_HPP__
#include <functional>
void foreach_ij(const std::function<void(int, int)>& task, size_t width, size_t height) {
	for (size_t j = 0; j < height; j++) {
		for (size_t i = 0; i < width; i++) {
			task(i, j);
		}
	}
}

void foreach_ij(const std::function<void(int, int)>& task, size_t x_begin, size_t x_end, size_t y_begin, size_t y_end) {
	for (size_t j = y_begin; j < y_end; j++) {
		for (size_t i = x_begin; i < x_end; i++) {
			task(i, j);
		}
	}
}

void foreach_i(const std::function<void(int)>& task, size_t size) {
	for (size_t i = 0; i < size; i++) {
		task(i);
	}
}
#endif // !__CORE_HPP__
