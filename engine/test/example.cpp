#include <gtest/gtest.h>
#include "include/mach.hpp"
TEST(example, add) {
	int res = 0;
	Mach mach;
	res = mach.addNum(3, 4);
	ASSERT_EQ(res, 7);
}