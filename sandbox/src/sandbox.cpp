#include <iostream>
#include <Engine/hello.hpp>
#include <Engine/mach.hpp>

int main(int* argc, char** argv) {
	Mach mach;
	Hello hello;
	hello.hello();
	hello.hello2();
	std::cout << "Test\n";
	std::cout << "Adding 3 and 5: " << mach.addNum(3, 5);
	return 0;
}