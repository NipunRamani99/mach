#ifndef __HELLO_HPP__
#define __HELLO_HPP__
#include <iostream>
class Hello {
public:
	Hello() {
		std::cout << "Initialized Hello Class \n";
	}
	~Hello() {}

	void hello() {
		std::cout << "Hello World!\n";
	}

	void hello2() {
		std::cout << "Hello World2!\n";
	}
};
#endif