#pragma once
#include <cstdint>
#define CLOTHSIM_API __declspec(dllexport)

extern "C" {
	//CLOTHSIM_API void cpp_init();
	//CLOTHSIM_API void cpp_update();
	CLOTHSIM_API int cpp_test(int a);
}