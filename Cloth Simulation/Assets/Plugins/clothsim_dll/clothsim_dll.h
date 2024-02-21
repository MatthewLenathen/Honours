#pragma once
#include <cstdint>
#include <glm.hpp>
#define CLOTHSIM_API __declspec(dllexport)

extern "C" {
	CLOTHSIM_API void cpp_init(glm::vec3* positions, int num_positions, float delta_time, int grid_size, int algorithm_type, int scenario, float spacing, int solver_iterations);
	CLOTHSIM_API void cpp_update(glm::vec3* positions, glm::vec3 wind_force);
}