#pragma once
#include <cstdint>
#include <glm.hpp>
#define CLOTHSIM_API __declspec(dllexport)

extern "C" {
	CLOTHSIM_API void cpp_init(glm::vec3* positions,int* triangles, int num_positions,int num_triangles, float delta_time, int algorithm_type, int scenario, int solver_iterations,int* static_particles, int num_static_particles, int substeps, glm::vec3 sphere_centre, float sphere_radius);
	CLOTHSIM_API void cpp_update(glm::vec3* positions, float wind_strength, float stretching_stiffness, float shearing_stiffness, int selected_particle_index, glm::vec3 mouse_world_pos, int stop_grabbing_index);
}