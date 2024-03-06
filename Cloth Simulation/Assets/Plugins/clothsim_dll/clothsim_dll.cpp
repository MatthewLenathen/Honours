#include "clothsim_dll.h"
#include "clothsim.h"
#include <vector>
#include <glm.hpp>

ClothSim g_ClothSim;

extern "C" {
	CLOTHSIM_API void cpp_init(glm::vec3* positions,int* triangles, int num_positions, int num_triangles, float delta_time, int algorithm_type, int scenario, int solver_iterations, int* static_particles, int num_static_particles, int substeps)
	{
		g_ClothSim.Init(positions,triangles, num_positions,num_triangles, delta_time, algorithm_type,scenario, solver_iterations, static_particles, num_static_particles, substeps);
	}
	
	CLOTHSIM_API void cpp_update(glm::vec3* positions, float wind_strength, float stretching_stiffness, float shearing_stiffness, int selected_particle_index, glm::vec3 mouse_world_pos, int stop_grabbing_index)
	{
		g_ClothSim.Update(positions,wind_strength, stretching_stiffness,shearing_stiffness, selected_particle_index, mouse_world_pos, stop_grabbing_index);
	}
}