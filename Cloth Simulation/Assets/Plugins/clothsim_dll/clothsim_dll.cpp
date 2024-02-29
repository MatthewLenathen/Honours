#include "clothsim_dll.h"
#include "clothsim.h"
#include <vector>
#include <glm.hpp>

ClothSim g_ClothSim;

extern "C" {
	CLOTHSIM_API void cpp_init(glm::vec3* positions,int* triangles, int num_positions, int num_triangles, float delta_time, int algorithm_type, int scenario, float spacing, int solver_iterations, int* static_particles, int num_static_particles)
	{
		g_ClothSim.Init(positions,triangles, num_positions,num_triangles, delta_time, algorithm_type,scenario, spacing, solver_iterations, static_particles, num_static_particles);
	}
	
	CLOTHSIM_API void cpp_update(glm::vec3* positions, float wind_strength, float stretching_stiffness, float shearing_stiffness)
	{
		g_ClothSim.Update(positions,wind_strength, stretching_stiffness,shearing_stiffness);
	}
}