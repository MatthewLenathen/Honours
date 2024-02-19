#include "clothsim_dll.h"
#include "clothsim.h"
#include <vector>
#include <glm.hpp>

ClothSim g_ClothSim;

extern "C" {
	CLOTHSIM_API void cpp_init(glm::vec3* positions, int num_positions, float delta_time, int grid_size, int algorithm_type, int scenario)
	{
		g_ClothSim.Init(positions, num_positions, delta_time, grid_size, algorithm_type,scenario);
	}
	
	CLOTHSIM_API void cpp_update(glm::vec3* positions, glm::vec3 wind_force)
	{
		g_ClothSim.Update(positions,wind_force);
	}
}