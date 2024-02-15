#include "clothsim_dll.h"
#include "clothsim.h"
#include <vector>
#include <glm.hpp>

ClothSim g_ClothSim;

extern "C" {
	/*CLOTHSIM_API void cpp_init(glm::vec3* positions, int numPositions, float fixedTimeStep)
	{
		g_ClothSim.Init();
	}
	CLOTHSIM_API void cpp_update(glm::vec3* positions, int numPositions, glm::vec3 windForce)
	{
		g_ClothSim.Update();
	}*/

	// Used to test it works in unity
	CLOTHSIM_API int cpp_test(int a)
	{
		return g_ClothSim.Test(a);
	}
}