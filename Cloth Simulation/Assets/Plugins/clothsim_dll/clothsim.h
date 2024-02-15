#pragma once
#include <vector>
using namespace std;

// Class representing a particle
class Particle
{
	glm::vec3 position;
	glm::vec3 velocity = glm::vec3(0.0f);
	float mass;
	bool is_static;

	// Create particle with values, starting velocity always 0
	Particle(glm::vec3 position, float mass, bool is_static) : position(position), mass(mass), is_static(is_static){}
};

// Class representing the simulation
class ClothSim
{
public:
	int algorithm_type; // 0 for mass-spring, 1 for PBD
	vector<Particle> particles;
	vector<glm::ivec2> structural_constraints;
	vector<glm::ivec2> shear_constraints;
	vector<glm::ivec2> bend_constraints;
	static float delta_time;


	void Init(glm::vec3* positions, int numPositions, float fixedTimeStep, )
	{

	}

	/*void Update()
	{

	}*/

	// Used to test dll works in unity
	int Test(int a)
	{
		return a * 2;
	}
};