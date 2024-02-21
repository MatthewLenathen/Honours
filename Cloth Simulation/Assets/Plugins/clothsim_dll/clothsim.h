#pragma once
#include <fstream>
#include <iostream>
#include <glm.hpp>
#include <vector>
using namespace std;

const glm::vec3 GRAVITY = glm::vec3(0.0f,-9.81f,0.0f);

// Class representing a particle
class Particle
{
public:
	glm::vec3 position;
	glm::vec3 velocity = glm::vec3(0.0f);
	float mass;
	bool is_static;

	// Create particle with values, starting velocity always 0
	Particle(const glm::vec3& position, float mass, bool is_static) : position(position), mass(mass), is_static(is_static){}
};

// Class representing the simulation
class ClothSim
{
	int _algorithm_type; // 0 for mass-spring, 1 for PBD
	int _scenario; // 0 = hanging cloth, rest tbd
	vector<Particle> _particles;
	int _num_particles;
	vector<glm::uvec2> _structural_constraints;
	vector<glm::uvec2> _shear_constraints;
	vector<glm::uvec2> _bend_constraints;
	float _delta_time;
	float _total_time = 0.0f;
	float _spacing;
	
public:
	void Init(glm::vec3* positions, int num_positions, float delta_time, int grid_size, int algorithm_type, int scenario, float spacing);

	// Testing particle positions upon initialisation, making sure Vector3 converts to glm vec3 nicely
	void LogParticlePositions(const std::vector<Particle>& particles);

	void Update(glm::vec3* positions, glm::vec3 wind_force);
};