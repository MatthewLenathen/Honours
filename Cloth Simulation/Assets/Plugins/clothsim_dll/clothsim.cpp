#include "clothsim.h"
#include <random>

std::random_device rd;
std::mt19937 gen(rd());
std::uniform_real_distribution<> dis(-100.0, 100.0);

const int SPRING_CONSTANT = 1000;
// Log 
void logConstraints(vector<glm::uvec2> constraints, string filename)
{
	std::ofstream file(filename);

	if (!file)
	{
		std::cerr << "Failed to open file: " << filename << std::endl;
		return;
	}

	for (const glm::uvec2& constraint : constraints)
	{
		file << "Particle index " << constraint.x << ", connected to particle index " << constraint.y << "\n";
	}
}

void LogTriangleIndices(int* triangles, int triangleCount)
{
	std::ofstream logFile("triangles_log.txt");
	if (logFile.is_open()) {
		for (int i = 0; i < triangleCount; ++i) {
			logFile << "Triangle Indices: "
				<< triangles[i * 3] << ", "
				<< triangles[i * 3 + 1] << ", "
				<< triangles[i * 3 + 2] << std::endl;
		}
		logFile.close();
	}
	else {
		std::cerr << "Failed to open log file." << std::endl;
	}
}


void ApplySpringForce(Particle& p1, Particle& p2, float rest_length, float spring_constant, float delta_time)
{
	glm::vec3 delta = p2.position - p1.position;
	float distance = glm::length(delta);
	glm::vec3 force_direction = glm::normalize(delta);

	// Spring force magnitude
	float force_magnitude = spring_constant * (distance - rest_length);
	glm::vec3 force = force_magnitude * force_direction;

	// Damping coefficient, prevents oscillation
	float dampingCoefficient = 0.25f;

	// Working on first particle
	if (!p1.is_static)
	{
		glm::vec3 velocityChange = force * delta_time;
		p1.velocity += velocityChange;
		// Applying damping
		p1.velocity *= 1 - dampingCoefficient * delta_time;
	}
	// Working on second particle
	if (!p2.is_static)
	{
		// negative force here because it's applied in the opposite direction
		glm::vec3 velocityChange = -force * delta_time;
		p2.velocity += velocityChange;
		// Same damping
		p2.velocity *= 1 - dampingCoefficient * delta_time;
	}
}

// Called at start of sim
void ClothSim::Init(glm::vec3* positions,int* triangles, int num_positions,int num_triangles, float delta_time, int algorithm_type, int scenario, float spacing, int solver_iterations, int* static_particles, int num_static_particles)
{
	// Since the g_ClothSim object stays alive, must clear the particles when initialising again
	_particles.clear();
	_structural_constraints.clear();
	_static_particles.clear();
	_triangles.clear();

	// Initialise necessary variables used in update
	_spacing = spacing;
	_num_particles = num_positions;
	_num_triangles = num_triangles;
	_delta_time = delta_time;
	_algorithm_type = algorithm_type;
	_scenario = scenario;
	_solver_iterations = solver_iterations;
	_total_time = 0.0f;
	_num_static_particles = num_static_particles;
	
	// Create particles and add them to particles vector
	for (int i = 0; i < num_positions; i++)
	{
		_particles.push_back(Particle(positions[i], 1.0f, false)); // mass of 1, inverse mass is still 1
	}

	//LogTriangleIndices(triangles, num_triangles);
	
	// Create triangles from triangles pointer, *3 because triangles data comes in 3's, represents indices that make up a triangle, e.g. {0,20,1} 
	for (int i = 0; i < num_triangles*3; i++)
	{
		_triangles.push_back(triangles[i]);
	}

	// Static particles, set specific particle by using index from static_particles array
	for (int i = 0; i < num_static_particles; i++)
	{
		_particles[static_particles[i]].is_static = true;
		_particles[static_particles[i]].inverse_mass = 0;
	}

	// Now branch off depending on algorithm/scenario
	switch (algorithm_type)
	{
	case 0: // mass-spring 
	{
		switch (scenario)
		{
		case 0: // Hanging cloth
		{
			GenerateConstraints(_triangles, _num_triangles, _structural_constraints, _shear_constraints);
			
			break;
		}
		case 1: // tbd
			
			break;
		default:
			break;
		}
		break;
	}
	case 1: // PBD
		switch (scenario)
		{
		case 0: // Hanging cloth
		{
			GenerateConstraints(_triangles, _num_triangles, _structural_constraints, _shear_constraints);
			logConstraints(_structural_constraints, "struct_constraints_log.txt");
			logConstraints(_shear_constraints, "shear_constraints_log.txt");
			break;
		}
		default:
			break;
		}

	default:
		break;
	}


	//LogParticlePositions(particles);
}

// Called every frame
void ClothSim::Update(glm::vec3* positions, glm::vec3 wind_force)
{
	_total_time += _delta_time;
	switch (_algorithm_type)
	{
	case 0: // Mass spring
		for (int i = 0; i < _num_particles; i++)
		{
			if (!_particles[i].is_static)
			{
				_particles[i].velocity += GRAVITY * _delta_time;
				_particles[i].velocity += wind_force * std::sin(_total_time) * (float)dis(gen) * _delta_time;
				_particles[i].position += _particles[i].velocity * _delta_time;
				positions[i] = _particles[i].position; // important, update actual positions that gets passed back to unity
			}
		}

		// Go through all structural constraints to apply force, constraint holds indices of particles that have a constraint
		for (auto constraint : _structural_constraints)
		{
			ApplySpringForce(_particles[constraint.x], _particles[constraint.y], _spacing, SPRING_CONSTANT, _delta_time);
		}
		break;

	case 1: // pbd, going off the papers algorithm to start
	{
		// first, hook up external forces to the system, e.g. gravity and wind
		for (int i = 0; i < _num_particles; i++)
		{
			if (!_particles[i].is_static)
				_particles[i].velocity += _delta_time * _particles[i].inverse_mass * (GRAVITY + (wind_force * std::sin(_total_time)));
		}

		// Next, damp velocities
		for (int i = 0; i < _num_particles; i++)
		{
			if (!_particles[i].is_static)
				_particles[i].velocity *= 0.99;
		}

		// Calculate predicted position next
		for (int i = 0; i < _num_particles; i++)
		{
			if (!_particles[i].is_static)
				_particles[i].predicted_position = _particles[i].position + (_delta_time * _particles[i].velocity);
		}

		// Collision constraints to be generated here, doing it later as i want to get main pbd working first
		// forall vertices i do generateCollisionConstraints(xi -> pi)

		// Now, for a set number of solver iterations, work on the constraints to slightly move particles to fit them
		for (int i = 0; i < _solver_iterations; i++)
		{
			for (auto constraint : _structural_constraints)
			{
				Particle& p1 = _particles[constraint.x];
				Particle& p2 = _particles[constraint.y];

				float distance_between_particles = glm::length(p1.predicted_position - p2.predicted_position);

				if (distance_between_particles == 0)
					continue;

				if (p1.is_static && p2.is_static)
					continue; // Skip this constraint if both particles are static

				glm::vec3 direction = glm::normalize(p1.predicted_position - p2.predicted_position);

				// Modelling from example given in section 3.3, distance constraint, assuming d to be _spacing
				glm::vec3 p1_correction = (p1.inverse_mass / (p1.inverse_mass + p2.inverse_mass)) * (distance_between_particles - _spacing) * direction;
				glm::vec3 p2_correction = (p2.inverse_mass / (p1.inverse_mass + p2.inverse_mass)) * (distance_between_particles - _spacing) * direction;

				p1.predicted_position -= p1_correction;
				p2.predicted_position += p2_correction;
			}
		}

		// Finally, after solving, set new velocity and position 
		for (int i = 0; i < _num_particles; i++)
		{
			if (!_particles[i].is_static) {
				_particles[i].velocity = (_particles[i].predicted_position - _particles[i].position) / _delta_time;
				_particles[i].position = _particles[i].predicted_position;
				positions[i] = _particles[i].position; // important, update actual positions that gets passed back to unity
			}
		}
		break;
	}
	default:
		break;

	}
}

// Generates constraints and stores them in the vectors passed in
void ClothSim::GenerateConstraints(const vector<int>& triangles, int num_triangles, vector<glm::uvec2>& structural_constraints, vector<glm::uvec2>& shear_constraints)
{
	// Using sets for uniqueness, and pairs to utilise minmax
	// e.g. a constraint between 1 and 20 will be turned into a pair (1,20) and added to temp_constraints
	// so that when we get to the same diagonal edge of the adjacent triangle, (20,1), then minmax will turn it into (1,20) and it will not be added to the set because it already is in there
	// This will make sure the shared diagonal edge constraint isn't added created twice
	std::set<std::pair<int, int>> temp_structural_constraints;
	std::set<std::pair<int, int>> temp_shear_constraints;

	// Loop through num triangles, we get triangles by multiplying i by 3 each time
	for (int i = 0; i < num_triangles; i += 2)
	{
		// First triangle
		int i1 = triangles[i * 3];
		int i2 = triangles[i * 3 + 1];
		int i3 = triangles[i * 3 + 2];

		// First triangle constraints
		temp_structural_constraints.insert(std::minmax(i1, i2));
		temp_structural_constraints.insert(std::minmax(i1, i3));

		// Since second triangle is i+1, need to check if we can do that
		if ((i + 1) < num_triangles)
		{
			// Second triangle
			int i4 = triangles[(i + 1) * 3];
			int i5 = triangles[(i + 1) * 3 + 1];
			int i6 = triangles[(i + 1) * 3 + 2];

			// Second triangle constraints
			temp_structural_constraints.insert(std::minmax(i4, i6));
			temp_structural_constraints.insert(std::minmax(i5, i6));

			// Two shear constraints
			temp_shear_constraints.insert(std::minmax(i2, i3));
			temp_shear_constraints.insert(std::minmax(i1, i6));
		}
	}

	// Now turn the pair into glm uvec2 for use in update
	for (const auto& constraint : temp_structural_constraints)
	{
		structural_constraints.push_back(glm::uvec2(constraint.first, constraint.second));
	}

	for (const auto& constraint : temp_shear_constraints)
	{
		shear_constraints.push_back(glm::uvec2(constraint.first, constraint.second));
	}

}

// Used to check particles, cant print through dll without external debug program
void ClothSim::LogParticlePositions(const std::vector<Particle>& particles)
{
	std::ofstream logFile("particles_log.txt");
	if (logFile.is_open()) {
		for (const auto& particle : particles) {
			logFile << "Particle Position: "
				<< particle.position.x << ", "
				<< particle.position.y << ", "
				<< particle.position.z << ", fixed = "
				<< particle.is_static << std::endl;
		}
		logFile.close();
	}
	else {
		std::cerr << "Failed to open log file." << std::endl;
	}
}

