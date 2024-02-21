#include "clothsim.h"
#include <random>

std::random_device rd;
std::mt19937 gen(rd());
std::uniform_real_distribution<> dis(-10.0, 10.0);

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
void ClothSim::Init(glm::vec3* positions, int num_positions, float delta_time, int grid_size, int algorithm_type, int scenario, float spacing, int solver_iterations)
{
	// Since the g_ClothSim object stays alive, must clear the particles when initialising again
	_particles.clear();
	_structural_constraints.clear();

	// Initialise necessary variables used in update
	_spacing = spacing;
	_num_particles = num_positions;
	_delta_time = delta_time;
	_algorithm_type = algorithm_type;
	_scenario = scenario;
	_solver_iterations = solver_iterations;

	// Create particles and add them to particles vector
	for (int i = 0; i < num_positions; i++)
	{
		_particles.push_back(Particle(positions[i], 1.0f, false)); // mass of 1, inverse mass is still 1
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
			// Create static particles
			int startIndexOfTopRow = grid_size * (grid_size - 1);

			// Iterate over the top row vertices and set them to be static
			for (int i = startIndexOfTopRow; i < _num_particles; i++)
			{
				_particles[i].is_static = true;
			}

			// Generate structural constraints
			for (int y = 0; y < grid_size; y++)
			{
				for (int x = 0; x < grid_size; x++)
				{
					unsigned int index = y * grid_size + x;

					// Horizontal spring
					if (x < grid_size - 1)
					{
						_structural_constraints.push_back(glm::uvec2(index, index + 1));
					}

					// Vertical spring
					if (y < grid_size - 1)
					{
						_structural_constraints.push_back(glm::uvec2(index, index + grid_size));
					}

				}
			}
			//logConstraints(_structural_constraints, "constraints_log.txt");



		}
		case 1: // tbd
			//...
			break;
		default:
			exit(EXIT_FAILURE);
		}
		break;
	}
	case 1: // PBD
		switch (scenario)
		{
		case 0: // Hanging cloth
			//...
			break;
		default:
			exit(EXIT_FAILURE);
		}

	default:
		exit(EXIT_FAILURE);
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
			}

			positions[i] = _particles[i].position;
		}

		// Go through all structural constraints to apply force, constraint holds indices of particles that have a constraint
		for (auto constraint : _structural_constraints)
		{
			ApplySpringForce(_particles[constraint.x], _particles[constraint.y], _spacing, SPRING_CONSTANT, _delta_time);
		}
		break;

	case 1: // pbd, going off the papers algorithm to start
		// first, hook up external forces to the system, e.g. gravity and wind
		for (int i = 0; i < _num_particles; i++)
		{
			if(!_particles[i].is_static)
				_particles[i].velocity += _delta_time * _particles[i].inverse_mass * (GRAVITY + wind_force);
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

		}
		
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

