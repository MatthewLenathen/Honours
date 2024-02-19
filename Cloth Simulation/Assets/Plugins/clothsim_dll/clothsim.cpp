#include "clothsim.h"
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

// Called at start of sim
void ClothSim::Init(glm::vec3* positions, int num_positions, float delta_time, int grid_size, int algorithm_type, int scenario)
{
	// Since the g_ClothSim object stays alive, must clear the particles when initialising again
	_particles.clear();
	_structural_constraints.clear();

	// Initialise necessary variables used in update
	_num_particles = num_positions;
	_delta_time = delta_time;
	_algorithm_type = algorithm_type;
	_scenario = scenario;

	// Create particles and add them to particles vector
	for (int i = 0; i < num_positions; i++)
	{
		_particles.push_back(Particle(positions[i], 1.0f, false));
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

	for (int i = 0; i < _num_particles; i++)
	{
		if (!_particles[i].is_static && _particles[i].position.y > -30.0f)
		{
			_particles[i].velocity += GRAVITY * _delta_time;
			_particles[i].velocity += wind_force * std::sin(_total_time) * 5.0f * _delta_time;
			_particles[i].position += _particles[i].velocity * _delta_time;
		}

		positions[i] = _particles[i].position;
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

