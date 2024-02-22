using System.Collections;
using System.Collections.Generic;
using System.Runtime.InteropServices;
using UnityEngine;

// Represents a particle in the cloth
public class Particle
{
    public Vector3 position;
    public Vector3 velocity;
    public float mass;
    public bool isStatic;

    // Constructor
    public Particle(Vector3 position, float mass, bool isStatic)
    {
        this.position = position;
        velocity = Vector3.zero;
        this.mass = mass;
        this.isStatic = isStatic;
    }
}

// Class to create a cloth mesh based on a grid size
public class MeshCreator
{
    public static Mesh generateVerticalMesh(int gridSize, float spacing)
    {
        Mesh mesh = new Mesh();

        // Create vertices
        Vector3[] vertices = new Vector3[gridSize * gridSize];
        for (int i = 0, y = 0; y < gridSize; y++)
        {
            for (int x = 0; x < gridSize; x++, i++)
            {
                vertices[i] = new Vector3(x * spacing, y * spacing, 0);
            }
        }

        // Create triangles
        int[] triangles = new int[gridSize * gridSize * 6];
        for (int ti = 0, vi = 0, y = 0; y < gridSize-1; y++, vi++)
        {
            for (int x = 0; x < gridSize-1; x++, ti += 6, vi++)
            {
                triangles[ti] = vi;
                triangles[ti + 3] = triangles[ti + 2] = vi + 1;
                triangles[ti + 4] = triangles[ti + 1] = vi + gridSize;
                triangles[ti + 5] = vi + gridSize + 1;
            }
        }

        mesh.vertices = vertices;
        mesh.triangles = triangles;
        mesh.RecalculateNormals();
        return mesh;
    }
}

public class cppFunctions
{
    [DllImport("clothsim_dll", EntryPoint = "cpp_init")]
    public static extern void cpp_init([In] Vector3[] vertices, int numParticles, float fixedDeltaTime, int gridSize, int algorithmType, int scenario, float spacing, int solverIterations);

    [DllImport("clothsim_dll", EntryPoint = "cpp_update")]
    public static extern void cpp_update([Out] Vector3[] vertices,[In] Vector3 WindForce);
}

[System.Diagnostics.DebuggerDisplay("{" + nameof(GetDebuggerDisplay) + "(),nq}")]
public class HangingCloth : MonoBehaviour
{
    bool CSHARP_SIM = false;

    // Start is called before the first frame update
    List<Particle> particles = new List<Particle>();

    private Mesh mesh;
    private Vector3[] vertices;

    private int gridSize = 20;
    private int numParticles = 20 * 20;
    private float spacing = 0.5f;
    private int algorithmType = 1; // 0 for mass spring, 1 for position based
    private int scenario = 0; // 0 for hanging cloth, 1 for ..
    private int solverIterations = 20;

    private Vector3 gravity = new Vector3(0.0f, -9.8f, 0.0f);
    private int springConstant = 10000;

    Vector3 windForce = new Vector3(0.0f, 0.0f, 1.0f); 
    float windStrength = 3.0f; 

    void Start()
    {
        MeshFilter meshFilter = GetComponent<MeshFilter>();
        meshFilter.mesh = MeshCreator.generateVerticalMesh(gridSize, spacing);

        mesh = meshFilter.mesh;
        vertices = mesh.vertices;

        if (CSHARP_SIM){
            // Create all particles and initialise them
            foreach (Vector3 vertex in vertices)
            {
                Particle p = new Particle(vertex, 1.0f, false);
                particles.Add(p);
            }

            // Fix the top row of particles
            // Since the mesh is created from the bottom left corner, the top row of vertices are the last ones in the list
            int startIndexOfTopRow = gridSize * (gridSize - 1);

            // Iterate over the top row vertices and set them to be static
            for (int i = startIndexOfTopRow; i < particles.Count; i++)
            {
                particles[i].isStatic = true;
            }

        }
        else{
		    cppFunctions.cpp_init(vertices, numParticles, Time.fixedDeltaTime,gridSize,algorithmType, scenario,spacing, solverIterations);
        }

       
    }

    void ApplyConstraints()
    {
        for (int y = 0; y < gridSize; y++)
        {
            for (int x = 0; x < gridSize; x++)
            {
                int index = y * gridSize + x;

                // Horizontal neighbour  
                if (x < gridSize - 1)
                {
                    ApplySpringForce(particles[index], particles[index + 1], spacing, springConstant);
                }

                // Vertical neighbour
                if (y < gridSize - 1)
                {
                    ApplySpringForce(particles[index], particles[index + gridSize], spacing, springConstant);
                }
            }
        }
    }

    void ApplySpringForce(Particle p1, Particle p2, float restLength, float springConstant)
    {
        Vector3 delta = p2.position - p1.position;
        float distance = delta.magnitude;
        Vector3 forceDirection = delta.normalized;

        // Spring force magnitude
        float forceMagnitude = springConstant * (distance - restLength);
        Vector3 force = forceMagnitude * forceDirection;

        // Damping coefficient, prevents oscillation
        float dampingCoefficient = 0.25f; 

        // Working on first particle
        if (!p1.isStatic)
        {
            Vector3 velocityChange = force * Time.deltaTime;
            p1.velocity += velocityChange;
            // Applying damping
            p1.velocity *= 1 - dampingCoefficient * Time.deltaTime;
        }
        // Working on second particle
        if (!p2.isStatic)
        {
            // negative force here because it's applied in the opposite direction
            Vector3 velocityChange = -force * Time.deltaTime;
            p2.velocity += velocityChange;
            // Same damping
            p2.velocity *= 1 - dampingCoefficient * Time.deltaTime;
        }
    }

    void FixedUpdate()
    {
        if (CSHARP_SIM)
        {
            //windStrength = Mathf.Sin(Time.time) * 3.0f;
            windStrength = Random.Range(-10.0f, 10.0f);
            // Simple Euler integration as a test
            for (int i = 0; i < particles.Count; i++)
            {
                if (!particles[i].isStatic)
                {
                    particles[i].velocity += gravity * Time.deltaTime;
                    particles[i].velocity += windForce * windStrength * Time.deltaTime;
                    particles[i].position += particles[i].velocity * Time.deltaTime;
                }
                vertices[i] = particles[i].position;
            }

            ApplyConstraints();
        }
        else{
            cppFunctions.cpp_update(vertices, windForce);
        }

        mesh.vertices = vertices;
        mesh.RecalculateNormals();
        mesh.RecalculateBounds();
        // Debug to check static particles
        /*  foreach (var particle in particles) {
            if (particle.isStatic) {
                Debug.Log("Static particle at: " + particle.position);
            }   
        } 
        */
    }

    private string GetDebuggerDisplay()
    {
        return ToString();
    }

}
