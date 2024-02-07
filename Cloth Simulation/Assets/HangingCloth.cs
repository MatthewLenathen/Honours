using System.Collections;
using System.Collections.Generic;
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
    public static Mesh generateMesh(int gridSize, float spacing)
    {
        Mesh mesh = new Mesh();

        // Create vertices
        Vector3[] vertices = new Vector3[(gridSize + 1) * (gridSize + 1)];
        for (int i = 0, y = 0; y <= gridSize; y++)
        {
            for (int x = 0; x <= gridSize; x++, i++)
            {
                vertices[i] = new Vector3(x * spacing, y * spacing, 0);
            }
        }

        // Create triangles
        int[] triangles = new int[gridSize * gridSize * 6];
        for (int ti = 0, vi = 0, y = 0; y < gridSize; y++, vi++)
        {
            for (int x = 0; x < gridSize; x++, ti += 6, vi++)
            {
                triangles[ti] = vi;
                triangles[ti + 3] = triangles[ti + 2] = vi + 1;
                triangles[ti + 4] = triangles[ti + 1] = vi + gridSize + 1;
                triangles[ti + 5] = vi + gridSize + 2;
            }
        }

        mesh.vertices = vertices;
        mesh.triangles = triangles;
        mesh.RecalculateNormals();
        return mesh;
    }
}

[System.Diagnostics.DebuggerDisplay("{" + nameof(GetDebuggerDisplay) + "(),nq}")]
public class HangingCloth : MonoBehaviour
{
    // Start is called before the first frame update
    List<Particle> particles = new List<Particle>();

    private Mesh mesh;
    private Vector3[] vertices;

    private int gridSize = 20;
    private float spacing = 0.5f;

    private Vector3 gravity = new Vector3(0.0f, -9.8f, 0.0f);
    private int springConstant = 1000;

    Vector3 windForce = new Vector3(0.0f, 0.0f, 1.0f); 
    float windStrength = 5.0f; 

    void Start()
    {
        MeshFilter meshFilter = GetComponent<MeshFilter>();
        meshFilter.mesh = MeshCreator.generateMesh(gridSize, spacing);

        mesh = meshFilter.mesh;
        vertices = mesh.vertices;

        // Create all particles and initialise them
        foreach (Vector3 vertex in vertices)
        {
            Particle p = new Particle(vertex, 1.0f, false);
            particles.Add(p);
        }

        // Fix the top row of particles
        // Since the mesh is created from the bottom left corner, the top row of vertices are the last ones in the list
        int startIndexOfTopRow = gridSize * (gridSize + 1);

        // Iterate over the top row vertices and set them to be static
        for (int i = startIndexOfTopRow; i < particles.Count; i++)
        {
            particles[i].isStatic = true;
        }

    }

    void ApplyConstraints()
    {
        for (int y = 0; y <= gridSize; y++)
        {
            for (int x = 0; x <= gridSize; x++)
            {
                int index = y * (gridSize + 1) + x;

                // Horizontal neighbour  
                if (x < gridSize)
                {
                    ApplySpringForce(particles[index], particles[index + 1], spacing, springConstant);
                }

                // Vertical neighbour
                if (y < gridSize)
                {
                    ApplySpringForce(particles[index], particles[index + gridSize + 1], spacing, springConstant);
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
            p1.velocity *= (1 - dampingCoefficient * Time.deltaTime);
        }
        // Working on second particle
        if (!p2.isStatic)
        {
            // negative force here because it's applied in the opposite direction
            Vector3 velocityChange = -force * Time.deltaTime;
            p2.velocity += velocityChange;
            // Same damping
            p2.velocity *= (1 - dampingCoefficient * Time.deltaTime);
        }
    }

    void Update()
    {
        windStrength = Mathf.Sin(Time.time) * 5.0f;
        // Simple Euler integration as a test
        for (int i = 0; i < particles.Count; i++)
        {
            if (!particles[i].isStatic)
            {
                particles[i].velocity += gravity * Time.deltaTime;
                particles[i].velocity += windForce * windStrength * Time.deltaTime;
                particles[i].position += particles[i].velocity * Time.deltaTime; ;
            }
            vertices[i] = particles[i].position;
        }

        ApplyConstraints();

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
