using System.Collections;
using System.IO;
using System.Collections.Generic;
using System.Runtime.InteropServices;
using System.Runtime.InteropServices.WindowsRuntime;
using UnityEngine;
using UnityEditor.Recorder;
using UnityEditor.Recorder.Encoder;
using UnityEditor.Recorder.Input;

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
        // Changed this, because it was generating too many triangles, now it's generating the correct amount
        int[] triangles = new int[(gridSize - 1) * (gridSize - 1) * 6];
        for (int ti = 0, vi = 0, y = 0; y < gridSize - 1; y++)
        {
            for (int x = 0; x < gridSize - 1; x++, ti += 6, vi++)
            {
                triangles[ti] = vi;
                triangles[ti + 3] = triangles[ti + 2] = vi + 1;
                triangles[ti + 4] = triangles[ti + 1] = vi + gridSize;
                triangles[ti + 5] = vi + gridSize + 1;
            }
            vi++; 
        }

        mesh.vertices = vertices;
        mesh.triangles = triangles;
        mesh.RecalculateNormals();
        return mesh;
    }

    public static Mesh generateHorizontalMesh(int gridSize, float spacing)
    {
        Mesh mesh = new Mesh();

        // Create vertices
        Vector3[] vertices = new Vector3[gridSize * gridSize];
        for (int i = 0, y = 0; y < gridSize; y++)
        {
            for (int x = 0; x < gridSize; x++, i++)
            {
                vertices[i] = new Vector3(x * spacing, 0, y * spacing);
            }
        }

        // Create triangles
        // Changed this, because it was generating too many triangles, now it's generating the correct amount
        int[] triangles = new int[(gridSize - 1) * (gridSize - 1) * 6];
        for (int ti = 0, vi = 0, y = 0; y < gridSize - 1; y++)
        {
            for (int x = 0; x < gridSize - 1; x++, ti += 6, vi++)
            {
                triangles[ti] = vi;
                triangles[ti + 3] = triangles[ti + 2] = vi + 1;
                triangles[ti + 4] = triangles[ti + 1] = vi + gridSize;
                triangles[ti + 5] = vi + gridSize + 1;
            }
            vi++; 
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
    public static extern void cpp_init([In] Vector3[] vertices,[In] int[] triangles, int numParticles, int numTriangles, float fixedDeltaTime, int algorithmType, int scenario, int solverIterations, [In] int[] staticParticleIndices, int numStaticParticles, int subSteps, Vector3 sphereCenter, float sphereRadius);

    [DllImport("clothsim_dll", EntryPoint = "cpp_update")]
    public static extern void cpp_update([Out] Vector3[] vertices,[In] float windStrength, [In] float stretchingStiffness, [In] float shearingStiffness, [In] int selectedParticleIndex, [In] Vector3 mouseWorldPos, [In] int stopGrabbingIndex);
}

[System.Diagnostics.DebuggerDisplay("{" + nameof(GetDebuggerDisplay) + "(),nq}")]
public class HangingCloth : MonoBehaviour
{
    bool CSHARP_SIM = false;

    // Start is called before the first frame update
    List<Particle> particles = new List<Particle>();

    public Mesh mesh;
    public MeshFilter meshFilter;
    Vector3[] vertices;

    BoxCollider boxCollider;

    int gridSize = 20;
    int numParticles; // Changed to vertices.Length instead of hardcoded value
    int numTriangles; // Need for the C++ code
    float spacing = 0.5f; // Only used for mesh generation, not in c++ anymore
    int algorithmType = 2; // 0 for mass spring, 1 for position based, 2 for XPBD
    int scenario = 0; // 0 for hanging cloth, 1 for ..
    int solverIterations = 30; // Number of iterations for the pbd solver
    int subSteps = 10; // Number of substeps for XPBD

    int[] triangles;
    int[] staticParticleIndices;
    int numStaticParticles;

    Vector3 gravity = new Vector3(0.0f, -9.8f, 0.0f);
    int springConstant = 1000;
    Vector3 windForce = new Vector3(0.0f, 0.0f, 1.0f);

    bool isDragging = false;
    int selectedParticleIndex = -1;
    Vector3 mouseWorldPos;
    int stopGrabbingIndex = -1;
    float distanceToCloth = 0.0f;

    Vector3 sphereCentre;
    float sphereRadius;

    // Public stuff to change in the editor
    [Range(0f, 1f)]
    public float stretchingStiffness = 1.0f;
    [Range(0f, 1f)]
    public float shearingStiffness = 1.0f;
    [Range(0f, 0.001f)]
    public float stretchingCompliance = 0.001f;
    [Range(0f, 0.001f)]
    public float shearingCompliance = 0.001f;
    [Range(0f, 100f)]
    public float windStrength = 1.0f;

    // Recorder Variables
    bool isRecording = false;
    private List<RecordParameters> pendingExperiments = new List<RecordParameters>();
    RecorderController m_RecorderController;
    RecorderControllerSettings controllerSettings;
    MovieRecorderSettings videoRecorderSettings;
    int fixedUpdateCounter = 0;
    int maxFixedUpdates = 250;

    public class RecordParameters
    {
        public int algorithm; // 0 = M-S, 1 = PBD, 2 = XPBD
        public int scenario; // 0 = hanging cloth, 1 = cloth fall on sphere

        public float pbdStretchingStiffness;
        public float pbdShearingStiffness;
        public int pbdSolverIterations;

        public float xpbdStretchingCompliance; 
        public float xpbdShearingCompliance;
        public int xpbdSubsteps;

        public string filename; 
        //public CameraSettings cameraSettings; // prefab for camera transform, or however you want to set it up
    }

    // Generate a string for recording filenames
    public string GenerateFilename(RecordParameters rp)
    {
        string filename = "";

        switch (rp.algorithm)
        {
            case 0:
                filename += "MS";
                break;
            case 1:
                filename += "PBD";
                break;
            case 2:
                filename += "XPBD";
                break;
        }

        switch (rp.scenario)
        {
            case 0:
                filename += "_HangingCloth";
                break;
            case 1:
                filename += "_ClothFallOnSphere";
                break;
        }

        // can use string interpolation to add the variables in there
        if(rp.algorithm==0)
        {
            filename += $"_SpringConstant_{springConstant}";
        }
        else if (rp.algorithm == 1) // PBD
        {
            filename += $"_StretchingStiffness_{rp.pbdStretchingStiffness}_ShearingStiffness_{rp.pbdShearingStiffness}_SolverIterations_{rp.pbdSolverIterations}";
        }
        else if (rp.algorithm == 2) // XPBD
        {
            filename += $"_StretchingCompliance_{rp.xpbdStretchingCompliance}_ShearingCompliance_{rp.xpbdShearingCompliance}_SubSteps_{rp.xpbdSubsteps}";
        }

        return filename;
    }

    void StartRecording(RecordParameters recordParameters)
    {
        // First, get the type of cloth
        if(recordParameters.scenario==0)
        {
            meshFilter = GetComponent<MeshFilter>();
            meshFilter.sharedMesh = MeshCreator.generateVerticalMesh(gridSize, spacing);

            mesh = meshFilter.mesh;
            vertices = mesh.vertices;
            triangles = mesh.triangles;

            boxCollider = GetComponent<BoxCollider>();
            boxCollider.center = mesh.bounds.center;
            boxCollider.size = mesh.bounds.size;

            numParticles = vertices.Length;
            numTriangles = triangles.Length / 3;

            // Assign static particles to be able to pass them to the C++ code
            staticParticleIndices = new int[2]; // Change this number for more/less static particles

            staticParticleIndices[0] = gridSize * (gridSize -1);
            staticParticleIndices[1] = gridSize * gridSize -1;

            numStaticParticles = staticParticleIndices.Length;

            sphereCentre = new Vector3(999.0f, 999.0f, 999.0f);
            sphereRadius = 0.0f;
        }
        else if (recordParameters.scenario == 1)
        {
            meshFilter = GetComponent<MeshFilter>();
            meshFilter.sharedMesh = MeshCreator.generateHorizontalMesh(gridSize, spacing);

            mesh = meshFilter.mesh;
            vertices = mesh.vertices;
            triangles = mesh.triangles;

            boxCollider = GetComponent<BoxCollider>();
            boxCollider.center = mesh.bounds.center;
            boxCollider.size = mesh.bounds.size;

            numParticles = vertices.Length;
            numTriangles = triangles.Length / 3;

            numStaticParticles = 0;

            //MeshFilter sphereMeshFilter = GameObject.Find("Sphere").GetComponent<MeshFilter>();
            sphereCentre = GameObject.Find("Sphere").transform.position;
            sphereRadius = 2.5f;
        }

        // Now setting the variables depending on the experiment
        scenario = recordParameters.scenario;
        algorithmType = recordParameters.algorithm;
        if (algorithmType == 1)
        {
            stretchingStiffness = recordParameters.pbdStretchingStiffness;
            shearingStiffness = recordParameters.pbdShearingStiffness;
            solverIterations = recordParameters.pbdSolverIterations;
        }
        if (algorithmType == 2)
        {
            stretchingCompliance = recordParameters.xpbdStretchingCompliance;
            shearingCompliance = recordParameters.xpbdShearingCompliance;
            subSteps = recordParameters.xpbdSubsteps;
        }

        cppFunctions.cpp_init(vertices, triangles, numParticles, numTriangles, Time.fixedDeltaTime, algorithmType, scenario, solverIterations, staticParticleIndices, numStaticParticles, subSteps, sphereCentre, sphereRadius);

        // Recorder Init
        var mediaOutputFolder = Path.Combine(Application.dataPath, "..", "Simulation_Recordings");

        // Set output file path to filename that is generated for each recording
        videoRecorderSettings.OutputFile = Path.Combine(mediaOutputFolder, recordParameters.filename);

        // Setup Recording
        controllerSettings.AddRecorderSettings(videoRecorderSettings);
        controllerSettings.SetRecordModeToManual();
        controllerSettings.FrameRate = 60.0f;

        RecorderOptions.VerboseMode = false;
        m_RecorderController.PrepareRecording();
        m_RecorderController.StartRecording();
        // After starting the recording, set isRecording to true
        isRecording = true;
    }
    
    // For the GUI buttons
    public void OnButtonClick()
    {
        /*
        for (int i = 0; i < 3; i++)
        {
            var rp = new RecordParameters
            {
                algorithm = i, // XPBD
                scenario = 0, // HangingCloth
                xpbdStretchingCompliance = 0.001f, 
                xpbdShearingCompliance = 0.001f, 
                xpbdSubsteps = 5,
                pbdShearingStiffness = 0.5f,
                pbdStretchingStiffness = 0.5f,
                pbdSolverIterations = 20
            };
            rp.filename = $"{GenerateFilename(rp)}";

            // Add the recording settings to the pendingExperiments list
            pendingExperiments.Add(rp);
        }
        */
        for (int i = 0; i < 3; i++)
        {
            int subs = 5 + i*10;
            var rp = new RecordParameters
            {
                algorithm = 2, // XPBD
                scenario = 1, // HangingCloth
                xpbdStretchingCompliance = 0.001f, 
                xpbdShearingCompliance = 0.001f, 
                xpbdSubsteps = 5,
                pbdShearingStiffness = 0.5f,
                pbdStretchingStiffness = 0.5f,
                pbdSolverIterations = 20
            };
            rp.xpbdSubsteps = subs;
            rp.filename = $"{GenerateFilename(rp)}";

            // Add the recording settings to the pendingExperiments list
            pendingExperiments.Add(rp);
        }
        
        //Debug.Log(GenerateFilename());
    }

    void Start()
    {
        // Init Recorder settings
        controllerSettings = ScriptableObject.CreateInstance<RecorderControllerSettings>();
        m_RecorderController = new RecorderController(controllerSettings);

        videoRecorderSettings = ScriptableObject.CreateInstance<MovieRecorderSettings>();
        videoRecorderSettings.name = "Recorder";
        videoRecorderSettings.Enabled = true;

        videoRecorderSettings.EncoderSettings = new CoreEncoderSettings
        {
            EncodingQuality = CoreEncoderSettings.VideoEncodingQuality.Medium,
            Codec = CoreEncoderSettings.OutputCodec.MP4
        };

        videoRecorderSettings.CaptureAudio = false;

        videoRecorderSettings.ImageInputSettings = new GameViewInputSettings
        {
            OutputWidth = 1920,
            OutputHeight = 1080
        };


        if (scenario == 0) // Hanging cloth
        {
            meshFilter = GetComponent<MeshFilter>();
            meshFilter.sharedMesh = MeshCreator.generateVerticalMesh(gridSize, spacing);

            mesh = meshFilter.mesh;
            vertices = mesh.vertices;
            triangles = mesh.triangles;

            boxCollider = GetComponent<BoxCollider>();
            boxCollider.center = mesh.bounds.center;
            boxCollider.size = mesh.bounds.size;

            numParticles = vertices.Length;
            numTriangles = triangles.Length / 3;

            // Assign static particles to be able to pass them to the C++ code
            staticParticleIndices = new int[2]; // Change this number for more/less static particles

            staticParticleIndices[0] = gridSize * (gridSize -1);
            staticParticleIndices[1] = gridSize * gridSize -1;

            // for(int i = 0;i<20;i++)
            // {
            //     staticParticleIndices[i] = gridSize * (gridSize -1) + i;
            // }

            numStaticParticles = staticParticleIndices.Length;

            sphereCentre = new Vector3(999.0f, 999.0f, 999.0f);
            sphereRadius = 0.0f;
        } 
        else if (scenario == 1){ // Falling horizontal cloth
            meshFilter = GetComponent<MeshFilter>();
            meshFilter.sharedMesh = MeshCreator.generateHorizontalMesh(gridSize, spacing);

            mesh = meshFilter.mesh;
            vertices = mesh.vertices;
            triangles = mesh.triangles;

            boxCollider = GetComponent<BoxCollider>();
            boxCollider.center = mesh.bounds.center;
            boxCollider.size = mesh.bounds.size;

            numParticles = vertices.Length;
            numTriangles = triangles.Length / 3;

            numStaticParticles = 0;

            //MeshFilter sphereMeshFilter = GameObject.Find("Sphere").GetComponent<MeshFilter>();
            sphereCentre = GameObject.Find("Sphere").transform.position;
            sphereRadius = 2.5f;
        }


        if (CSHARP_SIM){
            // Create all particles and initialise them
            foreach (Vector3 vertex in vertices)
            {
                Particle p = new Particle(vertex, 1.0f, false);
                particles.Add(p);
            }

            // Set static particles
            for (int i = 0; i < numStaticParticles; i++)
            {
                particles[staticParticleIndices[i]].isStatic = true;
            }

        }
        else{
            cppFunctions.cpp_init(vertices, triangles, numParticles,numTriangles, Time.fixedDeltaTime,algorithmType, scenario, solverIterations, staticParticleIndices, numStaticParticles, subSteps,sphereCentre, sphereRadius);
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
        float dampingCoefficient = 0.5f; 

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

    // Will return the index to the nearest particle to the hit position by calculating distance to all particles
    int FindClosestParticleToMouse(Vector3 hitPosition)
    {
        float minDistance = float.MaxValue;
        int closestParticleIndex = -1; // Set -1 to start
        for (int i = 0; i < vertices.Length; i++)
        {
            // distance between hit position and particle
            Vector3 worldPos = transform.TransformPoint(vertices[i]);
            float distance = Vector3.Distance(worldPos, hitPosition);
            if (distance < minDistance) // typical minimisation loop
            {
                minDistance = distance;
                closestParticleIndex = i;
            }
        }
        return closestParticleIndex;
    }

    void FixedUpdate()
    {
        if (CSHARP_SIM)
        {
            windStrength = Mathf.Sin(Time.time) * 3.0f;
            //windStrength = Random.Range(-10.0f, 10.0f);
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
            if(algorithmType ==2)
            {
                cppFunctions.cpp_update(vertices, windStrength, stretchingCompliance, shearingCompliance, selectedParticleIndex, mouseWorldPos, stopGrabbingIndex);
            }
            else{
                cppFunctions.cpp_update(vertices, windStrength, stretchingStiffness, shearingStiffness, selectedParticleIndex, mouseWorldPos, stopGrabbingIndex);
            }
        }

        mesh.vertices = vertices;
        mesh.RecalculateNormals();
        //mesh.RecalculateTangents();
        // testing other normal calc
        //CalculateNormals(mesh);

        mesh.RecalculateBounds();
        boxCollider.center = mesh.bounds.center;
        boxCollider.size = mesh.bounds.size;
        // increment fixed update counter if we are recording
        if(isRecording)
        {
            fixedUpdateCounter++;
        }
    }
    
    void Update()
    {
        // If mouse is clicked, cast a ray from the camera to the mouse position
        // Dont do this if we are already dragging a particle
        if (Input.GetMouseButtonDown(0) && !isDragging)
        {
            stopGrabbingIndex = -1;
            Ray ray = Camera.main.ScreenPointToRay(Input.mousePosition);
            //Debug.DrawLine(ray.origin, ray.origin + ray.direction * 1000, Color.blue, 4.0f);
            RaycastHit hit;

            // Debug.Log("Mouse position: " + Input.mousePosition);
            // Debug.Log("ray origin: " + ray.origin);
            // If the ray hits the cloth, find the nearest particle
            if (Physics.Raycast(ray, out hit))
            {
                //Debug.Log("Hit position: " + hit.point);
                selectedParticleIndex = FindClosestParticleToMouse(hit.point);
                isDragging = true;
                //Debug.Log("Selected particle: " + selectedParticleIndex);
                Vector3 worldPos = transform.TransformPoint(vertices[selectedParticleIndex]);
                //distanceToCloth = Vector3.Distance(Camera.main.transform.position, worldPos);
                distanceToCloth = Camera.main.WorldToScreenPoint(hit.point).z;
                
            }
        }
        if(isDragging){
            Vector3 mouseScreenPos = new Vector3(Input.mousePosition.x, Input.mousePosition.y, distanceToCloth);
            mouseWorldPos = Camera.main.ScreenToWorldPoint(mouseScreenPos);
        }
        // Check for mouse button release
        if (Input.GetMouseButtonUp(0))
        {
            isDragging = false;
            stopGrabbingIndex = selectedParticleIndex;
            selectedParticleIndex = -1;
        }

        // Handle starting recordings
        if (!isRecording && pendingExperiments.Count > 0)
        {
            var iLastElement = pendingExperiments.Count - 1;
            var rp = pendingExperiments[iLastElement];
            pendingExperiments.RemoveAt(iLastElement);
            StartRecording(rp);
        }
        // Handle stopping them
        if(isRecording && fixedUpdateCounter>=maxFixedUpdates)
        {
            m_RecorderController.StopRecording();
            isRecording = false;
            fixedUpdateCounter = 0;
        }
       
        
    }

    

    private string GetDebuggerDisplay()
    {
        return ToString();
    }


  
}



