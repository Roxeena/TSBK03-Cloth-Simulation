using System.Collections.Generic;
using UnityEngine;

//http://www.xbdev.net/physics/Verlet/index.php
//http://graphics.stanford.edu/~mdfisher/cloth.html
//https://github.com/mattatz/unity-verlet-simulator
//https://gamedevelopment.tutsplus.com/tutorials/simulate-tearable-cloth-and-ragdolls-with-simple-verlet-integration--gamedev-519
//https://github.com/Fewes/MoS/blob/master/MoS_Cloth/Assets/VeryLett.cs

public class ClothPlane : MonoBehaviour {
    //Mesh to simulate
    private Mesh plane;

    //Arrays of vectors for vertices, forceC:\Users\ElmQPC\Desktop\Malin\TSBK03 - cloth simulation\TSBK03-Cloth-Simulation\Assets\GridGenerator.css, acceleration and velocity for each point in the grid
    private Vector3[] vertices, totalVelocity, Acceleration, Velocity, Position, Forces;

    //Arrays for previous positions and velocity for the vertices
    private Vector3[] oldAcceleration, oldVelocity, oldPositions, veryOldPositions;

    //Array of lists over every connection that every vertex has to each other
    private List<int>[] Conections;

    //External forces to bring more life into the simulation
    private Vector3 Gravity, Wind;

    //numCells of the cloth (default 2x2)
    public int numCells = 2;    //Assumes square cloth
    public float Size = 1.0f;

    [Range(0.0f, 1.0f)]
    public float friction = 0.9f;
    public SphereCollider sphere;

    //Number of time chuncks, left over time
    private int numTimeSteps = 16;
    private float numTimeStepsSec = 16.0f / 1000.0f;
    private int leftOverTime = 0;

    // Paremeters for cloth simulation
    public float b;                     //Damping   
    public float k;                     //Spring konstant in Hookes' law
    public float kd;                    //Spring konstant in Hookes' law (diagonal springs)
    public float m;                     //Mass of total cloth
    private float g = 9.8f;             //Gravity acceleration //TODO: Make it constant
    //public float WindForce = 0.0f;      //Wind strength
    //public float WindVariation = 0.1f;  //Wind spread over time //TODO: Change the wind to some noise
    private Transform transform;

    //Parameters that are calculated
    private int numVerticies;   //Number of vertices in the cloth
    private float ld;           //Length of diagonal springs
    private float l;             //Length of the vertical and horizontal springs

    //Boolean values to determin the orientation of the cloth, stuck to roof, stuck to mouse, hanging in the top edges
    public bool lockTopRow = false;     //Locks the vertices in the top row to their original positions
    public bool lockTopCorner = false;  //Locks the top right vertex to original position
    public bool lockBothTopCorners = false;

    private bool collision = false;
    private bool locked = false;
    
    //Initialization
    void Start () {
        //Calculate the gap between teh springs
        l = Size / numCells;

        //Get the mesh, vertices and calculate external forces
        plane = GetComponent<MeshFilter>().mesh;
        vertices = plane.vertices;
        numVerticies = vertices.Length;
        Gravity = new Vector3(0, -m*g, 0);
        //Wind = new Vector3(-WindForce, 0, WindForce);   //TODO: Switch wind to some kind of noise
        //Now we know the size of the cloth, declare all the arrays for this size
        transform = GetComponent<Transform>();
        totalVelocity = new Vector3[numVerticies];
        Forces = new Vector3[numVerticies];
        Acceleration = new Vector3[numVerticies];
        Velocity = new Vector3[numVerticies];
        Position = new Vector3[numVerticies];
        oldPositions = new Vector3[numVerticies];
        veryOldPositions = new Vector3[numVerticies];
        oldAcceleration = new Vector3[numVerticies];
        oldVelocity = new Vector3[numVerticies];
        Conections = new List<int>[numVerticies];   //Array of lists for all connections between vertices

        //Calculate the length of the diagonal springs   
        ld = Mathf.Sqrt(2 * l * l);
       
        //Loop over all vertices to add all the connections between them
        int row = numCells + 1; //numCells of one side (Assumes a square cloth, no rectangle)
        for (int v = 0; v < numVerticies; ++v)
        {
            //Assumes this structure for a 3x3 plane
            //[8 7 6]
            //[5 4 3]
            //[2 1 0]

            //Initialize connections
            Conections[v] = new List<int>();

            int rowMax = v + row - (v % row);   //Last index for this row
            int rowMin = v - (v % row) ;        //First index for this row    

            //Check (x-1, z-1)
            if ((v - 1 - row) >= 0 && (v - 1 - row) >= (rowMin - row)) {
                Conections[v].Add(v - 1 - row);             
            }
            //Check (x, z-1)
            if ((v - row) >= 0) {
                Conections[v].Add( v - row);
            }
            //Check (x+1, z-1)
            if ((v + 1 - row) >= 0 && (v + 1 - row) < (rowMax - row) && (v + 1 - row) < numVerticies && (v + 1 - row) > 0) {
                Conections[v].Add( v + 1 - row);
            }
            //Check(x-1, z)
            if ((v - 1) >= 0 && (v - 1) >= rowMin) {
                Conections[v].Add( v - 1);
            }

            //Skip yourself! (x, z)

            //Check (x+1, z)
            if ((v + 1) < rowMax && (v + 1) < numVerticies) {
                Conections[v].Add( v + 1);
            }
            //Check (x-1, z+1)
            if ((v - 1 + row) < numVerticies && (v - 1 + row) >= rowMin + row) {
                Conections[v].Add((v - 1 + row));
            }
            //Check (x, z+1)
            if ((v + row) < numVerticies) {
                Conections[v].Add( v + row);
            }
            //Check (x+1, z+1)
            if ((v + 1 + row) < numVerticies && (v + 1 + row) < rowMax + row) {
                Conections[v].Add( v + 1 + row);
            }

            //Check if we found any connections
            if(Conections[v] == null) {
                Debug.LogError("Faild to connect vertex: " + v + " to any other vertexs!");
            }

            //Initialize Arrays (since we go through all the vertices)
            totalVelocity[v] = Vector3.zero;
            Forces[v] = Vector3.zero;
            Acceleration[v] = Vector3.zero;
            Velocity[v] = Vector3.zero;
            Position[v] = Vector3.zero;
            oldPositions[v] = Vector3.zero;
            veryOldPositions[v] = Vector3.zero;
            oldVelocity[v] = Vector3.zero;
            oldAcceleration[v] = Vector3.zero;
        }             
	}
	
	//Fixed update is for more physics heavy calculations
	void FixedUpdate () {
        //Divide elapsed time into chuncks
        int timeStep = (int)((float)(Time.deltaTime*1000.0f + leftOverTime) / (float) numTimeSteps);
       
        //Limit the timeStep to prevent freezing
        timeStep = Mathf.Max(Mathf.Min(timeStep, 5), 1);

        //Store left over time
        leftOverTime = (int)(Time.deltaTime*1000.0f - (timeStep * numTimeSteps));

        //Divide the calculation into all the timesteps
        for(int iteration = 0; iteration < timeStep; iteration++)
        {
            //Get the vertices
            vertices = plane.vertices;

            //Go through all vertices and calculate the forces
            for (int i = 0; i < numVerticies; ++i)
            {
                //Reset totalVelocity
                totalVelocity[i] = Vector3.zero;
                Forces[i] = Vector3.zero;
                locked = false;

                //Lock top row of cloth or top corner of cloth if enabled
                if ((lockTopRow && i <= numCells) || (lockTopCorner && i == numCells) || (lockBothTopCorners && (i == 0 || i == numCells)))
                {
                    veryOldPositions[i] = new Vector3(vertices[i].x, vertices[i].y, vertices[i].z);
                    oldPositions[i] = veryOldPositions[i];
                    vertices[i] = oldPositions[i];
                    locked = true;
                    continue;
                }

                //Save old position
                if (Time.frameCount < 2)
                {
                    veryOldPositions[i] = new Vector3(vertices[i].x, vertices[i].y, vertices[i].z);
                    oldPositions[i] = veryOldPositions[i];
                }
                else
                {
                    veryOldPositions[i] = oldPositions[i];
                    oldPositions[i] = new Vector3(vertices[i].x, vertices[i].y, vertices[i].z);
                }

                

                //Depending on the connections calculate the spring and damper forces between theese conections
                foreach (int connection in Conections[i])
                {
                    Vector3 Connected = Vector3.zero;
                    //If the connection is to a vertex we already have calculated, the position is in oldPositions 
                    //We dont want to calculate forces depending on the new position we calculated
                    //If we have not calculated this vertex before, use its current position
                    Connected = (connection < i)? oldPositions[connection] : vertices[connection];

                    //Check if the connection is diagonal and calculate diagonal spring and damper froces     
                    bool diagonal = !((connection) == i + 1 || (connection) == i - 1 || (connection) == i + (numCells + 1) || (connection) == i - (numCells + 1));
                    totalVelocity[i] += calculateSpringWithDamper(oldVelocity[i], oldVelocity[connection], oldPositions[i], Connected, diagonal, i);
                }

                //Add gravity and wind
                Forces[i] += transform.InverseTransformDirection(Physics.gravity) * 0.1f;

                totalVelocity[i] += transform.InverseTransformDirection(Physics.gravity) * 0.1f* numTimeStepsSec;
                //totalVelocity[i] += Wind * WindVariation * Mathf.Abs(Mathf.Sin(Time.time));

                //Verlet method to update the position of the vertex
                Vector3 newPos = VerletMethod(i);
                vertices[i] = newPos;
            }

            //Update the vertices of the mesh
            plane.vertices = vertices;
        }
    }

    //Return the damper force in vec3, take in the velocities in vec3
    //Return the spring force in vec3, take in the position of the vertexes in vec3, pos1 and pos2, bool diagonal if it is a diagonal spring
    Vector3 calculateSpringWithDamper(Vector3 v1, Vector3 v2, Vector3 pos1, Vector3 pos2, bool diagonal, int i)
    {
        //Vector from 1 to 2
        Vector3 r = pos2 - pos1;

        // Calculate the distance between the two vertexes
        float dl = r.magnitude;

        // Calculate the Spring force depending on if it is a diagonal spinrg or not
        if (dl < 0.001f)    //avoid division with zero
            dl = 0.001f;

        Vector3 F = (diagonal) ? -kd*(dl - ld)*(pos1 - pos2)/dl  : -k*(dl - l)*(pos1 - pos2)/dl;

        //Calculate damping
        //Velocity total
        Vector3 v12 = v1 + v2;
        //Vector3 dampF = -b *(v1 - v2);
        Vector3 dampF = Vector3.zero;

        if(collision)
        {
            //float e = 0.5f;
            //Vector3 impuls = -(1 + e)* (Vector3.Scale(r.normalized, (v1 - v2))/(1.0f/m + 1.0f/1.0f));
            Forces[i] += (F + dampF) * friction ;
        }
            
        else
            Forces[i] += F + dampF;

        //Add the velocity up for later use
        Vector3 springDisplacement = F / m * numTimeStepsSec;
        return springDisplacement - dampF / m * numTimeStepsSec;
    }


    //Calculate the position of an vertex via the VerletMethod
    //TODO: Switch to Verlet integration
    //https://gamedevelopment.tutsplus.com/tutorials/simulate-tearable-cloth-and-ragdolls-with-simple-verlet-integration--gamedev-519
    Vector3 VerletMethod(int index)
    {
        collision = false;
        oldAcceleration[index] = Acceleration[index];
        Acceleration[index] = Forces[index] / m;
        oldVelocity[index] = Velocity[index];
        Velocity[index] = oldVelocity[index] + totalVelocity[index];
        Position[index] = 1.99f * oldPositions[index] - 0.99f * veryOldPositions[index] + numTimeStepsSec * numTimeStepsSec * Acceleration[index];

        //Check for collisions
        if (!locked)
        {
            //Floor
            if (transform.TransformPoint(Position[index]).y <= 0.05f)
            {
                Position[index].y = transform.InverseTransformPoint(Vector3.zero).y + 0.05f;
                collision = true;
            }

            //Sphere
            float radius = sphere.radius * sphere.transform.lossyScale.x;
            Vector3 collisionNormal = transform.TransformPoint(Position[index]) - sphere.transform.position;
            if (collisionNormal.magnitude <= radius*1.3f)
            {
                collision = true;
                Position[index] = transform.InverseTransformPoint(sphere.transform.position + collisionNormal.normalized * radius*1.3f);
            }
        }
        else
            collision = false;

        return Position[index];
    }
}
