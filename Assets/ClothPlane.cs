using System.Collections.Generic;
using UnityEngine;

public class ClothPlane : MonoBehaviour {
    //Mesh to simulate
    private Mesh plane;

    //Counter for how many frames have been rendered, only used the first three times
    int frameCount = 0;

    //Arrays of vectors for vertices, forces, acceleration and velocity for each point in the grid
    private Vector3[] vertices, Forces, Acceleration, Velocity, Position;

    //Arrays for previous positions and velocity for the vertices
    private Vector3[] oldAcceleration, oldVelocity, oldPositions, veryOldPositions;

    //Array of lists over every connection that every vertex has to each other
    private List<int>[] Conections;

    //External forces to bring more life into the simulation
    private Vector3 Gravity, Wind;

    //Size of the cloth (default 2x2)
    public int xMax = 2, zMax = 2;    

    // Paremeters for cloth simulation
    public float b;                     //Damping   
    public float l;                     //Length of the vertical and horizontal springs
    public int k;                       //Spring konstant in Hookes' law
    public float kd;                    //Spring konstant in Hookes' law (diagonal springs)
    public float m;                     //Mass of one vertex
    public float h;                     //Euler step length //TODO: Switch to verlet
    public float g = 9.82f;             //Gravity acceleration //TODO: Make it constant
    public float WindForce = 0.0f;      //Wind strength
    public float WindVariation = 0.1f;  //Wind spread over time //TODO: Change the wind to some noise

    //Parameters that are calculated
    private int numVerticies;   //Number of vertices in the cloth
    private float ld;           //Length of diagonal springs

    //Boolean values to determin the orientation of the cloth, stuck to roof, stuck to mouse, hanging in the top edges
    public bool lockTopRow = false;     //Locks the vertices in the top row to their original positions
    public bool lockTopCorner = false;  //Locks the top right vertex to original position
    public bool mouseInput = false;     //Allows the cloth to be manupulated with mouse input
 
    
    //Initialization
    void Start () {
        //Get the mesh, vertices and calculate external forces
        plane = GetComponent<MeshFilter>().mesh;
        vertices = plane.vertices;
        numVerticies = vertices.Length;
        Gravity = new Vector3(0, m*g, 0);
        Wind = new Vector3(-WindForce, 0, WindForce);   //TODO: Switch wind to some kind of noise

        //Now we know the size of the cloth, declare all the arrays for this size
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
        int row = xMax + 1; //Size of one side (Assumes a square cloth, no rectangle)
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
        //Go through all vertices and calculate the forces
        for (int i = 0; i < numVerticies; ++i)
        {
            //Lock top row of cloth or top corner of cloth if enabled
            if ((lockTopRow  &&  i >= numVerticies - (1 + xMax) )|| (lockTopCorner && i == numVerticies - 1))
            {
                Forces[i] = Vector3.zero;
                veryOldPositions[i] = new Vector3(vertices[i].x, vertices[i].y, vertices[i].z);
                oldPositions[i] = veryOldPositions[i];
                vertices[i] = oldPositions[i];
                continue;
            }

            //Allow left mousebutton to controll top left corner and right mouse button to controll lower left corner if enabled
            if (mouseInput && ( i == numVerticies - (xMax +1) && Input.GetMouseButton(0) ) || (i == 0 && Input.GetMouseButton(1) ))
            {               
                Forces[i] = Vector3.zero;
                Vector3 mousePos = Camera.main.ScreenToWorldPoint(new Vector3( Input.mousePosition.x, Input.mousePosition.y, 25));
                veryOldPositions[i] = new Vector3(mousePos.x, mousePos.y, mousePos.z);
                oldPositions[i] = veryOldPositions[i];
                vertices[i] = oldPositions[i];
                continue;
            }

            //Reset Forces
            Forces[i] = Vector3.zero;

            //Save old position
            if (frameCount < 2) {
                veryOldPositions[i] = new Vector3(vertices[i].x, vertices[i].y, vertices[i].z);
                oldPositions[i] = veryOldPositions[i];
            }
            else {
                veryOldPositions[i] = oldPositions[i];
                oldPositions[i] = new Vector3(vertices[i].x, vertices[i].y, vertices[i].z);
            }

            //Depending on the connections calculate the spring and damper forces between theese conections
            foreach (int connection in Conections[i])
            {
                Vector3 Connected = Vector3.zero;
                //If the connection is to a vertex we already have calculated, the position is in oldPositions 
                //We dont want to calculate forces depending on the new position we calculated
                if (connection < i) {
                    Connected = oldPositions[connection];
                }
                //If we have not calculated this vertex before, use its current position
                else {
                    Connected = vertices[connection];
                }

                //Check if the connection is diagonal and calculate diagonal spring froces     
                if ((connection) == i + 1 || (connection) == i - 1 || (connection) == i + (xMax + 1) || (connection) == i - (xMax + 1)) {
                    Forces[i] += calculateSpring(oldPositions[i], Connected);
                }
                //Diagonal spring
                else {
                    Forces[i] += calculateSpring(oldPositions[i], Connected, true);
                }
                
                //Calculate the damper forces
                Forces[i] += calculateDamper(oldVelocity[i], oldVelocity[connection]);
            }

            //Add gravity and wind
            Forces[i] += Gravity;
            Forces[i] += Wind * WindVariation * Mathf.Abs(Mathf.Sin(Time.time));

            //Euler method to update the position of the vertex
            //TODO: Change this to Verlet method instead
            Vector3 newPos = EulerMethod(i);
            vertices[i] = newPos;
        }

        //Update the vertices of the mesh
        plane.vertices = vertices;
        vertices = plane.vertices;

        //Only count the first 10 frames
        if (frameCount < 10) frameCount++;
    }

    //Return the damper force in vec3, take in the velocities in vec3
    Vector3 calculateDamper(Vector3 v1, Vector3 v2)
    {
        return b*(v1 - v2);
    }

    //Return the spring force in vec3, take in the position of the vertexes in vec3, pos1 and pos2, bool diagonal if it is a diagonal spring
    Vector3 calculateSpring(Vector3 pos1, Vector3 pos2, bool diagonal = false)
    {
        // Calculate the distance between the two vertexes
        float dl = (pos1 - pos2).magnitude;
        
        // Calculate the Spring force depending on if it is a diagonal spinrg or not
        if( dl < 0.001f)//avoid division with zero
        {
            dl = 0.001f;
        }
        if (diagonal)
        {
            return kd*(dl - ld)*(pos1 - pos2)/dl;
        } 
        return k*(dl - l)*(pos1 - pos2)/dl;
    }

    //Calculate the position of an vertex via the EulerMethod with a steplength of h
    //TODO: Switch to Verlet integration
    //https://gamedevelopment.tutsplus.com/tutorials/simulate-tearable-cloth-and-ragdolls-with-simple-verlet-integration--gamedev-519
    Vector3 EulerMethod(int index)
    {
        //Save old position
        if (frameCount < 1) {
            oldAcceleration[index] = -Forces[index] / m;
            Acceleration[index] = oldAcceleration[index];
        }
        else {
            oldAcceleration[index] = Acceleration[index];
            Acceleration[index] = -Forces[index] / m;
        }
        Velocity[index] = oldVelocity[index] + h * Acceleration[index];
        oldVelocity[index] = Velocity[index];
        Position[index] = 2.0f*oldPositions[index] - veryOldPositions[index] + oldAcceleration[index];
        return Position[index];
    }
}
