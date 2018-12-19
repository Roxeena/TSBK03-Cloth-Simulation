using UnityEngine;

//Put me on an empty game object!
//Tutorial on: http://catlikecoding.com/unity/tutorials/procedural-grid/

//GUI
[RequireComponent(typeof(MeshFilter), typeof(MeshRenderer))]

public class GridGenerator : MonoBehaviour {

    public int numCells = 1; //numCells of the grid, num vertices. (default 1x1), Square always
    public float Size = 1.0f;
    [Range(0, 5)]
    public float randomNess = 0.0f;
    

    private Vector3[] vertices; //Vector for all vertices
    private Mesh mesh; //Rendered mesh

    //Function that generates vertices in a grid to a mesh etc...
    private void Generate()
    {
        //Create a new mesh for the MeshFilter component
        GetComponent<MeshFilter>().mesh = mesh = new Mesh();
        mesh.name = "Procedural Grid";

        //Create and calculate the vertices, texture coordinates and tangents
        vertices = new Vector3[(numCells + 1) * (numCells + 1)];  //why + 1?
        Vector2[] uv = new Vector2[vertices.Length];
        Vector4[] tangents = new Vector4[vertices.Length];
        Vector4 tangent = new Vector4(1f, 0f, 0f, -1f);
        for (int i = 0, y = 0; y <= numCells; y++) {
            for (int x = 0; x <= numCells; x++, i++) {
                float posX = (Size / numCells) * (float)x + randomNess * Random.onUnitSphere.x * Mathf.Epsilon;
                float posY = (Size / numCells) * (float)y + randomNess * Random.onUnitSphere.y * Mathf.Epsilon;
                vertices[i] = new Vector3(posX, posY, randomNess * Random.onUnitSphere.z * Mathf.Epsilon);
                uv[i] = new Vector2((float)x / numCells, (float)y / numCells);
                tangents[i] = tangent;
            }
        }

        //Assign the vertices, texture coordinates and tangents to the mesh
        mesh.vertices = vertices;
        mesh.uv = uv;
        mesh.tangents = tangents;

        //Create the triangels for the mesh
        int[] triangles = new int[numCells * numCells * 6];
        int idx = 0;
        int numPoints = numCells + 1;

        for (int y = 0; y < numCells; y++) {
            for (int x = 0; x < numCells; x++) {
                //triangel 1
                triangles[idx++] = numPoints*y + x;
                triangles[idx++] = numPoints*y + x + 1;
                triangles[idx++] = numPoints*y + x + numPoints;
                //triangel 2
                triangles[idx++] = numPoints*y + x + 1;
                triangles[idx++] = numPoints*y + x + numPoints + 1;
                triangles[idx++] = numPoints * y + x + numPoints;
            }
        }

        //Assign the triangles
        mesh.triangles = triangles;
    }

    //Function that generates the grid upon awake of the program
    private void Awake()
    {
        Generate();
        mesh.RecalculateNormals();
    }
}
