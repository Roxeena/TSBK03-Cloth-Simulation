using UnityEngine;

//Put me on an empty game object!
//Tutorial on: http://catlikecoding.com/unity/tutorials/procedural-grid/

//GUI
[RequireComponent(typeof(MeshFilter), typeof(MeshRenderer))]

public class Grid : MonoBehaviour {

    public int xSize = 1 , ySize = 1; //Size of the grid, num vertices. (default 1x1)

    private Vector3[] vertices; //Vector for all vertices
    private Mesh mesh; //Rendered mesh

    //Function that generates vertices in a grid to a mesh etc...
    private void Generate()
    {
        //Create a new mesh for the MeshFilter component
        GetComponent<MeshFilter>().mesh = mesh = new Mesh();
        mesh.name = "Procedural Grid";

        //Create and calculate the vertices, texture coordinates and tangents
        vertices = new Vector3[(xSize + 1) * (ySize + 1)];  //why + 1?
        Vector2[] uv = new Vector2[vertices.Length];
        Vector4[] tangents = new Vector4[vertices.Length];
        Vector4 tangent = new Vector4(1f, 0f, 0f, -1f);
        for (int i = 0, y = 0; y <= ySize; y++) {
            for (int x = 0; x <= xSize; x++, i++) {
                vertices[i] = new Vector3(x, y, 0.0f);
                uv[i] = new Vector2((float)x / xSize, (float)y / ySize);
                tangents[i] = tangent;
            }
        }

        //Assign the vertices, texture coordinates and tangents to the mesh
        mesh.vertices = vertices;
        mesh.uv = uv;
        mesh.tangents = tangents;

        //Create the triangels for the mesh
        int[] triangles = new int[xSize * ySize * 6];
        for (int ti = 0, vi = 0, y = 0; y < ySize; y++, vi++) {
            for (int x = 0; x < xSize; x++, ti += 6, vi++) {
                triangles[ti] = vi;
                triangles[ti + 3] = triangles[ti + 2] = vi + 1;
                triangles[ti + 4] = triangles[ti + 1] = vi + xSize + 1;
                triangles[ti + 5] = vi + xSize + 2;
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
