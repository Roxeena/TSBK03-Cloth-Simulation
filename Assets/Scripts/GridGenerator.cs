﻿using UnityEngine;

//Inspiration from: http://catlikecoding.com/unity/tutorials/procedural-grid/

//GUI
[RequireComponent(typeof(MeshFilter), typeof(MeshRenderer))]

public class GridGenerator : MonoBehaviour {

    public int numCells = 10; //numCells of the grid, num vertices. Square always
    public float Size = 5.0f;    

    private Vector3[] vertices; //Vector for all vertices
    private Mesh mesh; //Rendered mesh
    private Transform transform;

    //Function that generates vertices in a grid to a mesh etc...
    private void Generate()
    {
        //Transform to global coordinates
        Vector3 stepY = transform.up * (Size / numCells);
        Vector3 stepX = transform.right * (Size / numCells);
        Vector3 stepZ = transform.forward;
        Vector3 origin = transform.position - transform.right * (Size / 2);
        
        //Create a new mesh for the MeshFilter component
        GetComponent<MeshFilter>().mesh = mesh = new Mesh();
        mesh.name = "Procedural Cloth";

        //Create and calculate the vertices, texture coordinates and tangents
        vertices = new Vector3[(numCells + 1) * (numCells + 1)];  //why + 1?
        Vector2[] uv = new Vector2[vertices.Length];
        Vector4[] tangents = new Vector4[vertices.Length];
        Vector4 tangent = new Vector4(1f, 0f, 0f, -1f);
        for (int i = 0, y = 0; y <= numCells; y++) {
            for (int x = 0; x <= numCells; x++, i++) {
                //Force cloth to be set in a wave pattern
                float wave = (x % 2 == 0) ? -0.1f: 0.1f;
                Vector3 pos = origin + stepX * (float)x - stepY * (float)y + stepZ * wave;
                
                vertices[i] = transform.InverseTransformPoint(pos);
                uv[i] = new Vector2((float)x / (float)numCells, (float)y / (float)numCells);
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
        transform = GetComponent<Transform>();
        Generate();
        mesh.RecalculateNormals();
    }
}
