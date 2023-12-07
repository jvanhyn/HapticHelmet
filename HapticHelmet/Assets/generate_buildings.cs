using System.Collections;
using System.Collections.Generic;
using UnityEngine;
// using Random;

[RequireComponent(typeof (MeshFilter))]
[RequireComponent(typeof (MeshRenderer))]
public class generate_buildings : MonoBehaviour
{
    void Start () {

        int std_dist = 1;

        int[] x1 = {0, 0, 1, 2, 3, 4, 5, 6, 7, 5, 6, 7, 7};
        int[] x2 = {4, 2, 2, 4, 4, 6, 6, 8, 8, 6, 8, 8, 8};
        int[] y1 = {1, 3, 9, 9, 7, 4, 3, 2, 4, 9, 7, 6, 9};
        int[] y2 = {0, 2, 4, 8, 2, 3, 0, 1, 3, 5, 6, 5, 8};

        for(int i = 0; i < 13; i++) {
            CreateCube (x1[i], x2[i], y1[i], y2[i]);
        }
	}

	private void CreateCube (int x1, int x2, int y1, int y2) {

        // Random r = new Random();
        // int z = r.Next(0, 3);
        int z = 2;  

		Vector3[] vertices = {
			new Vector3 (x1, 0, y2),
			new Vector3 (x1, 0, y1),
			new Vector3 (x2, 0, y1),
			new Vector3 (x2, 0, y2),
			new Vector3 (x2, z, y2),
			new Vector3 (x2, z, y1),
			new Vector3 (x1, z, y1),
			new Vector3 (x1, z, y2),
		};

		int[] triangles = {
			0, 2, 1, //face front
			0, 3, 2,
			2, 3, 4, //face top
			2, 4, 5,
			1, 2, 5, //face right
			1, 5, 6,
			0, 7, 4, //face left
			0, 4, 3,
			5, 4, 7, //face back
			5, 7, 6,
			0, 6, 7, //face bottom
			0, 1, 6
		};
			
		Mesh mesh = GetComponent<MeshFilter> ().mesh;
		mesh.Clear ();
		mesh.vertices = vertices;
		mesh.triangles = triangles;
		mesh.Optimize ();
		mesh.RecalculateNormals ();
	}
}
