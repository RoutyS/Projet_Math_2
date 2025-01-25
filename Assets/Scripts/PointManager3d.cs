using System;
using System.Collections.Generic;
using UnityEngine;
using System.Linq;

public class PointManager3D  : MonoBehaviour
{ 
    private List<Vector3> points3D = new List<Vector3>();
    private List<Tetrahedron> tetrahedra = new List<Tetrahedron>();

 
    public void ConvexHull3D()
    {
        if (points3D.Count < 4) return;

        
        Tetrahedron initialTetra = CreateInitialTetrahedron(points3D.Take(4).ToList());
        tetrahedra.Add(initialTetra);


        for (int i = 4; i < points3D.Count; i++)
        {
            AddPointToConvexHull3D(points3D[i]);
        }
    }

    private Tetrahedron CreateInitialTetrahedron(List<Vector3> initialPoints)
    {
        if (initialPoints.Count != 4)
            throw new ArgumentException("Need exactly 4 points to create initial tetrahedron");

        return new Tetrahedron(
            initialPoints[0], 
            initialPoints[1], 
            initialPoints[2], 
            initialPoints[3]
        );
    }

    private void AddPointToConvexHull3D(Vector3 newPoint)
    {
        List<Tetrahedron> badTetrahedra = new List<Tetrahedron>();

        foreach (var tetra in tetrahedra)
        {
            if (tetra.IsPointInCircumsphere(newPoint))
            {
                badTetrahedra.Add(tetra);
            }
        }

      
        var boundaryFaces = GetBoundaryFaces(badTetrahedra);

        foreach (var badTetra in badTetrahedra)
        {
            tetrahedra.Remove(badTetra);
        }

       
        foreach (var face in boundaryFaces)
        {
            tetrahedra.Add(new Tetrahedron(
                face.A, face.B, face.C, newPoint
            ));
        }
    }

   
    public void Delaunay3D()
    {
      
        if (points3D.Count < 4) return;

        Tetrahedron superTetra = CreateSuperTetrahedron();
        tetrahedra.Add(superTetra);

        // Add points incrementally
        foreach (var point in points3D)
        {
            AddPointDelaunay3D(point);
        }

        
        RemoveSuperTetrahedronConnections();
    }

    private Tetrahedron CreateSuperTetrahedron()
    {
  
        Vector3 min = new Vector3(
            points3D.Min(p => p.x),
            points3D.Min(p => p.y),
            points3D.Min(p => p.z)
        );
        Vector3 max = new Vector3(
            points3D.Max(p => p.x),
            points3D.Max(p => p.y),
            points3D.Max(p => p.z)
        );

        
        float margin = Mathf.Max(
            max.x - min.x, 
            max.y - min.y, 
            max.z - min.z
        ) * 2;

        Vector3 center = (min + max) / 2;
        return new Tetrahedron(
            center + new Vector3(-margin, -margin, -margin),
            center + new Vector3(margin, -margin, -margin),
            center + new Vector3(0, margin, -margin),
            center + new Vector3(0, 0, margin)
        );
    }

    private void AddPointDelaunay3D(Vector3 point)
    {
        List<Tetrahedron> badTetrahedra = tetrahedra
            .Where(t => t.IsPointInCircumsphere(point))
            .ToList();

        
        var boundaryFaces = GetBoundaryFaces(badTetrahedra);

       
        foreach (var tetra in badTetrahedra)
        {
            tetrahedra.Remove(tetra);
        }

      
        foreach (var face in boundaryFaces)
        {
            tetrahedra.Add(new Tetrahedron(
                face.A, face.B, face.C, point
            ));
        }
    }

    private void RemoveSuperTetrahedronConnections()
    {
        
        tetrahedra.RemoveAll(t => 
            t.HasSuperTetrahedronVertex() || 
            t.IsDegenerate()
        );
    }

    private List<Face3D> GetBoundaryFaces(List<Tetrahedron> tetrahedra)
    {
        var faceCounts = new Dictionary<Face3D, int>();

      
        foreach (var tetra in tetrahedra)
        {
            foreach (var face in tetra.GetFaces())
            {
                if (!faceCounts.ContainsKey(face))
                    faceCounts[face] = 0;
                faceCounts[face]++;
            }
        }

       
        return faceCounts
            .Where(kvp => kvp.Value == 1)
            .Select(kvp => kvp.Key)
            .ToList();
    }

  
    public void GenerateVoronoi3D()
    {
        
        List<Vector3> voronoiVertices = new List<Vector3>();

        foreach (var tetra in tetrahedra)
        {
            Vector3 circumcenter = CalculateCircumcenter3D(
                tetra.A, tetra.B, tetra.C, tetra.D
            );
            voronoiVertices.Add(circumcenter);
        }

       
    }

    private Vector3 CalculateCircumcenter3D(Vector3 a, Vector3 b, Vector3 c, Vector3 d)
    {
        // Compute 3D circumcenter using geometric calculations
        Matrix4x4 augmentedMatrix = new Matrix4x4();
        
        augmentedMatrix.SetRow(0, new Vector4(a.x, a.y, a.z, a.sqrMagnitude));
        augmentedMatrix.SetRow(1, new Vector4(b.x, b.y, b.z, b.sqrMagnitude));
        augmentedMatrix.SetRow(2, new Vector4(c.x, c.y, c.z, c.sqrMagnitude));
        augmentedMatrix.SetRow(3, new Vector4(d.x, d.y, d.z, d.sqrMagnitude));

        Matrix4x4 inverseMatrix = augmentedMatrix.inverse;
        
        return new Vector3(
            inverseMatrix.m00 / 2f,
            inverseMatrix.m10 / 2f,
            inverseMatrix.m20 / 2f
        );
    }
}


public class Tetrahedron
{
    public Vector3 A, B, C, D;

    public Tetrahedron(Vector3 a, Vector3 b, Vector3 c, Vector3 d)
    {
        A = a; B = b; C = c; D = d;
    }

    public bool IsPointInCircumsphere(Vector3 point)
    {
        Vector3 circumCenter = CalculateCircumcenter3D(A, B, C, D);
        float radius = Vector3.Distance(circumCenter, A);
        return Vector3.Distance(circumCenter, point) <= radius;
    }
    private Vector3 CalculateCircumcenter3D(Vector3 a, Vector3 b, Vector3 c, Vector3 d)
    {
        Matrix4x4 augmentedMatrix = new Matrix4x4();
        
        augmentedMatrix.SetRow(0, new Vector4(a.x, a.y, a.z, a.sqrMagnitude));
        augmentedMatrix.SetRow(1, new Vector4(b.x, b.y, b.z, b.sqrMagnitude));
        augmentedMatrix.SetRow(2, new Vector4(c.x, c.y, c.z, c.sqrMagnitude));
        augmentedMatrix.SetRow(3, new Vector4(d.x, d.y, d.z, d.sqrMagnitude));

        Matrix4x4 inverseMatrix = augmentedMatrix.inverse;
        
        return new Vector3(
            inverseMatrix.m00 / 2f,
            inverseMatrix.m10 / 2f,
            inverseMatrix.m20 / 2f
        );
    }
    public List<Face3D> GetFaces()
    {
        return new List<Face3D>
        {
            new Face3D(A, B, C),
            new Face3D(A, B, D),
            new Face3D(A, C, D),
            new Face3D(B, C, D)
        };
    }

    public bool HasSuperTetrahedronVertex()
    {
    
        return IsSpecialVertex(A) || IsSpecialVertex(B) || 
               IsSpecialVertex(C) || IsSpecialVertex(D);
    }

    private bool IsSpecialVertex(Vector3 vertex)
    {
        
        return Mathf.Abs(vertex.x) > 1e6f || 
               Mathf.Abs(vertex.y) > 1e6f || 
               Mathf.Abs(vertex.z) > 1e6f;
    }

    public bool IsDegenerate()
    {
        
        float volume = Mathf.Abs(Vector3.Dot(
            Vector3.Cross(B - A, C - A), 
            D - A
        )) / 6f;

        return volume < 1e-10f;
    }
}

public class Face3D : IEquatable<Face3D>
{
    public Vector3 A, B, C;

    public Face3D(Vector3 a, Vector3 b, Vector3 c)
    {
    
        Vector3[] sortedPoints = new[] { a, b, c }
            .OrderBy(p => p.x)
            .ThenBy(p => p.y)
            .ThenBy(p => p.z)
            .ToArray();

        A = sortedPoints[0];
        B = sortedPoints[1];
        C = sortedPoints[2];
    }

    public bool Equals(Face3D other)
    {
        if (other == null) return false;
        return Vector3.Distance(A, other.A) < 1e-6f &&
               Vector3.Distance(B, other.B) < 1e-6f &&
               Vector3.Distance(C, other.C) < 1e-6f;
    }
    public override bool Equals(object obj)
    {
        return Equals(obj as Face3D);
    }

    public override int GetHashCode()
    {
        return A.GetHashCode() ^ B.GetHashCode() ^ C.GetHashCode();
    }
    
    
}