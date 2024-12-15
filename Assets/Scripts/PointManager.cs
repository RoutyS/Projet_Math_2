using System.Collections;
using System.Collections.Generic;
using Unity.VisualScripting;
using UnityEngine;
using System.Linq;

public class PointManager : MonoBehaviour
{
    public GameObject pointPrefab; // Prefab pour repr�senter un point visuellement
    public LineRenderer lineRenderer; // Pour tracer l'enveloppe convexe

    private List<Vector2> points = new List<Vector2>();
    private List<GameObject> pointObjects = new List<GameObject>();
    private List<Triangle> triangles = new List<Triangle>();

    void Update()
    {
        if (Input.GetMouseButtonDown(0)) // Clic gauche de la souris
        {
            AddPoint();
        }
        if (Input.GetKeyDown(KeyCode.Space)) // Touche Espace pour Jarvis March
        {
            JarvisMarch();
        }
        if (Input.GetKeyDown(KeyCode.G)) // Touche G pour Graham Scan
        {
            GrahamScan();
        }
        if (Input.GetKeyDown(KeyCode.T)) // Touche T pour triangulation incr�mentale
        {
            TriangulationIncrementale();
        }
        if (Input.GetKeyDown(KeyCode.D)) // Touche D pour Delaunay
        {
            TriangulationDelaunay();
        }

    }

    void AddPoint()
    {
        // Convertir la position de la souris en coordonn�es du monde
        Vector3 mousePos = Input.mousePosition;
        mousePos.z = Mathf.Abs(Camera.main.transform.position.z);
        Vector3 worldPos = Camera.main.ScreenToWorldPoint(mousePos);
        Vector2 pointPos = new Vector2(worldPos.x, worldPos.y);

        // Ajouter le point � la liste
        points.Add(pointPos);
        GameObject newPoint = Instantiate(pointPrefab, new Vector3(pointPos.x, pointPos.y, 0), Quaternion.identity);
        pointObjects.Add(newPoint);
    }

    // Jarvis March
    public void JarvisMarch()
    {
        if (points.Count < 3) return;
        List<Vector2> hull = new List<Vector2>();

        // Trouver le point le plus � gauche
        Vector2 leftmost = points[0];
        foreach (Vector2 p in points)
            if (p.x < leftmost.x) leftmost = p;

        Vector2 current = leftmost;
        do
        {
            hull.Add(current);
            Vector2 next = points[0];
            foreach (Vector2 candidate in points)
            {
                if (next == current || IsCounterClockwise(current, next, candidate))
                    next = candidate;
            }
            current = next;
        } while (current != leftmost);

        DrawHull(hull);
    }

    // Graham Scan
    public void GrahamScan()
    {
        if (points.Count < 3) return;

        // �tape 1 : Trouver le point le plus bas
        Vector2 pivot = points.OrderBy(p => p.y).ThenBy(p => p.x).First();

        // �tape 2 : Trier les points par angle polaire
        List<Vector2> sortedPoints = points.OrderBy(p => Mathf.Atan2(p.y - pivot.y, p.x - pivot.x)).ToList();

        // �tape 3 : Construire l'enveloppe convexe
        Stack<Vector2> hull = new Stack<Vector2>();
        hull.Push(sortedPoints[0]);
        hull.Push(sortedPoints[1]);

        for (int i = 2; i < sortedPoints.Count; i++)
        {
            Vector2 top = hull.Pop();
            while (hull.Count > 0 && !IsCounterClockwise(hull.Peek(), top, sortedPoints[i]))
            {
                top = hull.Pop();
            }
            hull.Push(top);
            hull.Push(sortedPoints[i]);
        }

        DrawHull(hull.ToList());
    }

    bool IsCounterClockwise(Vector2 a, Vector2 b, Vector2 c)
    {
        return (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x) > 0;
    }

    public void DrawHull(List<Vector2> hull)
    {
        lineRenderer.positionCount = hull.Count + 1;
        for (int i = 0; i < hull.Count; i++)
        {
            lineRenderer.SetPosition(i, new Vector3(hull[i].x, hull[i].y, 0));
        }
        lineRenderer.SetPosition(hull.Count, new Vector3(hull[0].x, hull[0].y, 0)); // Boucler le polygone
        lineRenderer.loop = true;
    }

    /* 
    Pour tester Jarvis March, d�commentez cette ligne dans Update() :
        JarvisMarch();
    Et commentez GrahamScan().
    
    Pour tester Graham Scan, d�commentez cette ligne dans Update() :
        GrahamScan();
    Et commentez JarvisMarch().
    */

    void TriangulationIncrementale()
    {
        if (points.Count < 3)
        {
            Debug.Log("Pas assez de points pour effectuer la triangulation incr�mentale.");
            return;
        }
        Debug.Log("Triangulation incr�mentale commenc�e.");

        if (points.Count < 3) return;
        triangles.Clear();

        // Ajouter un triangle initial (simple pour d�buter)
        triangles.Add(new Triangle(points[0], points[1], points[2]));

        for (int i = 3; i < points.Count; i++)
        {
            AddPointToTriangulation(points[i]);
        }
        DrawTriangles();
    }

    void AddPointToTriangulation(Vector2 newPoint)
    {
        List<Triangle> badTriangles = new List<Triangle>();

        // Trouver tous les triangles dont le cercle circonscrit contient le nouveau point
        foreach (Triangle triangle in triangles)
        {
            if (triangle.IsPointInCircumcircle(newPoint))
            {
                badTriangles.Add(triangle);
            }
        }

        // Cr�er de nouvelles ar�tes
        List<Edge> polygon = GetPolygonEdges(badTriangles);

        // Retirer les mauvais triangles
        foreach (Triangle badTriangle in badTriangles)
        {
            triangles.Remove(badTriangle);
        }

        // Cr�er de nouveaux triangles
        foreach (Edge edge in polygon)
        {
            triangles.Add(new Triangle(edge.A, edge.B, newPoint));
        }
    }

    List<Edge> GetPolygonEdges(List<Triangle> badTriangles)
    {
        List<Edge> edges = new List<Edge>();
        foreach (Triangle triangle in badTriangles)
        {
            edges.AddRange(triangle.GetEdges());
        }
        return edges;
    }

    void TriangulationDelaunay()
    {
        if (triangles.Count == 0)
        {
            Debug.Log("Aucun triangle pour effectuer la triangulation de Delaunay.");
            return;
        }
        Debug.Log("Triangulation de Delaunay commenc�e.");

        bool flipped;
        do
        {
            flipped = false;
            foreach (var triangle in triangles.ToList())
            {
                foreach (var edge in triangle.GetEdges())
                {
                    var adjacentTriangle = FindAdjacentTriangle(triangle, edge);
                    if (adjacentTriangle != null && !IsDelaunay(edge, triangle, adjacentTriangle))
                    {
                        FlipEdge(edge, triangle, adjacentTriangle);
                        flipped = true;
                    }
                }
            }
        } while (flipped);
        DrawTriangles();
    }

    Triangle FindAdjacentTriangle(Triangle triangle, Edge edge)
    {
        foreach (var other in triangles)
        {
            if (other != triangle && other.HasEdge(edge))
            {
                return other;
            }
        }
        return null;
    }

    bool IsDelaunay(Edge edge, Triangle t1, Triangle t2)
    {
        Vector2 opposite = t2.GetOppositePoint(edge);
        return !t1.IsPointInCircumcircle(opposite);
    }

    void FlipEdge(Edge edge, Triangle t1, Triangle t2)
    {
        Vector2 a = edge.A;
        Vector2 b = edge.B;
        Vector2 c = t1.GetOppositePoint(edge);
        Vector2 d = t2.GetOppositePoint(edge);

        triangles.Remove(t1);
        triangles.Remove(t2);

        triangles.Add(new Triangle(a, c, d));
        triangles.Add(new Triangle(b, c, d));
    }

    void DrawTriangles()
    {
        foreach (Triangle triangle in triangles)
        {
            Debug.DrawLine(triangle.A, triangle.B, Color.green, 10f);
            Debug.DrawLine(triangle.B, triangle.C, Color.green, 10f);
            Debug.DrawLine(triangle.C, triangle.A, Color.green, 10f);
        }
    }


}

public class Triangle
{
    public Vector2 A, B, C;

    public Triangle(Vector2 a, Vector2 b, Vector2 c)
    {
        A = a; B = b; C = c;
    }

    public bool IsPointInCircumcircle(Vector2 point)
    {
        float ax = A.x - point.x;
        float ay = A.y - point.y;
        float bx = B.x - point.x;
        float by = B.y - point.y;
        float cx = C.x - point.x;
        float cy = C.y - point.y;

        float det = (ax * (by * cy - by * by) - ay * (bx * cy - cx * bx) + (ax * bx - ay * by) * cx);
        return det > 0;
    }

    public List<Edge> GetEdges()
    {
        return new List<Edge>
        {
            new Edge(A, B),
            new Edge(B, C),
            new Edge(C, A)
        };
    }

    public bool HasEdge(Edge edge)
    {
        return (A == edge.A && B == edge.B) || (A == edge.B && B == edge.A) ||
               (B == edge.A && C == edge.B) || (B == edge.B && C == edge.A) ||
               (C == edge.A && A == edge.B) || (C == edge.B && A == edge.A);
    }

    public Vector2 GetOppositePoint(Edge edge)
    {
        if (A != edge.A && A != edge.B) return A;
        if (B != edge.A && B != edge.B) return B;
        return C;
    }
}

public struct Edge
{
    public Vector2 A, B;

    public Edge(Vector2 a, Vector2 b)
    {
        A = a;
        B = b;
    }

    public override bool Equals(object obj)
    {
        if (obj is Edge other)
        {
            return (A == other.A && B == other.B) || (A == other.B && B == other.A);
        }
        return false;
    }

    public override int GetHashCode()
    {
        return A.GetHashCode() ^ B.GetHashCode();
    }
}