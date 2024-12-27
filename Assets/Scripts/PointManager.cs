using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.Linq;
using System.Diagnostics;

public class PointManager : MonoBehaviour
{
    public GameObject pointPrefab; // Prefab pour représenter un point visuellement
    public LineRenderer lineRenderer; // Pour tracer l'enveloppe convexe
    private GameObject voronoiParent;

    public Color colorJarvis = Color.blue;
    public Color colorGraham = Color.green;
    public Color colorIncremental = Color.cyan;
    public Color colorDelaunay = Color.yellow;
    public Color colorVoronoi = Color.red;

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
            MeasureExecutionTime(JarvisMarch, "Jarvis March");
        }
        if (Input.GetKeyDown(KeyCode.G)) // Touche G pour Graham Scan
        {
            MeasureExecutionTime(GrahamScan, "Graham Scan");
        }
        if (Input.GetKeyDown(KeyCode.T)) // Touche T pour triangulation incrémentale
        {
            MeasureExecutionTime(TriangulationIncrementale, "Triangulation Incrémentale");
        }
        if (Input.GetKeyDown(KeyCode.D)) // Triangulation de Delaunay
        {
            MeasureExecutionTime(TriangulationDelaunay, "Triangulation de Delaunay");
        }
        if (Input.GetKeyDown(KeyCode.A)) // Ajouter un point avec Delaunay
        {
            MeasureExecutionTime(() => AddPointDelaunay(new Vector2(Random.Range(-5f, 5f), Random.Range(-5f, 5f))), "Ajout de Point pour Delaunay");
        }
        if (Input.GetKeyDown(KeyCode.R)) // Supprimer un point avec Delaunay
        {
            if (points.Count > 0)
            {
                MeasureExecutionTime(() => RemovePointDelaunay(points[Random.Range(0, points.Count)]), "Suppression de Point pour Delaunay");
            }
        }
        if (Input.GetKeyDown(KeyCode.V)) // Générer le diagramme de Voronoï
        {
            MeasureExecutionTime(GenerateVoronoi, "Génération du Diagramme de Voronoï");
        }

    }

    void AddPoint()
    {
        // Convertir la position de la souris en coordonnées du monde
        Vector3 mousePos = Input.mousePosition;
        mousePos.z = Mathf.Abs(Camera.main.transform.position.z);
        Vector3 worldPos = Camera.main.ScreenToWorldPoint(mousePos);
        Vector2 pointPos = new Vector2(worldPos.x, worldPos.y);

        // Ajouter le point à la liste
        points.Add(pointPos);
        GameObject newPoint = Instantiate(pointPrefab, new Vector3(pointPos.x, pointPos.y, 0), Quaternion.identity);
        pointObjects.Add(newPoint);

        // Mettre à jour la triangulation en temps réel
        if (points.Count >= 3)
        {
            TriangulationIncrementale();
        }
    }

    void MeasureExecutionTime(System.Action algorithm, string algorithmName)
    {
        var stopwatch = System.Diagnostics.Stopwatch.StartNew();
        algorithm();
        stopwatch.Stop();
        UnityEngine.Debug.Log($"{algorithmName} executed in {stopwatch.ElapsedMilliseconds} ms.");
    }

    // Enveloppe convexe avec Jarvis March
    public void JarvisMarch()
    {
        if (points.Count < 3) return;

        List<Vector2> hull = new List<Vector2>();
        Vector2 leftmost = points.OrderBy(p => p.x).First();
        Vector2 current = leftmost;

        do
        {
            hull.Add(current);
            Vector2 next = points[0];
            foreach (var candidate in points)
            {
                if (next == current || IsCounterClockwise(current, next, candidate))
                {
                    next = candidate;
                }
            }
            current = next;
        } while (current != leftmost);

        DrawHull(hull, colorJarvis);
    }

    // Enveloppe convexe avec Graham Scan
    public void GrahamScan()
    {
        if (points.Count < 3) return;

        Vector2 pivot = points.OrderBy(p => p.y).ThenBy(p => p.x).First();
        var sortedPoints = points.OrderBy(p => Mathf.Atan2(p.y - pivot.y, p.x - pivot.x)).ToList();

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

        DrawHull(hull.ToList(), colorGraham);
    }

    bool IsCounterClockwise(Vector2 a, Vector2 b, Vector2 c)
    {
        return (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x) > 0;
    }

    public void DrawHull(List<Vector2> hull, Color lineColor)
    {
        lineRenderer.positionCount = hull.Count + 1;
        lineRenderer.startWidth = 0.02f;
        lineRenderer.endWidth = 0.02f;
        lineRenderer.material = new Material(Shader.Find("Sprites/Default"));
        lineRenderer.startColor = lineColor;
        lineRenderer.endColor = lineColor;

        for (int i = 0; i < hull.Count; i++)
        {
            lineRenderer.SetPosition(i, new Vector3(hull[i].x, hull[i].y, 0));
        }
        lineRenderer.SetPosition(hull.Count, new Vector3(hull[0].x, hull[0].y, 0));
    }

    // Triangulation incrémentale
    void TriangulationIncrementale()
    {
        if (points.Count < 3) return;

        triangles.Clear();
        triangles.Add(new Triangle(points[0], points[1], points[2]));

        for (int i = 3; i < points.Count; i++)
        {
            AddPointToTriangulation(points[i]);
        }
        DrawTriangles(colorIncremental);
    }

    void AddPointToTriangulation(Vector2 newPoint)
    {
        List<Triangle> badTriangles = new List<Triangle>();
        foreach (var triangle in triangles)
        {
            if (triangle.IsPointInCircumcircle(newPoint))
            {
                badTriangles.Add(triangle);
            }
        }

        var polygon = GetPolygonEdges(badTriangles);

        foreach (var badTriangle in badTriangles)
        {
            triangles.Remove(badTriangle);
        }

        foreach (var edge in polygon)
        {
            triangles.Add(new Triangle(edge.A, edge.B, newPoint));
        }
    }

    List<Edge> GetPolygonEdges(List<Triangle> badTriangles)
    {
        List<Edge> edges = new List<Edge>();
        foreach (var triangle in badTriangles)
        {
            edges.AddRange(triangle.GetEdges());
        }
        return edges;
    }

    // Triangulation de Delaunay
    void TriangulationDelaunay()
    {
        if (triangles.Count == 0) return;

        bool flipped;
        int iterationLimit = 1000;
        int iteration = 0;

        do
        {
            flipped = false;
            iteration++;

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

            if (iteration > iterationLimit)
            {
                UnityEngine.Debug.LogError("Triangulation de Delaunay failed: infinite loop detected.");
                break;
            }
        } while (flipped);

        DrawTriangles(colorDelaunay);
    }


    Triangle FindAdjacentTriangle(Triangle triangle, Edge edge)
    {
        return triangles.FirstOrDefault(t => t != triangle && t.HasEdge(edge));
    }

    bool IsDelaunay(Edge edge, Triangle t1, Triangle t2)
    {
        var opposite = t2.GetOppositePoint(edge);
        return !t1.IsPointInCircumcircle(opposite);
    }

    void FlipEdge(Edge edge, Triangle t1, Triangle t2)
    {
        var a = edge.A;
        var b = edge.B;
        var c = t1.GetOppositePoint(edge);
        var d = t2.GetOppositePoint(edge);

        triangles.Remove(t1);
        triangles.Remove(t2);

        triangles.Add(new Triangle(a, c, d));
        triangles.Add(new Triangle(b, c, d));
    }

    void DrawTriangles(Color lineColor)
    {
        foreach (var triangle in triangles)
        {
            CreateTriangleVisualization(triangle, lineColor);
        }
    }

    void CreateTriangleVisualization(Triangle triangle, Color lineColor)
    {
        GameObject triangleObject = new GameObject("Triangle");
        LineRenderer lineRenderer = triangleObject.AddComponent<LineRenderer>();

        lineRenderer.positionCount = 4;
        lineRenderer.startWidth = 0.02f;
        lineRenderer.endWidth = 0.02f;
        lineRenderer.material = new Material(Shader.Find("Sprites/Default"));
        lineRenderer.startColor = lineColor;
        lineRenderer.endColor = lineColor;

        lineRenderer.SetPosition(0, new Vector3(triangle.A.x, triangle.A.y, 0));
        lineRenderer.SetPosition(1, new Vector3(triangle.B.x, triangle.B.y, 0));
        lineRenderer.SetPosition(2, new Vector3(triangle.C.x, triangle.C.y, 0));
        lineRenderer.SetPosition(3, new Vector3(triangle.A.x, triangle.A.y, 0));
    }

    public void AddPointDelaunay(Vector2 newPoint)
    {
        // Ajoutez le nouveau point et ajustez les triangles
        AddPointToTriangulation(newPoint);
        TriangulationDelaunay();
    }

    public void RemovePointDelaunay(Vector2 pointToRemove)
    {
        if (!points.Contains(pointToRemove)) return; // Vérifie que le point existe
        points.Remove(pointToRemove);
        triangles.Clear();
        TriangulationIncrementale();
        TriangulationDelaunay();
    }


    // Diagramme de Voronoï
   void GenerateVoronoi()
    {
        if (voronoiParent != null)
        {
            Destroy(voronoiParent);
        }
        voronoiParent = new GameObject("VoronoiParent");

        if (triangles.Count == 0 || points.Count < 3)
        {
            UnityEngine.Debug.LogError("Impossible de générer un diagramme de Voronoï avec moins de 3 points.");
            return;
        }

        foreach (var triangle in triangles)
        {
            var circumcenter1 = CalculateCircumcenter(triangle.A, triangle.B, triangle.C);

            foreach (var edge in triangle.GetEdges())
            {
                var adjacentTriangle = FindAdjacentTriangle(triangle, edge);
                if (adjacentTriangle != null)
                {
                    var circumcenter2 = CalculateCircumcenter(adjacentTriangle.A, adjacentTriangle.B, adjacentTriangle.C);
                    CreateVoronoiEdge(circumcenter1, circumcenter2);
                }
            }
        }
    }

    Vector2 CalculateCircumcenter(Vector2 a, Vector2 b, Vector2 c)
    {
        float D = 2 * (a.x * (b.y - c.y) + b.x * (c.y - a.y) + c.x * (a.y - b.y));
        if (Mathf.Abs(D) < Mathf.Epsilon)
        {
            UnityEngine.Debug.LogError("Les points sont colinéaires, impossible de calculer le centre circonscrit.");
            return Vector2.zero;
        }

        float Ux = ((a.sqrMagnitude * (b.y - c.y)) + (b.sqrMagnitude * (c.y - a.y)) + (c.sqrMagnitude * (a.y - b.y))) / D;
        float Uy = ((a.sqrMagnitude * (c.x - b.x)) + (b.sqrMagnitude * (a.x - c.x)) + (c.sqrMagnitude * (b.x - a.x))) / D;
        return new Vector2(Ux, Uy);
    }

    void CreateVoronoiEdge(Vector2 start, Vector2 end)
    {
        GameObject voronoiEdge = new GameObject("VoronoiEdge");
        voronoiEdge.transform.parent = voronoiParent.transform;
        LineRenderer lineRenderer = voronoiEdge.AddComponent<LineRenderer>();

        lineRenderer.positionCount = 2;
        lineRenderer.startWidth = 0.02f;
        lineRenderer.endWidth = 0.02f;
        lineRenderer.material = new Material(Shader.Find("Sprites/Default"));
        lineRenderer.startColor = colorVoronoi;
        lineRenderer.endColor = colorVoronoi;

        lineRenderer.SetPosition(0, new Vector3(start.x, start.y, 0));
        lineRenderer.SetPosition(1, new Vector3(end.x, end.y, 0));
    }





}

public class Triangle
{
    public Vector2 A, B, C;

    public Triangle(Vector2 a, Vector2 b, Vector2 c)
    {
        A = a;
        B = b;
        C = c;
    }

    public bool IsPointInCircumcircle(Vector2 point)
    {
        float ax = A.x - point.x;
        float ay = A.y - point.y;
        float bx = B.x - point.x;
        float by = B.y - point.y;
        float cx = C.x - point.x;
        float cy = C.y - point.y;

        float det = ax * (by * cy - cx * by) - ay * (bx * cy - cx * bx) + (ax * bx - ay * by) * cx;
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
