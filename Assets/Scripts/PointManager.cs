using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.Linq;
using System.Diagnostics;

public class PointManager : MonoBehaviour
{
    public GameObject pointPrefab; // Prefab pour représenter un point visuellement
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
            MeasureExecutionTime(JarvisMarch, "Jarvis March");
        }
        if (Input.GetKeyDown(KeyCode.G)) // Touche G pour Graham Scan
        {
            MeasureExecutionTime(GrahamScan, "Graham Scan");
        }
        if (Input.GetKeyDown(KeyCode.T)) // Touche T pour triangulation incrémentale
        {
            TriangulationIncrementale();
        }
        if (Input.GetKeyDown(KeyCode.D)) // Touche D pour Delaunay
        {
            TriangulationDelaunay();
        }
        if (Input.GetKeyDown(KeyCode.A)) // Ajouter un point avec Delaunay
        {
            AddPointDelaunay(new Vector2(Random.Range(-5f, 5f), Random.Range(-5f, 5f)));
        }

        if (Input.GetKeyDown(KeyCode.R)) // Supprimer un point avec Delaunay
        {
            if (points.Count > 0) RemovePointDelaunay(points[Random.Range(0, points.Count)]);
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

    public void JarvisMarch()
    {
        if (points.Count < 3) return;
        List<Vector2> hull = new List<Vector2>();

        // Trouver le point le plus à gauche
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

    public void GrahamScan()
    {
        if (points.Count < 3) return;

        // Étape 1 : Trouver le point le plus bas
        Vector2 pivot = points.OrderBy(p => p.y).ThenBy(p => p.x).First();

        // Étape 2 : Trier les points par angle polaire
        List<Vector2> sortedPoints = points.OrderBy(p => Mathf.Atan2(p.y - pivot.y, p.x - pivot.x)).ToList();

        // Étape 3 : Construire l'enveloppe convexe
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

    void TriangulationIncrementale()
    {
        if (points.Count < 3)
        {
            UnityEngine.Debug.Log("Pas assez de points pour effectuer la triangulation incrémentale.");
            return;
        }

        triangles.Clear();
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

        foreach (Triangle triangle in triangles)
        {
            if (triangle.IsPointInCircumcircle(newPoint))
            {
                badTriangles.Add(triangle);
            }
        }

        List<Edge> polygon = GetPolygonEdges(badTriangles);

        foreach (Triangle badTriangle in badTriangles)
        {
            triangles.Remove(badTriangle);
        }

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
            UnityEngine.Debug.Log("Aucun triangle pour effectuer la triangulation de Delaunay.");
            return;
        }

        bool flipped;
        int iteration = 0; // Pour éviter une boucle infinie
        do
        {
            flipped = false;
            iteration++;
            UnityEngine.Debug.Log($"Delaunay Iteration: {iteration}");

            foreach (var triangle in triangles.ToList())
            {
                foreach (var edge in triangle.GetEdges())
                {
                    var adjacentTriangle = FindAdjacentTriangle(triangle, edge);
                    if (adjacentTriangle != null && !IsDelaunay(edge, triangle, adjacentTriangle))
                    {
                        UnityEngine.Debug.Log($"Flipping edge between {edge.A} and {edge.B}");
                        FlipEdge(edge, triangle, adjacentTriangle);
                        flipped = true;
                    }
                }
            }

            if (iteration > 100) // Limite pour éviter les boucles infinies
            {
                UnityEngine.Debug.LogError("Triangulation Delaunay bloquée dans une boucle infinie.");
                break;
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
        bool isInCircumcircle = t1.IsPointInCircumcircle(opposite);

        if (isInCircumcircle)
        {
            UnityEngine.Debug.Log($"Point {opposite} est dans le cercle circonscrit de {t1.A}, {t1.B}, {t1.C}");
        }

        return !isInCircumcircle;
    }


    void FlipEdge(Edge edge, Triangle t1, Triangle t2)
    {
        Vector2 a = edge.A;
        Vector2 b = edge.B;
        Vector2 c = t1.GetOppositePoint(edge);
        Vector2 d = t2.GetOppositePoint(edge);

        // Supprimez les triangles existants
        if (triangles.Contains(t1)) triangles.Remove(t1);
        if (triangles.Contains(t2)) triangles.Remove(t2);

        // Ajoutez les nouveaux triangles
        triangles.Add(new Triangle(a, c, d));
        triangles.Add(new Triangle(b, c, d));

        UnityEngine.Debug.Log($"Edge flipped: {a}-{b} replaced by {c}-{d}");
    }


    void DrawTriangles()
    {
        foreach (Triangle triangle in triangles)
        {
            CreateTriangleVisualization(triangle);
        }
    }

    void CreateTriangleVisualization(Triangle triangle)
    {
        GameObject triangleObject = new GameObject("Triangle");
        LineRenderer lineRenderer = triangleObject.AddComponent<LineRenderer>();

        lineRenderer.positionCount = 4; // 3 sommets + retour au premier point
        lineRenderer.startWidth = 0.05f;
        lineRenderer.endWidth = 0.05f;
        lineRenderer.useWorldSpace = true;

        lineRenderer.SetPosition(0, new Vector3(triangle.A.x, triangle.A.y, 0));
        lineRenderer.SetPosition(1, new Vector3(triangle.B.x, triangle.B.y, 0));
        lineRenderer.SetPosition(2, new Vector3(triangle.C.x, triangle.C.y, 0));
        lineRenderer.SetPosition(3, new Vector3(triangle.A.x, triangle.A.y, 0));

        lineRenderer.material = new Material(Shader.Find("Sprites/Default"));
        lineRenderer.startColor = Color.green;
        lineRenderer.endColor = Color.green;
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


    public void GenerateVoronoi()
    {
        // Nettoyer les anciennes visualisations
        foreach (var obj in GameObject.FindGameObjectsWithTag("VoronoiEdge"))
        {
            Destroy(obj);
        }

        // Calculer les points et arêtes de Voronoï
        List<Vector2> voronoiPoints = new List<Vector2>();
        foreach (var triangle in triangles)
        {
            Vector2 circumcenter = CalculateCircumcenter(triangle.A, triangle.B, triangle.C);
            voronoiPoints.Add(circumcenter);
        }

        Rect bounds = new Rect(-5, -5, 10, 10); // Limite les arêtes à une boîte englobante
        foreach (var triangle in triangles)
        {
            Vector2 center1 = CalculateCircumcenter(triangle.A, triangle.B, triangle.C);

            foreach (var edge in triangle.GetEdges())
            {
                var adjacentTriangle = FindAdjacentTriangle(triangle, edge);
                if (adjacentTriangle != null)
                {
                    Vector2 center2 = CalculateCircumcenter(adjacentTriangle.A, adjacentTriangle.B, adjacentTriangle.C);
                    if (bounds.Contains(center1) && bounds.Contains(center2))
                    {
                        CreateVoronoiEdge(center1, center2);
                    }
                }
            }
        }
    }

    void CreateVoronoiEdge(Vector2 start, Vector2 end)
    {
        GameObject voronoiEdge = new GameObject("VoronoiEdge");
        voronoiEdge.tag = "VoronoiEdge";
        LineRenderer lineRenderer = voronoiEdge.AddComponent<LineRenderer>();

        lineRenderer.positionCount = 2;
        lineRenderer.startWidth = 0.05f;
        lineRenderer.endWidth = 0.05f;
        lineRenderer.useWorldSpace = true;

        lineRenderer.SetPosition(0, new Vector3(start.x, start.y, 0));
        lineRenderer.SetPosition(1, new Vector3(end.x, end.y, 0));

        lineRenderer.material = new Material(Shader.Find("Sprites/Default"));
        lineRenderer.startColor = Color.red;
        lineRenderer.endColor = Color.red;
    }



    Vector2 CalculateCircumcenter(Vector2 a, Vector2 b, Vector2 c)
    {
        float D = 2 * (a.x * (b.y - c.y) + b.x * (c.y - a.y) + c.x * (a.y - b.y));
        float Ux = ((a.sqrMagnitude * (b.y - c.y)) + (b.sqrMagnitude * (c.y - a.y)) + (c.sqrMagnitude * (a.y - b.y))) / D;
        float Uy = ((a.sqrMagnitude * (c.x - b.x)) + (b.sqrMagnitude * (a.x - c.x)) + (c.sqrMagnitude * (b.x - a.x))) / D;
        return new Vector2(Ux, Uy);
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

        float det = ax * (by * cy - cy * cy) - ay * (bx * cy - cx * bx) + (ax * bx - ay * by) * cx;
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
