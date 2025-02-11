using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.Linq;
using System.Diagnostics;
using System;


public class PointManager : MonoBehaviour
{
    public GameObject pointPrefab; // Prefab pour représenter un point visuellement
    public LineRenderer lineRenderer; // Pour tracer l'enveloppe convexe
    private GameObject voronoiParent; // Pour contenir toutes les arêtes de Voronoï

    public Color colorJarvis = Color.blue;
    public Color colorGraham = Color.green;
    public Color colorIncremental = Color.cyan;
    public Color colorDelaunay = Color.yellow;
    public Color colorVoronoi = Color.red;

    private List<Vector2> points = new List<Vector2>();
    private List<GameObject> pointObjects = new List<GameObject>();
    private List<Triangle> triangles = new List<Triangle>();

    private GrapheIncidence2D graphe = new GrapheIncidence2D();

    private bool afficherCouleursOrientation = false;


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
        /*if (Input.GetKeyDown(KeyCode.D)) // Triangulation de Delaunay
        {
            MeasureExecutionTime(TriangulationDelaunay, "Triangulation de Delaunay");
        }*/
        /*if (Input.GetKeyDown(KeyCode.A)) // Ajouter un point avec Delaunay
        {
            MeasureExecutionTime(() => AddPointDelaunay(new Vector2(UnityEngine.Random.Range(-5f, 5f), UnityEngine.Random.Range(-5f, 5f))), "Ajout de Point pour Delaunay");
        }*/

        if (Input.GetKeyDown(KeyCode.V)) // Générer le diagramme de Voronoï
        {
            MeasureExecutionTime(GenerateVoronoi, "Génération du Diagramme de Voronoï");
        }

        if (Input.GetKeyDown(KeyCode.C))  // ou une autre touche de ton choix
        {
            CompareAlgorithms();  // Cela va comparer Jarvis et Graham
        }

        if (Input.GetKeyDown(KeyCode.I)) // Touche I pour Information/Incidence
        {
            DebugGrapheIncidence();
        }

        if (Input.GetKeyDown(KeyCode.R)) // Reset
        {
            ClearAll();
            if (points.Count > 0)
            {
                MeasureExecutionTime(() => RemovePointDelaunay(points[UnityEngine.Random.Range(0, points.Count)]), "Suppression de Point pour Delaunay");
            }
        }

        /*if (Input.GetKeyDown(KeyCode.O)) // Appuyer sur O pour activer/désactiver l'affichage des couleurs
        {
            afficherCouleursOrientation = !afficherCouleursOrientation;
            UnityEngine.Debug.Log("Affichage des couleurs d'orientation : " + (afficherCouleursOrientation ? "Activé" : "Désactivé"));

            if (afficherCouleursOrientation)
            {
                ColorerTriangles(); // Appliquer la couleur
            }
            else
            {
                ReinitialiserCouleurTriangles(); // Réinitialiser les couleurs normales
            }
        }*/

        /*if (Input.GetKeyDown(KeyCode.O)) // Appuyer sur O pour activer/désactiver
        {
            afficherCouleursOrientation = !afficherCouleursOrientation;
            MettreAJourAffichageOrientation();
            UnityEngine.Debug.Log("Mode d'affichage des orientations : " + (afficherCouleursOrientation ? "Activé" : "Désactivé"));
        }*/

        if (Input.GetKeyDown(KeyCode.O))
        {
            afficherCouleursOrientation = !afficherCouleursOrientation;
            MettreAJourAffichageOrientation();
            UnityEngine.Debug.Log("Mode d'affichage des orientations : " + (afficherCouleursOrientation ? "Activé" : "Désactivé"));
        }




        MettreAJourCouleurPoints();

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

    private void ClearVisualization()
    {
        // Réinitialiser le LineRenderer
        if (lineRenderer != null)
        {
            lineRenderer.positionCount = 0;
        }

        // Effacer les triangles
        triangles.Clear();

        // Effacer les arêtes de Voronoi si elles existent
        if (voronoiParent != null)
        {
            Destroy(voronoiParent);
        }
    }

    private void ClearAll()
    {
        // Effacer les visualisations
        ClearVisualization();

        // Effacer tous les points
        foreach (var point in pointObjects)
        {
            Destroy(point);
        }
        pointObjects.Clear();
        points.Clear();

        // Détruire tous les objets Triangle dans la scène
        GameObject[] triangleObjects = GameObject.FindGameObjectsWithTag("Triangle");
        foreach (var triangleObj in triangleObjects)
        {
            Destroy(triangleObj);
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

    public void CompareAlgorithms()
    {
        // Mesurer Jarvis
        var jarvisTime = new Stopwatch();
        jarvisTime.Start();
        JarvisMarch();
        jarvisTime.Stop();

        // Mesurer Graham
        var grahamTime = new Stopwatch();
        grahamTime.Start();
        GrahamScan();
        grahamTime.Stop();

        // Afficher la comparaison
        UnityEngine.Debug.Log($"Jarvis March: {jarvisTime.ElapsedMilliseconds}ms");
        UnityEngine.Debug.Log($"Graham Scan: {grahamTime.ElapsedMilliseconds}ms");
        UnityEngine.Debug.Log($"Le plus efficace est : {(jarvisTime.ElapsedMilliseconds < grahamTime.ElapsedMilliseconds ? "Jarvis" : "Graham")}");
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
    /*void TriangulationIncrementale()
    {
        if (points.Count < 3)
        {
            UnityEngine.Debug.Log("Pas assez de points pour la triangulation incrémentale.");
            return;
        }

        try
        {
            ClearVisualization();
            triangles.Clear();

            // Créer le triangle initial
            triangles.Add(new Triangle(points[0], points[1], points[2]));

            // Ajouter les points suivants un par un
            for (int i = 3; i < points.Count; i++)
            {
                AddPointToTriangulation(points[i]);
            }

            DrawTriangles(colorIncremental);
            UnityEngine.Debug.Log($"Triangulation incrémentale terminée avec {triangles.Count} triangles.");
        }
        catch (Exception e)
        {
            UnityEngine.Debug.LogError($"Erreur lors de la triangulation incrémentale : {e.Message}");
        }
    }*/

    void TriangulationIncrementale()
    {
        if (points.Count < 3)
        {
            UnityEngine.Debug.Log("Pas assez de points pour la triangulation incrémentale.");
            return;
        }

        try
        {
            ClearVisualization();
            triangles.Clear();
            graphe = new GrapheIncidence2D(); // Réinitialiser le graphe d'incidence

            // Créer le triangle initial dans le sens trigonométrique
            Vector2 a = points[0];
            Vector2 b = points[1];
            Vector2 c = points[2];

            // Garantir l'orientation trigonométrique
            if (!IsCounterClockwise(a, b, c))
            {
                // Inverser l'ordre si besoin
                var temp = b;
                b = c;
                c = temp;
            }

            Triangle initialTriangle = new Triangle(a, b, c);
            triangles.Add(initialTriangle);

            // Ajouter l'arête initiale au graphe d'incidence
            graphe.AjouterArete(a, b, initialTriangle);
            graphe.AjouterArete(b, c, initialTriangle);
            graphe.AjouterArete(c, a, initialTriangle);

            // Ajouter les points suivants un par un
            for (int i = 3; i < points.Count; i++)
            {
                AddPointToTriangulation(points[i]);
            }

            DrawTriangles(colorIncremental);
            UnityEngine.Debug.Log($"Triangulation incrémentale terminée avec {triangles.Count} triangles.");
        }
        catch (Exception e)
        {
            UnityEngine.Debug.LogError($"Erreur lors de la triangulation incrémentale : {e.Message}");
        }

        CorrigerOrientationTriangles();
    }

    /*void AddPointToTriangulation(Vector2 newPoint)
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
    }*/

    void AddPointToTriangulation(Vector2 newPoint)
    {
        List<Triangle> badTriangles = new List<Triangle>();
        List<Edge> polygonEdges = new List<Edge>();

        // Optimisation : Rechercher d'abord les triangles contenant le point
        foreach (var triangle in triangles)
        {
            if (triangle.IsPointInCircumcircle(newPoint))
            {
                badTriangles.Add(triangle);
            }
        }

        // Identifier les bords du polygone de trou
        HashSet<Edge> boundaryEdges = new HashSet<Edge>();
        foreach (var badTriangle in badTriangles)
        {
            foreach (var edge in badTriangle.GetEdges())
            {
                // Ne conserver que les arêtes qui n'apparaissent qu'une seule fois
                bool isShared = badTriangles.Count(t => t.HasEdge(edge)) > 1;
                if (!isShared)
                {
                    boundaryEdges.Add(edge);
                }
            }
        }

        // Supprimer les triangles problématiques
        foreach (var badTriangle in badTriangles)
        {
            triangles.Remove(badTriangle);
        }

        // Créer de nouveaux triangles
        foreach (var edge in boundaryEdges)
        {
            // Créer un triangle en s'assurant de l'orientation trigonométrique
            Triangle newTriangle = IsCounterClockwise(edge.A, edge.B, newPoint)
                ? new Triangle(edge.A, edge.B, newPoint)
                : new Triangle(edge.B, edge.A, newPoint);

            triangles.Add(newTriangle);

            // Mettre à jour le graphe d'incidence
            graphe.AjouterArete(edge.A, edge.B, newTriangle);
            graphe.AjouterArete(edge.B, newPoint, newTriangle);
            graphe.AjouterArete(newPoint, edge.A, newTriangle);
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
    /*void TriangulationDelaunay()
    {
        if (triangles.Count == 0)
        {
            UnityEngine.Debug.Log("Aucun triangle à traiter pour Delaunay.");
            return;
        }

        try
        {
            bool flipped;
            int iterationLimit = 1000;
            int iteration = 0;
            int flipCount = 0;

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
                            flipCount++;
                        }
                    }
                }

                if (iteration > iterationLimit)
                {
                    UnityEngine.Debug.LogWarning("Triangulation de Delaunay : limite d'itérations atteinte.");
                    break;
                }
            } while (flipped);

            DrawTriangles(colorDelaunay);
            UnityEngine.Debug.Log($"Triangulation de Delaunay terminée en {iteration} itérations. {flipCount} arêtes flippées.");
        }
        catch (Exception e)
        {
            UnityEngine.Debug.LogError($"Erreur lors de la triangulation de Delaunay : {e.Message}");
        }
    }*/

    void TriangulationDelaunay()
    {
        if (triangles.Count < 2) return;

        bool flipped;
        int iterationLimit = 1000;
        int iteration = 0;

        do
        {
            flipped = false;
            iteration++;

            var currentTriangles = new List<Triangle>(triangles);

            foreach (var triangle in currentTriangles)
            {
                foreach (var edge in triangle.GetEdges())
                {
                    var adjacentTriangle = FindAdjacentTriangle(triangle, edge);

                    if (adjacentTriangle != null)
                    {
                        if (!IsDelaunay(edge, triangle, adjacentTriangle))
                        {
                            FlipEdge(edge, triangle, adjacentTriangle);
                            flipped = true;
                        }
                    }
                }
            }

            if (iteration > iterationLimit)
            {
                UnityEngine.Debug.LogWarning("Triangulation de Delaunay : limite d'itérations atteinte");
                break;
            }
        } while (flipped);

        DrawTriangles(colorDelaunay);
    }

    private List<Triangle> SubdivideTriangle(Triangle triangle, Vector2 point)
    {
        List<Triangle> newTriangles = new List<Triangle>();
        // Créer trois nouveaux triangles en connectant le point avec chaque sommet
        newTriangles.Add(new Triangle(triangle.A, triangle.B, point));
        newTriangles.Add(new Triangle(triangle.B, triangle.C, point));
        newTriangles.Add(new Triangle(triangle.C, triangle.A, point));

        // Supprimer l'ancien triangle
        triangles.Remove(triangle);
        // Ajouter les nouveaux triangles
        triangles.AddRange(newTriangles);

        return newTriangles;
    }

    private List<Triangle> FindAdjacentTrianglesForEdge(Edge edge)
    {
        List<Triangle> adjacentTriangles = new List<Triangle>();
        foreach (var triangle in triangles)
        {
            if (triangle.HasEdge(edge))
            {
                adjacentTriangles.Add(triangle);
            }
            if (adjacentTriangles.Count == 2) break; // On a trouvé les deux triangles
        }
        return adjacentTriangles;
    }

    public void AddPointDelaunay(Vector2 newPoint)
    {
        // Trouver le triangle contenant le nouveau point
        Triangle containingTriangle = FindContainingTriangle(newPoint);

        if (containingTriangle == null)
        {
            UnityEngine.Debug.LogWarning("Aucun triangle ne contient le point.");
            return;
        }

        // Subdiviser ce triangle en trois
        List<Triangle> newTriangles = SubdivideTriangle(containingTriangle, newPoint);

        // Appliquer le flipping localement
        Queue<Edge> edgesToCheck = new Queue<Edge>();
        foreach (var triangle in newTriangles)
        {
            foreach (var edge in triangle.GetEdges())
            {
                edgesToCheck.Enqueue(edge);
            }
        }

        while (edgesToCheck.Count > 0)
        {
            Edge edge = edgesToCheck.Dequeue();
            var adjacentTriangles = FindAdjacentTrianglesForEdge(edge);

            if (adjacentTriangles.Count == 2 &&
                !IsDelaunay(edge, adjacentTriangles[0], adjacentTriangles[1]))
            {
                FlipEdge(edge, adjacentTriangles[0], adjacentTriangles[1]);

                // Ajouter les nouvelles arêtes à vérifier
                foreach (var newTriangle in adjacentTriangles)
                {
                    foreach (var newEdge in newTriangle.GetEdges())
                    {
                        edgesToCheck.Enqueue(newEdge);
                    }
                }
            }
        }
    }

    Triangle FindAdjacentTriangle(Triangle triangle, Edge edge)
    {
        // Rechercher un triangle différent de 'triangle' qui partage exactement cette arête
        return triangles.FirstOrDefault(t =>
            t != triangle &&
            t.HasEdge(edge)
        );
    }


    private void CleanupVoronoiEdges()
    {
        // Détruire l'ancien parent s'il existe
        if (voronoiParent != null)
        {
            Destroy(voronoiParent);
        }
        // Créer un nouveau parent
        voronoiParent = new GameObject("VoronoiEdges");
    }

    /*private void AddBoundingPoints(List<Vector2> points, Vector2[] boundingBox, float margin)
    {
        float minX = boundingBox[0].x - margin;
        float maxX = boundingBox[1].x + margin;
        float minY = boundingBox[0].y - margin;
        float maxY = boundingBox[2].y + margin;

        // Ajouter des points aux coins du rectangle englobant élargi
        points.Add(new Vector2(minX, minY));
        points.Add(new Vector2(maxX, minY));
        points.Add(new Vector2(maxX, maxY));
        points.Add(new Vector2(minX, maxY));
    }*/

    /*private List<Triangle> GenerateDelaunayTriangulation(List<Vector2> points)
    {
        // Créer une triangulation initiale avec un super-triangle
        List<Triangle> delaunayTriangles = new List<Triangle>();

        // Calculer les dimensions du super-triangle
        float minX = points.Min(p => p.x);
        float maxX = points.Max(p => p.x);
        float minY = points.Min(p => p.y);
        float maxY = points.Max(p => p.y);

        float dx = (maxX - minX) * 2;
        float dy = (maxY - minY) * 2;

        Vector2 p1 = new Vector2(minX - dx, minY - dy);
        Vector2 p2 = new Vector2(maxX + dx * 2, minY - dy);
        Vector2 p3 = new Vector2(minX + dx / 2, maxY + dy * 2);

        delaunayTriangles.Add(new Triangle(p1, p2, p3));

        // Ajouter les points un par un
        foreach (var point in points)
        {
            List<Triangle> badTriangles = new List<Triangle>();

            // Trouver tous les triangles dont le cercle circonscrit contient le point
            foreach (var triangle in delaunayTriangles)
            {
                if (triangle.IsPointInCircumcircle(point))
                {
                    badTriangles.Add(triangle);
                }
            }

            // Trouver le polygone de trou
            HashSet<Edge> boundary = new HashSet<Edge>();
            foreach (var triangle in badTriangles)
            {
                foreach (var edge in triangle.GetEdges())
                {
                    bool isShared = badTriangles.Count(t => t.HasEdge(edge)) > 1;
                    if (!isShared)
                    {
                        boundary.Add(edge);
                    }
                }
            }

            // Supprimer les mauvais triangles
            foreach (var triangle in badTriangles)
            {
                delaunayTriangles.Remove(triangle);
            }

            // Retrianguler le trou avec le nouveau point
            foreach (var edge in boundary)
            {
                delaunayTriangles.Add(new Triangle(edge.A, edge.B, point));
            }
        }

        // Supprimer les triangles qui partagent un sommet avec le super-triangle
        delaunayTriangles.RemoveAll(t =>
            t.SharesVertex(p1) || t.SharesVertex(p2) || t.SharesVertex(p3));

        return delaunayTriangles;
    }*/


    bool IsDelaunay(Edge edge, Triangle t1, Triangle t2)
    {
        Vector2 oppositeInT1 = t1.GetOppositePoint(edge);
        Vector2 oppositeInT2 = t2.GetOppositePoint(edge);

        return !t1.IsPointInCircumcircle(oppositeInT2) &&
               !t2.IsPointInCircumcircle(oppositeInT1);
    }

    /*bool IsDelaunay(Edge edge, Triangle t1, Triangle t2)
    {
        var opposite = t2.GetOppositePoint(edge);
        return !t1.IsPointInCircumcircle(opposite);
    }*/

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
        triangleObject.tag = "Triangle";
        LineRenderer lineRenderer = triangleObject.AddComponent<LineRenderer>();

        // Création des lignes du triangle
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

        // ✅ Toujours créer le MeshRenderer mais l'activer/désactiver après
        MeshFilter meshFilter = triangleObject.AddComponent<MeshFilter>();
        MeshRenderer meshRenderer = triangleObject.AddComponent<MeshRenderer>();

        Mesh mesh = new Mesh();
        mesh.vertices = new Vector3[]
        {
        new Vector3(triangle.A.x, triangle.A.y, 0),
        new Vector3(triangle.B.x, triangle.B.y, 0),
        new Vector3(triangle.C.x, triangle.C.y, 0)
        };

        mesh.triangles = new int[] { 0, 1, 2 };
        mesh.RecalculateNormals();

        meshFilter.mesh = mesh;

        // Matériau temporaire
        meshRenderer.material = new Material(Shader.Find("Standard"));
        meshRenderer.material.color = lineColor;

        // 🛑 Désactiver le MeshRenderer si afficherCouleursOrientation est désactivé
        meshRenderer.enabled = afficherCouleursOrientation;
    }


    /*void CreateTriangleVisualization(Triangle triangle, Color lineColor)
    {
        GameObject triangleObject = new GameObject("Triangle");
        triangleObject.tag = "Triangle";
        LineRenderer lineRenderer = triangleObject.AddComponent<LineRenderer>();

        MeshFilter meshFilter = triangleObject.AddComponent<MeshFilter>();
        MeshRenderer meshRenderer = triangleObject.AddComponent<MeshRenderer>();

        Mesh mesh = new Mesh();
        mesh.vertices = new Vector3[]
        {
        new Vector3(triangle.A.x, triangle.A.y, 0),
        new Vector3(triangle.B.x, triangle.B.y, 0),
        new Vector3(triangle.C.x, triangle.C.y, 0)
        };

        mesh.triangles = new int[] { 0, 1, 2 };
        mesh.RecalculateNormals();

        meshFilter.mesh = mesh;

        // Matériau temporaire
        meshRenderer.material = new Material(Shader.Find("Standard"));
        meshRenderer.material.color = lineColor;

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


    }*/

    /*public void AddPointDelaunay(Vector2 newPoint)
    {
        // Trouver le triangle contenant le nouveau point
        Triangle containingTriangle = FindContainingTriangle(newPoint);

        // Subdiviser ce triangle en trois
        List<Triangle> newTriangles = SubdivideTriangle(containingTriangle, newPoint);

        // Appliquer le flipping localement
        Queue<Edge> edgesToCheck = new Queue<Edge>();
        foreach (var triangle in newTriangles)
        {
            foreach (var edge in triangle.GetEdges())
            {
                edgesToCheck.Enqueue(edge);
            }
        }

        while (edgesToCheck.Count > 0)
        {
            Edge edge = edgesToCheck.Dequeue();
            var adjacentTriangles = FindAdjacentTrianglesForEdge(edge);
            if (adjacentTriangles.Count == 2 && !IsDelaunay(edge, adjacentTriangles[0], adjacentTriangles[1]))
            {
                FlipEdge(edge, adjacentTriangles[0], adjacentTriangles[1]);
                // Ajouter les nouvelles arêtes à vérifier
                foreach (var newEdge in adjacentTriangles[0].GetEdges().Concat(adjacentTriangles[1].GetEdges()))
                {
                    edgesToCheck.Enqueue(newEdge);
                }
            }
        }
    }*/

    private Triangle FindContainingTriangle(Vector2 point)
    {
        return triangles.FirstOrDefault(t => PointInTriangle(point, t.A, t.B, t.C));
    }

    private bool PointInTriangle(Vector2 p, Vector2 a, Vector2 b, Vector2 c)
    {
        float d1 = Sign(p, a, b);
        float d2 = Sign(p, b, c);
        float d3 = Sign(p, c, a);

        bool hasNeg = (d1 < 0) || (d2 < 0) || (d3 < 0);
        bool hasPos = (d1 > 0) || (d2 > 0) || (d3 > 0);

        return !(hasNeg && hasPos);
    }

    private float Sign(Vector2 p1, Vector2 p2, Vector2 p3)
    {
        return (p1.x - p3.x) * (p2.y - p3.y) - (p2.x - p3.x) * (p1.y - p3.y);
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
    public void GenerateVoronoi()
    {
        CleanupVoronoiEdges();
        if (triangles.Count == 0) return;

        // Calculer la boîte englobante avec une marge plus grande
        Vector2[] boundingBox = GetExtendedBoundingBox(2f); // Facteur d'extension de 2x

        // Stocker les arêtes de Voronoï déjà traitées pour éviter les doublons
        HashSet<VoronoiEdge> processedEdges = new HashSet<VoronoiEdge>();

        // Pour chaque triangle de Delaunay
        foreach (var triangle in triangles)
        {
            var cc1 = CalculateCircumcenterPrecise(triangle.A, triangle.B, triangle.C);

            foreach (var edge in triangle.GetEdges())
            {
                var adjacentTriangle = FindAdjacentTriangle(triangle, edge);
                if (adjacentTriangle != null)
                {
                    var cc2 = CalculateCircumcenterPrecise(
                        adjacentTriangle.A,
                        adjacentTriangle.B,
                        adjacentTriangle.C
                    );

                    var voronoiEdge = new VoronoiEdge(cc1, cc2);
                    if (!processedEdges.Contains(voronoiEdge))
                    {
                        processedEdges.Add(voronoiEdge);

                        // Prolonger l'arête jusqu'aux limites si nécessaire
                        var (start, end) = ExtendVoronoiEdge(cc1, cc2, boundingBox);
                        if (start != Vector2.zero || end != Vector2.zero) // Vérifier si l'extension est valide
                        {
                            CreateVoronoiEdge(start, end);
                        }
                    }
                }
                else
                {
                    // Pour les arêtes sur l'enveloppe convexe
                    Vector2 edgeMidpoint = (edge.A + edge.B) * 0.5f;
                    Vector2 perpendicular = new Vector2(-(edge.B.y - edge.A.y), edge.B.x - edge.A.x).normalized;

                    // Prolonger dans la direction perpendiculaire à l'arête
                    var boundaryPoint = ExtendRayToBox(cc1, perpendicular, boundingBox);
                    if (boundaryPoint != Vector2.zero)
                    {
                        CreateVoronoiEdge(cc1, boundaryPoint);
                    }
                }
            }
        }
    }

    private Vector2[] GetExtendedBoundingBox(float extensionFactor)
    {
        if (points.Count == 0) return new Vector2[4];

        float minX = points.Min(p => p.x);
        float maxX = points.Max(p => p.x);
        float minY = points.Min(p => p.y);
        float maxY = points.Max(p => p.y);

        float centerX = (minX + maxX) * 0.5f;
        float centerY = (minY + maxY) * 0.5f;

        float width = (maxX - minX) * extensionFactor;
        float height = (maxY - minY) * extensionFactor;

        float halfWidth = width * 0.5f;
        float halfHeight = height * 0.5f;

        return new Vector2[] {
        new Vector2(centerX - halfWidth, centerY - halfHeight),
        new Vector2(centerX + halfWidth, centerY - halfHeight),
        new Vector2(centerX + halfWidth, centerY + halfHeight),
        new Vector2(centerX - halfWidth, centerY + halfHeight)
    };
    }

    private Vector2 CalculateCircumcenterPrecise(Vector2 a, Vector2 b, Vector2 c)
    {
        // Utiliser des doubles pour plus de précision
        double d = 2.0 * ((a.x * (b.y - c.y)) + (b.x * (c.y - a.y)) + (c.x * (a.y - b.y)));

        if (Mathf.Abs((float)d) < 1e-10)
        {
            // Gérer le cas dégénéré
            return (a + b + c) / 3f;
        }

        double aSq = a.x * a.x + a.y * a.y;
        double bSq = b.x * b.x + b.y * b.y;
        double cSq = c.x * c.x + c.y * c.y;

        double x = (aSq * (b.y - c.y) + bSq * (c.y - a.y) + cSq * (a.y - b.y)) / d;
        double y = (aSq * (c.x - b.x) + bSq * (a.x - c.x) + cSq * (b.x - a.x)) / d;

        return new Vector2((float)x, (float)y);
    }

    private (Vector2, Vector2) ExtendVoronoiEdge(Vector2 start, Vector2 end, Vector2[] boundingBox)
    {
        Vector2 direction = (end - start).normalized;
        float maxDistance = Vector2.Distance(boundingBox[0], boundingBox[2]) * 2;

        Vector2 extendedStart = start - direction * maxDistance;
        Vector2 extendedEnd = end + direction * maxDistance;

        return ClipLineToBox(extendedStart, extendedEnd, boundingBox);
    }

    private (Vector2, Vector2) ClipLineToBox(Vector2 start, Vector2 end, Vector2[] boundingBox)
    {
        float minX = boundingBox[0].x;
        float maxX = boundingBox[2].x;
        float minY = boundingBox[0].y;
        float maxY = boundingBox[2].y;

        float x1 = start.x;
        float y1 = start.y;
        float x2 = end.x;
        float y2 = end.y;

        // Utiliser l'algorithme de Cohen-Sutherland pour le clipping
        int code1 = ComputeOutCode(x1, y1, minX, maxX, minY, maxY);
        int code2 = ComputeOutCode(x2, y2, minX, maxX, minY, maxY);

        bool accept = false;

        while (true)
        {
            if ((code1 | code2) == 0)
            {
                accept = true;
                break;
            }
            else if ((code1 & code2) != 0)
            {
                break;
            }
            else
            {
                float x = 0, y = 0;
                int codeOut = code1 != 0 ? code1 : code2;

                if ((codeOut & 8) != 0)
                {
                    x = x1 + (x2 - x1) * (maxY - y1) / (y2 - y1);
                    y = maxY;
                }
                else if ((codeOut & 4) != 0)
                {
                    x = x1 + (x2 - x1) * (minY - y1) / (y2 - y1);
                    y = minY;
                }
                else if ((codeOut & 2) != 0)
                {
                    y = y1 + (y2 - y1) * (maxX - x1) / (x2 - x1);
                    x = maxX;
                }
                else if ((codeOut & 1) != 0)
                {
                    y = y1 + (y2 - y1) * (minX - x1) / (x2 - x1);
                    x = minX;
                }

                if (codeOut == code1)
                {
                    x1 = x;
                    y1 = y;
                    code1 = ComputeOutCode(x1, y1, minX, maxX, minY, maxY);
                }
                else
                {
                    x2 = x;
                    y2 = y;
                    code2 = ComputeOutCode(x2, y2, minX, maxX, minY, maxY);
                }
            }
        }

        if (accept)
        {
            return (new Vector2(x1, y1), new Vector2(x2, y2));
        }
        return (Vector2.zero, Vector2.zero);
    }

    private int ComputeOutCode(float x, float y, float minX, float maxX, float minY, float maxY)
    {
        int code = 0;
        if (y > maxY) code |= 8;
        if (y < minY) code |= 4;
        if (x > maxX) code |= 2;
        if (x < minX) code |= 1;
        return code;
    }

    private class VoronoiEdge
    {
        public Vector2 Start;
        public Vector2 End;

        public VoronoiEdge(Vector2 start, Vector2 end)
        {
            Start = start;
            End = end;
        }

        public override bool Equals(object obj)
        {
            if (!(obj is VoronoiEdge)) return false;
            VoronoiEdge other = (VoronoiEdge)obj;
            return (Start == other.Start && End == other.End) ||
                   (Start == other.End && End == other.Start);
        }

        public override int GetHashCode()
        {
            return Start.GetHashCode() ^ End.GetHashCode();
        }
    }

    /*private Vector2[] GetBoundingBox()
    {
        if (points.Count == 0) return new Vector2[4];

        float minX = points.Min(p => p.x);
        float maxX = points.Max(p => p.x);
        float minY = points.Min(p => p.y);
        float maxY = points.Max(p => p.y);

        // Calculer la marge en fonction de la taille du nuage de points
        float width = maxX - minX;
        float height = maxY - minY;
        float margin = Mathf.Max(width, height) * 0.2f; // 20% de la plus grande dimension

        return new Vector2[] {
        new Vector2(minX - margin, minY - margin),
        new Vector2(maxX + margin, minY - margin),
        new Vector2(maxX + margin, maxY + margin),
        new Vector2(minX - margin, maxY + margin)
    };
    }*/

    /*private bool IsEdgeInBounds(Vector2 start, Vector2 end, Vector2[] boundingBox)
    {
        float minX = boundingBox[0].x;
        float maxX = boundingBox[2].x;
        float minY = boundingBox[0].y;
        float maxY = boundingBox[2].y;

        // Vérifier si au moins une partie de l'arête est dans les limites
        bool startInBounds = start.x >= minX && start.x <= maxX && start.y >= minY && start.y <= maxY;
        bool endInBounds = end.x >= minX && end.x <= maxX && end.y >= minY && end.y <= maxY;

        // Si les deux points sont hors limites, vérifier si l'arête traverse la boîte
        if (!startInBounds && !endInBounds)
        {
            return LineIntersectsBox(start, end, boundingBox);
        }

        return startInBounds || endInBounds;
    }*/

    private bool LineIntersectsBox(Vector2 start, Vector2 end, Vector2[] boundingBox)
    {
        // Vérifier l'intersection avec chaque côté de la boîte
        for (int i = 0; i < 4; i++)
        {
            Vector2 boxStart = boundingBox[i];
            Vector2 boxEnd = boundingBox[(i + 1) % 4];

            if (LinesIntersect(start, end, boxStart, boxEnd))
            {
                return true;
            }
        }
        return false;
    }

    private bool LinesIntersect(Vector2 a1, Vector2 a2, Vector2 b1, Vector2 b2)
    {
        Vector2 b = a2 - a1;
        Vector2 d = b2 - b1;
        float bDotDPerp = b.x * d.y - b.y * d.x;

        // Si les lignes sont parallèles
        if (bDotDPerp == 0)
        {
            return false;
        }

        Vector2 c = b1 - a1;
        float t = (c.x * d.y - c.y * d.x) / bDotDPerp;
        if (t < 0 || t > 1)
        {
            return false;
        }

        float u = (c.x * b.y - c.y * b.x) / bDotDPerp;
        if (u < 0 || u > 1)
        {
            return false;
        }

        return true;
    }

    /*private Vector2 CalculateBoundaryPoint(Edge edge, Vector2 circumcenter, Vector2[] boundingBox)
    {
        // Calculer le point milieu de l'arête
        Vector2 midPoint = (edge.A + edge.B) * 0.5f;

        // Vecteur direction depuis le circumcentre vers le point milieu
        Vector2 direction = (midPoint - circumcenter).normalized;

        // Distance max pour être sûr d'atteindre la limite de la boîte
        float maxDist = Vector2.Distance(boundingBox[0], boundingBox[2]) * 2;

        return circumcenter + direction * maxDist;
    }*/

    private bool IsValidVoronoiEdge(Vector2 start, Vector2 end)
    {
        // Vérifier si l'arête n'est pas dégénérée
        if (Vector2.Distance(start, end) < 0.0001f)
            return false;

        // Vérifier si les points ne sont pas trop éloignés
        float maxDistance = 1000f; // Ajustez selon vos besoins
        if (Vector2.Distance(start, end) > maxDistance)
            return false;

        return true;
    }

    private Vector2 ExtendRayToBox(Vector2 start, Vector2 direction, Vector2[] boundingBox)
    {
        float minX = boundingBox[0].x;
        float maxX = boundingBox[2].x;
        float minY = boundingBox[0].y;
        float maxY = boundingBox[2].y;

        // Calculer les intersections possibles avec les bords de la boîte
        float tMinX = (minX - start.x) / direction.x;
        float tMaxX = (maxX - start.x) / direction.x;
        float tMinY = (minY - start.y) / direction.y;
        float tMaxY = (maxY - start.y) / direction.y;

        // Gérer le cas où direction.x ou direction.y est proche de zéro
        if (Mathf.Abs(direction.x) < 1e-10)
        {
            tMinX = float.NegativeInfinity;
            tMaxX = float.PositiveInfinity;
        }
        if (Mathf.Abs(direction.y) < 1e-10)
        {
            tMinY = float.NegativeInfinity;
            tMaxY = float.PositiveInfinity;
        }

        // Trouver les intersections valides
        float tMin = Mathf.Max(
            Mathf.Min(tMinX, tMaxX),
            Mathf.Min(tMinY, tMaxY)
        );
        float tMax = Mathf.Min(
            Mathf.Max(tMinX, tMaxX),
            Mathf.Max(tMinY, tMaxY)
        );

        // Si tMax est négatif, le rayon va dans la mauvaise direction
        // Si tMin > tMax, le rayon rate la boîte
        if (tMax < 0 || tMin > tMax)
        {
            return Vector2.zero;
        }

        // Utiliser tMax pour obtenir le point le plus éloigné qui est encore dans la boîte
        return start + direction * tMax;
    }


    private void CreateVoronoiEdge(Vector2 start, Vector2 end)
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


    Vector2 CalculateCircumcenter(Vector2 a, Vector2 b, Vector2 c)
    {
        float D = 2 * (a.x * (b.y - c.y) + b.x * (c.y - a.y) + c.x * (a.y - b.y));
        float Ux = ((a.sqrMagnitude * (b.y - c.y)) + (b.sqrMagnitude * (c.y - a.y)) + (c.sqrMagnitude * (a.y - b.y))) / D;
        float Uy = ((a.sqrMagnitude * (c.x - b.x)) + (b.sqrMagnitude * (a.x - c.x)) + (c.sqrMagnitude * (b.x - a.x))) / D;
        return new Vector2(Ux, Uy);
    }


    void DebugGrapheIncidence()
    {
        foreach (var sommet in graphe.adjacencySommetsAretes.Keys)
        {
            UnityEngine.Debug.Log($"Sommet {sommet} connecté à {graphe.adjacencySommetsAretes[sommet].Count} arêtes");
            UnityEngine.Debug.Log($"Triangles adjacents : {graphe.GetTrianglesAdjacents(sommet).Count}");
        }
    }

    void CorrigerOrientationTriangles()
    {
        int count = 0;
        for (int i = 0; i < triangles.Count; i++)
        {
            Triangle triangle = triangles[i];

            if (!IsCounterClockwise(triangle.A, triangle.B, triangle.C))
            {
                (triangle.B, triangle.C) = (triangle.C, triangle.B);
                count++;
            }
        }
        UnityEngine.Debug.Log($"Orientation corrigée pour {count} triangles !");
    }



    void MettreAJourCouleurPoints()
    {
        foreach (GameObject point in pointObjects)
        {
            Vector2 pos = point.transform.position;

            // Vérifier si l'objet a un MeshRenderer
            MeshRenderer renderer = point.GetComponent<MeshRenderer>();

            if (renderer != null)
            {
                // Changer la couleur en fonction de la position Y
                renderer.material.color = (pos.y >= 0) ? Color.green : Color.red;
            }
            else
            {
                UnityEngine.Debug.LogWarning($"L'objet {point.name} n'a pas de MeshRenderer !");
            }
        }
    }

    void ColorerTriangles()
    {
        if (!afficherCouleursOrientation) return;

        foreach (var triangle in triangles)
        {
            bool isCorrect = IsCounterClockwise(triangle.A, triangle.B, triangle.C);
            GameObject triangleObj = FindTriangleObject(triangle);

            if (triangleObj != null)
            {
                LineRenderer lineRenderer = triangleObj.GetComponent<LineRenderer>();

                if (lineRenderer != null)
                {
                    // Vert si l'orientation est correcte, Rouge sinon
                    lineRenderer.startColor = isCorrect ? Color.green : Color.red;
                    lineRenderer.endColor = isCorrect ? Color.green : Color.red;
                }
            }
        }
    }



    void ReinitialiserCouleurTriangles()
    {
        foreach (var triangle in triangles)
        {
            GameObject triangleObj = FindTriangleObject(triangle);

            if (triangleObj != null)
            {
                LineRenderer lineRenderer = triangleObj.GetComponent<LineRenderer>();

                if (lineRenderer != null)
                {
                    // Remettre la couleur d'origine (blanc par défaut)
                    lineRenderer.startColor = Color.white;
                    lineRenderer.endColor = Color.white;
                }
            }
        }
    }


    /*GameObject FindTriangleObject(Triangle triangle)
    {
        GameObject[] triangleObjects = GameObject.FindGameObjectsWithTag("Triangle");

        foreach (GameObject obj in triangleObjects)
        {
            LineRenderer lineRenderer = obj.GetComponent<LineRenderer>();

            if (lineRenderer != null && lineRenderer.positionCount >= 3)
            {
                Vector3 v0 = lineRenderer.GetPosition(0);
                Vector3 v1 = lineRenderer.GetPosition(1);
                Vector3 v2 = lineRenderer.GetPosition(2);

                // Vérifier si les positions correspondent au triangle donné (approximation)
                if (Vector2.Distance((Vector2)v0, triangle.A) < 0.01f &&
                    Vector2.Distance((Vector2)v1, triangle.B) < 0.01f &&
                    Vector2.Distance((Vector2)v2, triangle.C) < 0.01f)
                {
                    return obj;
                }
            }
        }

        return null;
    }*/

    GameObject FindTriangleObject(Triangle triangle)
    {
        GameObject[] triangleObjects = GameObject.FindGameObjectsWithTag("Triangle");

        foreach (GameObject obj in triangleObjects)
        {
            LineRenderer lineRenderer = obj.GetComponent<LineRenderer>();

            if (lineRenderer != null && lineRenderer.positionCount >= 3)
            {
                Vector3 v0 = lineRenderer.GetPosition(0);
                Vector3 v1 = lineRenderer.GetPosition(1);
                Vector3 v2 = lineRenderer.GetPosition(2);

                // Debug : Afficher les coordonnées trouvées
                UnityEngine.Debug.Log($"Checking TriangleObj: {v0}, {v1}, {v2}");

                if (Vector2.Distance((Vector2)v0, triangle.A) < 0.01f &&
                    Vector2.Distance((Vector2)v1, triangle.B) < 0.01f &&
                    Vector2.Distance((Vector2)v2, triangle.C) < 0.01f)
                {
                    return obj;
                }
            }
        }

        return null;
    }

    void MettreAJourAffichageOrientation()
    {
        GameObject[] triangleObjects = GameObject.FindGameObjectsWithTag("Triangle");

        foreach (GameObject obj in triangleObjects)
        {
            MeshRenderer meshRenderer = obj.GetComponent<MeshRenderer>();

            if (meshRenderer != null)
            {
                meshRenderer.enabled = afficherCouleursOrientation;
            }
        }
    }





}


public class GrapheIncidence2D
{
    // Dictionnaire des sommets vers leurs arêtes adjacentes
    public Dictionary<Vector2, HashSet<Edge>> adjacencySommetsAretes = new Dictionary<Vector2, HashSet<Edge>>();

    // Dictionnaire des arêtes vers les triangles qu'elles forment
    public Dictionary<Edge, HashSet<Triangle>> adjacencyAretesTriangles = new Dictionary<Edge, HashSet<Triangle>>();

    public void AjouterArete(Vector2 sommetA, Vector2 sommetB, Triangle triangle)
    {
        Edge nouvelleArete = new Edge(sommetA, sommetB);

        // Ajouter les sommets
        if (!adjacencySommetsAretes.ContainsKey(sommetA))
            adjacencySommetsAretes[sommetA] = new HashSet<Edge>();
        if (!adjacencySommetsAretes.ContainsKey(sommetB))
            adjacencySommetsAretes[sommetB] = new HashSet<Edge>();

        // Ajouter l'arête aux sommets
        adjacencySommetsAretes[sommetA].Add(nouvelleArete);
        adjacencySommetsAretes[sommetB].Add(nouvelleArete);

        // Ajouter le triangle à l'arête
        if (!adjacencyAretesTriangles.ContainsKey(nouvelleArete))
            adjacencyAretesTriangles[nouvelleArete] = new HashSet<Triangle>();
        adjacencyAretesTriangles[nouvelleArete].Add(triangle);
    }

    // Méthode pour vérifier si un point est connecté à d'autres points
    public bool EstConnecte(Vector2 point)
    {
        return adjacencySommetsAretes.ContainsKey(point) && adjacencySommetsAretes[point].Count > 0;
    }

    // Méthode pour obtenir les triangles adjacents à un point
    public HashSet<Triangle> GetTrianglesAdjacents(Vector2 point)
    {
        HashSet<Triangle> trianglesAdjacents = new HashSet<Triangle>();
        if (adjacencySommetsAretes.ContainsKey(point))
        {
            foreach (var arete in adjacencySommetsAretes[point])
            {
                if (adjacencyAretesTriangles.ContainsKey(arete))
                {
                    trianglesAdjacents.UnionWith(adjacencyAretesTriangles[arete]);
                }
            }
        }
        return trianglesAdjacents;
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

    public bool SharesVertex(Vector2 point)
    {
        return A == point || B == point || C == point;
    }

    public Vector2 GetOppositeVertex(Edge edge)
    {
        if (!HasEdge(edge)) return Vector2.zero;

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
        if (!(obj is Edge)) return false;
        Edge other = (Edge)obj;
        return (A == other.A && B == other.B) || (A == other.B && B == other.A);
    }

    public override int GetHashCode()
    {
        return A.GetHashCode() ^ B.GetHashCode();
    }
}
