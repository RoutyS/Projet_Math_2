using UnityEngine;
using System.Collections.Generic;
using System.Linq;
using System;
using System.Diagnostics;

public class PointManager3D : MonoBehaviour
{
    [Header("Visualization")]
    public GameObject pointPrefab;
    public LineRenderer lineRenderer;  // Référence au LineRenderer
    public float pointSize = 0.2f;

    public Color colorJarvis = Color.blue;
    public Color colorGraham = Color.green;
    public Color colorIncremental = Color.cyan;
    public Color colorDelaunay = Color.yellow;
    public Color colorVoronoi = Color.red;

    private List<Vector3> points3D = new List<Vector3>();
    private List<GameObject> pointObjects = new List<GameObject>();
    private List<Tetrahedron> tetrahedra = new List<Tetrahedron>();
    private Camera mainCamera;

    private GrapheIncidence graphe = new GrapheIncidence();
    private Vector3? dragStartPosition;

    private bool afficherCouleursOrientation = false;


    void Start()
    {
        mainCamera = Camera.main;
        if (lineRenderer != null)
        {
            // Configuration du LineRenderer
            lineRenderer.material = new Material(Shader.Find("Sprites/Default"));
            lineRenderer.startWidth = 0.05f;
            lineRenderer.endWidth = 0.05f;
            lineRenderer.positionCount = 0;
            // Définir le matériau pour supporter les couleurs
            lineRenderer.material.SetColor("_Color", Color.white);
        }
    }

    void Update()
    {
        HandleInput();
    }

    void HandleInput()
    {
        // Mode de placement des points
        if (Input.GetKey(KeyCode.LeftControl)) // Maintenir Ctrl pour un placement précis
        {
            if (Input.GetMouseButtonDown(0))
            {
                Ray ray = mainCamera.ScreenPointToRay(Input.mousePosition);
                Plane horizontalPlane = new Plane(Vector3.up, Vector3.zero);

                // Utiliser cette version de Raycast
                if (horizontalPlane.Raycast(ray, out float distance))
                {
                    Vector3 precisPoint = ray.GetPoint(distance);
                    AddPoint3D(precisPoint);

                    // Afficher des informations de débogage
                    UnityEngine.Debug.Log($"Point 3D précis ajouté : {precisPoint}");
                }
            }
        }
        else
        {
            // Mode placement standard
            if (Input.GetMouseButtonDown(0))
            {
                Ray ray = mainCamera.ScreenPointToRay(Input.mousePosition);
                float distance = 10f;
                Vector3 pointPos = ray.GetPoint(distance);
                AddPoint3D(pointPos);
            }
        }

        // Autres touches de contrôle
        if (Input.GetKeyDown(KeyCode.Space)) // Jarvis March
        {
            ClearVisualization();
            var stopwatch = System.Diagnostics.Stopwatch.StartNew();
            JarvisMarch3D();
            stopwatch.Stop();
            UnityEngine.Debug.Log($"Jarvis March 3D exécuté en {stopwatch.ElapsedMilliseconds}ms");
        }

        if (Input.GetKeyDown(KeyCode.G)) // Graham Scan
        {
            ClearVisualization();
            GrahamScan3D();
        }

        if (Input.GetKeyDown(KeyCode.T)) // Triangulation Incrémentale
        {
            ClearVisualization();
            TriangulationIncrementale3D();
        }

        if (Input.GetKeyDown(KeyCode.D)) // Triangulation de Delaunay
        {
            ClearVisualization();
            TriangulationDelaunay3D();
        }

        if (Input.GetKeyDown(KeyCode.V)) // Diagramme de Voronoï
        {
            ClearVisualization();
            GenerateVoronoi3D();
        }

        if (Input.GetKeyDown(KeyCode.R)) // Reset
        {
            ClearAll();
        }

        if (Input.GetKeyDown(KeyCode.P)) // Point aléatoire
        {
            AddRandomPoint3D();
        }

        if (Input.GetKeyDown(KeyCode.I)) // Débogage
        {
            DebugGrapheIncidence3D();
        }

        if (Input.GetKeyDown(KeyCode.O))
        {
            afficherCouleursOrientation = !afficherCouleursOrientation;
            MettreAJourAffichageOrientation3D();
            UnityEngine.Debug.Log("Affichage des tétraèdres en 3D : " + (afficherCouleursOrientation ? "Activé" : "Désactivé"));
        }



    }
    /*void HandlePointPlacement()
    {
        if (Input.GetMouseButtonDown(0))  // Premier clic
        {
            Ray ray = mainCamera.ScreenPointToRay(Input.mousePosition);
            dragStartPosition = ray.GetPoint(10f);
        }

        if (Input.GetMouseButtonUp(0) && dragStartPosition.HasValue)
        {
            Ray ray = mainCamera.ScreenPointToRay(Input.mousePosition);
            Vector3 endPosition = ray.GetPoint(10f);
            float height = endPosition.y - dragStartPosition.Value.y;
            Vector3 newPoint = new Vector3(dragStartPosition.Value.x, height, dragStartPosition.Value.z);
            AddPoint3D(newPoint);
            dragStartPosition = null;
        }
    }*/

    void HandlePointPlacement()
    {
        // Mode placement avec contraintes
        if (Input.GetKey(KeyCode.LeftControl)) // Maintenir Ctrl pour un placement précis
        {
            // Placement sur un plan de travail
            if (Input.GetMouseButtonDown(0))
            {
                Ray ray = mainCamera.ScreenPointToRay(Input.mousePosition);
                Plane horizontalPlane = new Plane(Vector3.up, Vector3.zero);

                // Utiliser cette version de Raycast
                if (horizontalPlane.Raycast(ray, out float distance))
                {
                    Vector3 precisPoint = ray.GetPoint(distance);
                    AddPoint3D(precisPoint);

                    // Afficher des informations de débogage
                    UnityEngine.Debug.Log($"Point 3D précis ajouté : {precisPoint}");
                }
            }
        }
        else
        {
            // Votre méthode de placement existante
            if (Input.GetMouseButtonDown(0))
            {
                Ray ray = mainCamera.ScreenPointToRay(Input.mousePosition);
                float distance = 10f;
                Vector3 pointPos = ray.GetPoint(distance);
                AddPoint3D(pointPos);
            }
        }
    }

    [Header("Point Configuration")]
    public float minPointSize = 0.1f;
    public float maxPointSize = 0.5f;
    public Color pointColorGradient = Color.white;

    void ConfigurePointVisualization(GameObject point)
    {
        // Configurer la taille et la couleur
        point.transform.localScale = Vector3.one * pointSize;

        // Appliquer un dégradé de couleur basé sur la position
        Renderer renderer = point.GetComponent<Renderer>();
        if (renderer != null)
        {
            float heightFactor = Mathf.InverseLerp(0, 10, point.transform.position.y);
            renderer.material.color = Color.Lerp(Color.blue, Color.red, heightFactor);
        }
    }

    void AddRandomPoint3D()
    {
        Vector3 randomPoint = new Vector3(
            UnityEngine.Random.Range(-10f, 10f),
            UnityEngine.Random.Range(0f, 10f),
            UnityEngine.Random.Range(-10f, 10f)
        );
        AddPoint3D(randomPoint);
    }

    void AddPoint3D(Vector3 position)
    {
        points3D.Add(position);
        GameObject point = Instantiate(pointPrefab, position, Quaternion.identity);

        ConfigurePointVisualization(point);

        pointObjects.Add(point);

        point.transform.localScale = Vector3.one * pointSize;
        pointObjects.Add(point);

        // Mettre à jour le graphe d'incidence
        if (points3D.Count >= 4)
        {
            foreach (var edge in CreateEdgesForPoint(position))
            {
                graphe.AjouterArete(position, edge);
            }
        }
    }

    private List<Edge3D> CreateEdgesForPoint(Vector3 newPoint)
    {
        List<Edge3D> newEdges = new List<Edge3D>();
        foreach (var existingPoint in points3D)
        {
            if (existingPoint != newPoint)
            {
                newEdges.Add(new Edge3D(newPoint, existingPoint));
            }
        }
        return newEdges;
    }

    void ClearVisualization()
    {
        if (lineRenderer != null)
        {
            lineRenderer.positionCount = 0;
        }
        tetrahedra.Clear();
    }

    private void ClearAll()
    {
        // Effacer les visualisations
        ClearVisualization();

        // Détruire tous les points
        foreach (var point in pointObjects)
        {
            Destroy(point);
        }
        pointObjects.Clear();
        points3D.Clear();
    }

    public class GrapheIncidence
    {
        public Dictionary<Vector3, List<Edge3D>> sommetVersAretes = new Dictionary<Vector3, List<Edge3D>>();
        public Dictionary<Edge3D, List<Face3D>> areteVersFaces = new Dictionary<Edge3D, List<Face3D>>();
        public Dictionary<Vector3, List<Face3D>> sommetVersFaces = new Dictionary<Vector3, List<Face3D>>();

        public void AjouterArete(Vector3 sommet, Edge3D arete)
        {
            if (!sommetVersAretes.ContainsKey(sommet))
                sommetVersAretes[sommet] = new List<Edge3D>();
            sommetVersAretes[sommet].Add(arete);
        }

        public void AjouterFace(Edge3D arete, Face3D face)
        {
            if (!areteVersFaces.ContainsKey(arete))
                areteVersFaces[arete] = new List<Face3D>();
            areteVersFaces[arete].Add(face);
        }
    }

    // Ajout de la méthode pour dessiner une liste de lignes
    private void DrawLines(List<Vector3> points, Color color)
    {
        if (lineRenderer == null) return;

        lineRenderer.startColor = color;
        lineRenderer.endColor = color;
        lineRenderer.positionCount = points.Count;
        lineRenderer.SetPositions(points.ToArray());
    }

    private void DrawTetrahedron(Tetrahedron tetra, Color color)
    {
        List<Vector3> lines = new List<Vector3>();

        // Ajouter toutes les arêtes
        lines.Add(tetra.A);
        lines.Add(tetra.B);

        lines.Add(tetra.B);
        lines.Add(tetra.C);

        lines.Add(tetra.C);
        lines.Add(tetra.A);

        lines.Add(tetra.A);
        lines.Add(tetra.D);

        lines.Add(tetra.B);
        lines.Add(tetra.D);

        lines.Add(tetra.C);
        lines.Add(tetra.D);

        if (lineRenderer != null)
        {
            lineRenderer.startColor = color;
            lineRenderer.endColor = color;
            lineRenderer.material.color = color;  // Important pour la couleur
            lineRenderer.positionCount = lines.Count;
            lineRenderer.SetPositions(lines.ToArray());
        }
    }

    // Jarvis March en 3D (Gift Wrapping)
    void JarvisMarch3D()
    {
        if (points3D.Count < 4) return;

        List<Triangle3D> hullFaces = new List<Triangle3D>();

        Vector3 firstPoint = points3D.OrderBy(p => p.y).First();
        Vector3 secondPoint = points3D
            .OrderBy(p => Vector3.Angle(Vector3.right, p - firstPoint))
            .Skip(1)
            .First();
        Vector3 thirdPoint = points3D
            .OrderBy(p => Vector3.Angle(Vector3.Cross(secondPoint - firstPoint, Vector3.up), p - firstPoint))
            .Skip(2)
            .First();

        hullFaces.Add(new Triangle3D(firstPoint, secondPoint, thirdPoint));

        foreach (var face in hullFaces.ToList())
        {
            foreach (var edge in face.GetEdges())
            {
                Vector3 bestPoint = Vector3.zero;
                float bestAngle = float.MinValue;

                foreach (var point in points3D)
                {
                    if (point != edge.Start && point != edge.End)
                    {
                        float angle = Vector3.SignedAngle(
                            face.Normal,
                            Vector3.Cross(edge.End - edge.Start, point - edge.Start),
                            edge.End - edge.Start
                        );

                        if (angle > bestAngle)
                        {
                            bestAngle = angle;
                            bestPoint = point;
                        }
                    }
                }

                if (bestPoint != Vector3.zero)
                {
                    hullFaces.Add(new Triangle3D(edge.Start, edge.End, bestPoint));
                }
            }
        }

        // Visualisation avec le LineRenderer
        List<Vector3> lines = new List<Vector3>();
        foreach (var face in hullFaces)
        {
            foreach (var edge in face.GetEdges())
            {
                lines.Add(edge.Start);
                lines.Add(edge.End);
            }
        }

        if (lineRenderer != null)
        {
            lineRenderer.startColor = colorJarvis;
            lineRenderer.endColor = colorJarvis;
            lineRenderer.positionCount = lines.Count;
            lineRenderer.SetPositions(lines.ToArray());
        }
    }

    // Graham Scan adapté en 3D
    void GrahamScan3D()
    {
        if (points3D.Count < 4) return;

        Vector3 center = points3D.Aggregate(Vector3.zero, (acc, p) => acc + p) / points3D.Count;
        var sortedPoints = points3D
            .OrderBy(p => Vector3.SignedAngle(Vector3.right, p - center, Vector3.up))
            .ToList();

        List<Vector3> hull = new List<Vector3>();
        foreach (var point in sortedPoints)
        {
            while (hull.Count >= 2 && !IsConvex(hull[hull.Count - 2], hull[hull.Count - 1], point))
            {
                hull.RemoveAt(hull.Count - 1);
            }
            hull.Add(point);
        }

        hull.Add(hull[0]); // Fermer l'enveloppe

        if (lineRenderer != null)
        {
            lineRenderer.startColor = colorGraham;
            lineRenderer.endColor = colorGraham;
            lineRenderer.positionCount = hull.Count;
            lineRenderer.SetPositions(hull.ToArray());
        }
    }

    // Triangulation Incrémentale en 3D
    void TriangulationIncrementale3D()
    {
        if (points3D.Count < 4) return;

        tetrahedra.Clear();
        tetrahedra.Add(new Tetrahedron(points3D[0], points3D[1], points3D[2], points3D[3]));

        for (int i = 4; i < points3D.Count; i++)
        {
            Vector3 point = points3D[i];
            List<Tetrahedron> badTetrahedra = new List<Tetrahedron>();

            foreach (var tetra in tetrahedra.ToList())
            {
                if (tetra.IsPointInCircumsphere(point))
                {
                    badTetrahedra.Add(tetra);
                }
            }

            foreach (var tetra in badTetrahedra)
            {
                tetrahedra.Remove(tetra);
            }

            foreach (var face in GetBoundaryFaces(badTetrahedra))
            {
                tetrahedra.Add(new Tetrahedron(face.A, face.B, face.C, point));
            }
        }

        DrawTetrahedra(tetrahedra, colorIncremental);
    }

    // Triangulation de Delaunay en 3D
    void TriangulationDelaunay3D()
    {
        if (points3D.Count < 4) return;

        var stopwatch = new Stopwatch();
        stopwatch.Start();
        float size = 1000f;

        Vector3[] superTetraPoints = new[]
        {
            new Vector3(-size, -size, -size),
            new Vector3(size, -size, -size),
            new Vector3(0, size, -size),
            new Vector3(0, 0, size)
        };

        tetrahedra.Clear();
        tetrahedra.Add(new Tetrahedron(
            superTetraPoints[0],
            superTetraPoints[1],
            superTetraPoints[2],
            superTetraPoints[3]
        ));

        foreach (var point in points3D)
        {
            AddPointToDelaunay(point);
        }

        tetrahedra.RemoveAll(t =>
            t.HasVertex(superTetraPoints[0]) ||
            t.HasVertex(superTetraPoints[1]) ||
            t.HasVertex(superTetraPoints[2]) ||
            t.HasVertex(superTetraPoints[3])
        );

        DrawTetrahedra(tetrahedra, colorDelaunay);
        stopwatch.Stop();
        UnityEngine.Debug.Log($"Triangulation Delaunay 3D effectuée en {stopwatch.ElapsedMilliseconds}ms");
    }

    void AddPointToDelaunay(Vector3 point)
    {
        List<Tetrahedron> badTetrahedra = new List<Tetrahedron>();

        foreach (var tetra in tetrahedra.ToList())
        {
            if (tetra.IsPointInCircumsphere(point))
            {
                badTetrahedra.Add(tetra);
            }
        }

        List<Face3D> boundary = GetBoundaryFaces(badTetrahedra);

        foreach (var tetra in badTetrahedra)
        {
            tetrahedra.Remove(tetra);
        }

        foreach (var face in boundary)
        {
            tetrahedra.Add(new Tetrahedron(face.A, face.B, face.C, point));
        }
    }

    // Diagramme de Voronoï en 3D
    void GenerateVoronoi3D()
    {
        if (points3D.Count < 4) return;

        TriangulationDelaunay3D();

        List<Vector3> voronoiLines = new List<Vector3>();
        foreach (var tetra in tetrahedra)
        {
            Vector3 center = tetra.CalculateCircumcenter();

            foreach (var face in tetra.GetFaces())
            {
                var adjacentTetra = FindAdjacentTetrahedron(tetra, face);
                if (adjacentTetra != null)
                {
                    Vector3 adjacentCenter = adjacentTetra.CalculateCircumcenter();
                    voronoiLines.Add(center);
                    voronoiLines.Add(adjacentCenter);
                }
            }
        }

        if (lineRenderer != null)
        {
            lineRenderer.startColor = colorVoronoi;
            lineRenderer.endColor = colorVoronoi;
            lineRenderer.positionCount = voronoiLines.Count;
            lineRenderer.SetPositions(voronoiLines.ToArray());
        }
    }

    // Méthodes utilitaires
    private List<Face3D> GetBoundaryFaces(List<Tetrahedron> tetrahedra)
    {
        Dictionary<Face3D, int> faceCounts = new Dictionary<Face3D, int>();

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

    private Tetrahedron FindAdjacentTetrahedron(Tetrahedron tetra, Face3D face)
    {
        return tetrahedra.FirstOrDefault(t =>
            t != tetra && t.HasFace(face));
    }

    private bool IsConvex(Vector3 a, Vector3 b, Vector3 c)
    {
        return Vector3.Cross(b - a, c - b).y > 0;
    }

    // Méthodes de dessin
    private void DrawTetrahedra(List<Tetrahedron> tetras, Color color)
    {
        CorrigerOrientationTetrahedres();

        List<Vector3> lines = new List<Vector3>();
        foreach (var tetra in tetras)
        {
            // Première face (ABC)
            lines.Add(tetra.A);
            lines.Add(tetra.B);
            lines.Add(tetra.B);
            lines.Add(tetra.C);
            lines.Add(tetra.C);
            lines.Add(tetra.A);

            // Connexions avec D
            lines.Add(tetra.A);
            lines.Add(tetra.D);
            lines.Add(tetra.B);
            lines.Add(tetra.D);
            lines.Add(tetra.C);
            lines.Add(tetra.D);
        }

        foreach (var tetra in tetras)
        {
            CreateTetrahedronVisualization(tetra, color);
        }

        if (lineRenderer != null)
        {
            lineRenderer.startColor = color;
            lineRenderer.endColor = color;
            lineRenderer.positionCount = lines.Count;
            lineRenderer.SetPositions(lines.ToArray());
        }
    }

    float Orientation3D(Vector3 a, Vector3 b, Vector3 c, Vector3 d)
    {
        float det = Matrix4x4.TRS(Vector3.zero, Quaternion.identity, Vector3.one).determinant;
        return det;
    }

    void CorrigerOrientationTriangles3D()
    {
        for (int i = 0; i < tetrahedra.Count; i++)
        {
            Tetrahedron t = tetrahedra[i];

            // Calcul du déterminant pour vérifier si la base est orientée correctement
            float orientation = Orientation3D(t.A, t.B, t.C, t.D);

            if (orientation < 0) // Mauvais sens → on inverse un sommet
            {
                (t.B, t.C) = (t.C, t.B);
            }
        }

        UnityEngine.Debug.Log("Orientation des tétraèdres corrigée en 3D !");
    }


    void DebugGrapheIncidence3D()
    {
        UnityEngine.Debug.Log("Débogage du Graphe d'Incidence 3D :");
        UnityEngine.Debug.Log($"Nombre total de sommets : {points3D.Count}");
        UnityEngine.Debug.Log($"Nombre de tétraèdres : {tetrahedra.Count}");

        // Analyse des arêtes
        HashSet<Edge3D> toutesLesAretes = new HashSet<Edge3D>();
        foreach (var tetra in tetrahedra)
        {
            toutesLesAretes.UnionWith(tetra.Aretes);
        }
        UnityEngine.Debug.Log($"Nombre total d'arêtes : {toutesLesAretes.Count}");

        // Analyse des faces
        HashSet<Face3D> toutesLesFaces = new HashSet<Face3D>();
        foreach (var tetra in tetrahedra)
        {
            toutesLesFaces.UnionWith(tetra.Faces);
        }
        UnityEngine.Debug.Log($"Nombre total de faces : {toutesLesFaces.Count}");
    }

    void CreateTetrahedronVisualization(Tetrahedron tetra, Color color)
    {
        GameObject tetraObj = new GameObject("Tetrahedron");
        tetraObj.tag = "Tetrahedron";

        MeshFilter meshFilter = tetraObj.AddComponent<MeshFilter>();
        MeshRenderer meshRenderer = tetraObj.AddComponent<MeshRenderer>();

        Mesh mesh = new Mesh();
        Vector3[] vertices = { tetra.A, tetra.B, tetra.C, tetra.D };
        int[] triangles = {
        0, 1, 2,  // Face ABC
        0, 1, 3,  // Face ABD
        0, 2, 3,  // Face ACD
        1, 2, 3   // Face BCD
    };

        mesh.vertices = vertices;
        mesh.triangles = triangles;
        mesh.RecalculateNormals();

        meshFilter.mesh = mesh;

        // Appliquer un matériau coloré
        meshRenderer.material = new Material(Shader.Find("Standard"));
        meshRenderer.material.color = color;
    }


    void MettreAJourAffichageOrientation3D()
    {
        GameObject[] tetrahedrons = GameObject.FindGameObjectsWithTag("Tetrahedron");

        foreach (GameObject obj in tetrahedrons)
        {
            MeshRenderer meshRenderer = obj.GetComponent<MeshRenderer>();
            if (meshRenderer != null)
            {
                meshRenderer.enabled = afficherCouleursOrientation;
            }
        }
    }

    void CorrigerOrientationTetrahedres()
    {
        for (int i = 0; i < tetrahedra.Count; i++)
        {
            Tetrahedron t = tetrahedra[i];

            // Vérification de l'orientation du tétraèdre
            float orientation = Orientation3D(t.A, t.B, t.C, t.D);

            if (orientation < 0) // Si la base est mal orientée, on inverse un sommet
            {
                (t.B, t.C) = (t.C, t.B); // On échange B et C
            }
        }
        UnityEngine.Debug.Log("Orientation des tétraèdres corrigée !");
    }




}

// Classes de support
public class Triangle3D
{
    public Vector3 A, B, C;
    public Vector3 Normal;

    public Triangle3D(Vector3 a, Vector3 b, Vector3 c)
    {
        A = a;
        B = b;
        C = c;
        Normal = Vector3.Cross(B - A, C - A).normalized;
    }

    public Edge3D[] GetEdges()
    {
        return new[]
        {
            new Edge3D(A, B),
            new Edge3D(B, C),
            new Edge3D(C, A)
        };
    }
}

public class Edge3D
{
    public Vector3 Start, End;

    public Edge3D(Vector3 start, Vector3 end)
    {
        Start = start;
        End = end;
    }
}

public class Tetrahedron
{
    public Vector3 A, B, C, D;
    public List<Edge3D> Aretes { get; private set; }
    public List<Face3D> Faces { get; private set; }

    public Tetrahedron(Vector3 a, Vector3 b, Vector3 c, Vector3 d)
    {
        A = a; B = b; C = c; D = d;

        // Construire les arêtes
        Aretes = new List<Edge3D>
        {
            new Edge3D(A, B),
            new Edge3D(B, C),
            new Edge3D(C, A),
            new Edge3D(A, D),
            new Edge3D(B, D),
            new Edge3D(C, D)
        };

        // Construire les faces
        Faces = GetFaces();
    }

    public bool IsPointInCircumsphere(Vector3 point)
    {
        Vector3 center = CalculateCircumcenter();
        float radius = Vector3.Distance(center, A);
        return Vector3.Distance(center, point) <= radius;
    }

    public Vector3 CalculateCircumcenter()
    {
        // Calculer le centre de la sphère circonscrite
        Matrix4x4 m = new Matrix4x4();
        m.SetRow(0, new Vector4(A.x, A.y, A.z, 1));
        m.SetRow(1, new Vector4(B.x, B.y, B.z, 1));
        m.SetRow(2, new Vector4(C.x, C.y, C.z, 1));
        m.SetRow(3, new Vector4(D.x, D.y, D.z, 1));

        if (Mathf.Abs(m.determinant) < 1e-10f)
        {
            return (A + B + C + D) / 4f;
        }

        Matrix4x4 mx = m;
        mx.SetRow(0, new Vector4(A.sqrMagnitude, A.y, A.z, 1));
        mx.SetRow(1, new Vector4(B.sqrMagnitude, B.y, B.z, 1));
        mx.SetRow(2, new Vector4(C.sqrMagnitude, C.y, C.z, 1));
        mx.SetRow(3, new Vector4(D.sqrMagnitude, D.y, D.z, 1));
        float dx = mx.determinant;

        Matrix4x4 my = m;
        my.SetRow(0, new Vector4(A.sqrMagnitude, A.x, A.z, 1));
        my.SetRow(1, new Vector4(B.sqrMagnitude, B.x, B.z, 1));
        my.SetRow(2, new Vector4(C.sqrMagnitude, C.x, C.z, 1));
        my.SetRow(3, new Vector4(D.sqrMagnitude, D.x, D.z, 1));
        float dy = -my.determinant;

        Matrix4x4 mz = m;
        mz.SetRow(0, new Vector4(A.sqrMagnitude, A.x, A.y, 1));
        mz.SetRow(1, new Vector4(B.sqrMagnitude, B.x, B.y, 1));
        mz.SetRow(2, new Vector4(C.sqrMagnitude, C.x, C.y, 1));
        mz.SetRow(3, new Vector4(D.sqrMagnitude, D.x, D.y, 1));
        float dz = mz.determinant;

        return new Vector3(dx / (2f * m.determinant), dy / (2f * m.determinant), dz / (2f * m.determinant));
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

    public bool HasFace(Face3D face)
    {
        return GetFaces().Any(f => f.Equals(face));
    }

    public bool HasVertex(Vector3 vertex)
    {
        float epsilon = 1e-6f;
        return Vector3.Distance(A, vertex) < epsilon ||
               Vector3.Distance(B, vertex) < epsilon ||
               Vector3.Distance(C, vertex) < epsilon ||
               Vector3.Distance(D, vertex) < epsilon;
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

namespace GestionGraphes
{
    public class GrapheIncidence
    {
        public Dictionary<Vector3, List<Edge3D>> sommetVersAretes = new Dictionary<Vector3, List<Edge3D>>();
        public Dictionary<Edge3D, List<Face3D>> areteVersFaces = new Dictionary<Edge3D, List<Face3D>>();
        public Dictionary<Vector3, List<Face3D>> sommetVersFaces = new Dictionary<Vector3, List<Face3D>>();

        public void AjouterArete(Vector3 sommet, Edge3D arete)
        {
            if (!sommetVersAretes.ContainsKey(sommet))
                sommetVersAretes[sommet] = new List<Edge3D>();
            sommetVersAretes[sommet].Add(arete);
        }

        public void AjouterFace(Edge3D arete, Face3D face)
        {
            if (!areteVersFaces.ContainsKey(arete))
                areteVersFaces[arete] = new List<Face3D>();
            areteVersFaces[arete].Add(face);
        }
    }
}
