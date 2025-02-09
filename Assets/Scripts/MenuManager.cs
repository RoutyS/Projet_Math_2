using UnityEngine;
using UnityEngine.UI;

public class MenuManager : MonoBehaviour
{
    public MonoBehaviour script2D; // Référence au script 2D (PointManager2D)
    public MonoBehaviour script3D; // Référence au script 3D (PointManager3D)
    public GameObject canvas; // Référence au Canvas contenant le menu

    public Button button2D; // Bouton pour activer le mode 2D
    public Button button3D; // Bouton pour activer le mode 3D

    void Start()
    {
        // Ajouter des listeners pour les boutons
        button2D.onClick.AddListener(Activate2D);
        button3D.onClick.AddListener(Activate3D);

        // Désactiver les deux scripts au début pour éviter les conflits
        script2D.enabled = false;
        script3D.enabled = false;
    }

    void Activate2D()
    {
        script2D.enabled = true; // Activer le script 2D
        script3D.enabled = false; // Désactiver le script 3D
        canvas.SetActive(false); // Masquer le Canvas après le choix
        Debug.Log("Mode 2D activé");
    }

    void Activate3D()
    {
        script2D.enabled = false; // Désactiver le script 2D
        script3D.enabled = true; // Activer le script 3D
        canvas.SetActive(false); // Masquer le Canvas après le choix
        Debug.Log("Mode 3D activé");
    }
}
