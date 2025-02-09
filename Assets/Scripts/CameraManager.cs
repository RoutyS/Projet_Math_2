using UnityEngine;

public class CameraManager : MonoBehaviour
{
    public float rotationSpeed = 2f;
    public float panSpeed = 10f;
    public float zoomSpeed = 10f;

    private Vector3 lastMousePosition;
    private bool isRotating = false;
    private bool isPanning = false;

    void Update()
    {
        HandleMouseInput();
    }

    void HandleMouseInput()
    {
        // Rotation avec clic droit
        if (Input.GetMouseButtonDown(1))
        {
            isRotating = true;
            lastMousePosition = Input.mousePosition;
        }
        else if (Input.GetMouseButtonUp(1))
        {
            isRotating = false;
        }

        // Pan avec clic molette
        if (Input.GetMouseButtonDown(2))
        {
            isPanning = true;
            lastMousePosition = Input.mousePosition;
        }
        else if (Input.GetMouseButtonUp(2))
        {
            isPanning = false;
        }

        // Zoom avec la molette
        float scrollDelta = Input.GetAxis("Mouse ScrollWheel");
        if (scrollDelta != 0)
        {
            Vector3 zoomDirection = transform.forward;
            transform.position += zoomDirection * scrollDelta * zoomSpeed;
        }

        // Appliquer la rotation
        if (isRotating)
        {
            Vector3 mouseDelta = Input.mousePosition - lastMousePosition;

            // Rotation horizontale autour de l'axe Y global
            transform.RotateAround(transform.position, Vector3.up, mouseDelta.x * rotationSpeed);

            // Rotation verticale autour de l'axe X local
            transform.RotateAround(transform.position, transform.right, -mouseDelta.y * rotationSpeed);
        }

        // Appliquer le pan
        if (isPanning)
        {
            Vector3 mouseDelta = Input.mousePosition - lastMousePosition;
            Vector3 moveDirection =
                transform.right * (-mouseDelta.x) +
                transform.up * (-mouseDelta.y);

            transform.position += moveDirection * panSpeed * Time.deltaTime;
        }

        lastMousePosition = Input.mousePosition;
    }
}