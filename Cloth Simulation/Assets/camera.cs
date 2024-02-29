using UnityEngine;

public class SimpleCameraController : MonoBehaviour
{
    public float movementSpeed = 10f;
    public float turnSpeed = 4f;
    private bool isMovementEnabled = true;

    void Update()
    {
        // Toggle movement with the "M" key
        if (Input.GetKeyDown(KeyCode.M))
        {
            isMovementEnabled = !isMovementEnabled;
        }

        if (isMovementEnabled)
        {
            // Movement
            float horizontal = Input.GetAxis("Horizontal") * movementSpeed * Time.deltaTime;
            float vertical = Input.GetAxis("Vertical") * movementSpeed * Time.deltaTime;

            transform.Translate(horizontal, 0, vertical);

            // Turning
            float yaw = Input.GetAxis("Mouse X") * turnSpeed;
            float pitch = Input.GetAxis("Mouse Y") * turnSpeed;

            transform.Rotate(0, yaw, 0, Space.World);
            transform.Rotate(-pitch, 0, 0);
        }
    }
}