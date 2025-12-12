using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;


public class ObjectSpawner : MonoBehaviour
{
    public GameObject[] objectToSpawnArray; // Array of spawnable objects for selection
    public GameObject directionIndicatorPrefab; // Prefab for the direction indicator
    public GameObject egoVehicle; // Prefab for the autonomous vehicle
    private GameObject objectToSpawn; // The object that will be spawned
    private Vector3 initialMousePosition; // Initial position of the mouse when key is pressed
    private Vector3 initialRayHitPosition; // Initial position where the raycast hits an object
    private Vector3 moveDirection; // Move direction of the randomly spawned obstacles
    private GameObject spawnedObject; // The object that is spawned
    private GameObject directionIndicator; // The direction indicator object
    private float[] maxSpeedArray = new float[] {1.11f, 11.1f, 0.0f}; // Array of maximum speeds for the objects
    private float maxSpeed; // The maximum speed of the object that will be spawned
    private const float maxDragDistance = 100f; // Maximum distance the mouse can be dragged in pixels
    private const float maxIndicatorSize = 2f; // Maximum size scaling of the direction indicator
    private KeyCode[] keysToCheck = {KeyCode.H, KeyCode.V, KeyCode.O, KeyCode.Z, KeyCode.P, KeyCode.L}; // The array for pressable keys
    private KeyCode buttonKey; // The key that was pressed
    private bool isKeyPressed = false; // Flag to check if a key is being pressed
    private bool testObstacle = false;
    private float initialScaleY; // Initial scale of the static obstacle (the obstacle #3 or index 2 at objectToSpawnArray)
    private float destroyTime = 10f;
    private float spawnRange = 3.0f; // Spawn point distance from the vehicle
    private float halfLaneWidth = 1.5f; // Most common width of the lanes
    public bool longDestroyTime = false;  // 
    void Update()
    {
        // If no key is currently being pressed and any key is pressed down
        if (!isKeyPressed && Input.anyKeyDown)
        {
            KeyCode keyPressed = FetchKeyPressed(); // Fetch the key that was pressed
            switch (keyPressed)
            {
                case KeyCode.H:
                    testObstacle = false; // Set testObstacle flag
                    destroyTime = (longDestroyTime ? 300f : 10f); // Set destroy time to 10 sec
                    HandleKeyPress(0, keyPressed); // Handle the H key press
                    break;
                case KeyCode.V:
                    testObstacle = false; // Set testObstacle flag
                    destroyTime = (longDestroyTime ? 300f : 10f); // Set destroy time to 10 sec
                    HandleKeyPress(1, keyPressed); // Handle the V key press
                    break;
                case KeyCode.O:
                    testObstacle = true; // Set testObstacle flag
                    destroyTime = (longDestroyTime ? 300f : 60f); // Set destroy time to 10 sec
                    HandleKeyPress(2, keyPressed); // Handle the O key press
                    break;
                case KeyCode.Z:
                    testObstacle = false; // Set testObstacle flag
                    destroyTime = (longDestroyTime ? 300f : 20f); // Set destroy time to 10 sec
                    HandleKeyPress(0, keyPressed); // Handle the O key press
                    break;
                case KeyCode.P:
                    testObstacle = false; // Set testObstacle flag
                    destroyTime = (longDestroyTime ? 300f : 60f); // Set destroy time to 10 sec
                    HandleKeyPress(2, keyPressed); // Handle the O key press
                    break;
                // Add more cases here as needed
            }
        }

        // If a key is being pressed and the same key is released
        if (isKeyPressed && Input.GetKeyUp(buttonKey) && buttonKey != KeyCode.Z) 
        {
            HandleKeyRelease(); // Handle the key release
        }

        UpdateDirectionIndicator(); // Update the direction indicator
        UpdateTestObjectScale();
    }

    // Function to fetch the key that was pressed
    KeyCode FetchKeyPressed()
    {
        foreach (KeyCode vKey in keysToCheck)
        {
            if (Input.GetKey(vKey))
            {
                return vKey;
            }
        }
        return KeyCode.None;
    }

    // Function to handle a key press
    void HandleKeyPress(int index, KeyCode keyPressed)
    {
        isKeyPressed = true; // Set the flag to true
        buttonKey = keyPressed; // Store the key that was pressed
        objectToSpawn = objectToSpawnArray[index]; // Set the object to spawn
        maxSpeed = maxSpeedArray[index]; // Set the maximum speed of the object

        if (buttonKey == KeyCode.Z)
        {
            // Get vehicle position
            Vector3 vehiclePosition = egoVehicle.transform.position;
            // Generate a random spawn point within a specified distance to the vehicle
            float randomX = Random.Range(vehiclePosition.x - spawnRange, vehiclePosition.x + spawnRange);
            float randomZ = Random.Range(vehiclePosition.z - spawnRange, vehiclePosition.z + spawnRange);
            Vector3 randomSpawnPoint = new Vector3(randomX, 0, randomZ);
            // Calculate the vector from the prefab to the random point
            Vector3 directionToPoint = randomSpawnPoint - vehiclePosition;
            // Calculate the longitudinal distance (along the prefab's forward direction)
            float randomDistanceMag = directionToPoint.sqrMagnitude;

            
            if (randomDistanceMag < halfLaneWidth)
            {
                int randomMultiplier1 = Random.Range(0, 2) * 2 - 1;
                int randomMultiplier2 = Random.Range(0, 2) * 2 - 1;
                randomSpawnPoint = randomSpawnPoint + new Vector3(halfLaneWidth * randomMultiplier1, 0, halfLaneWidth * randomMultiplier2);
            }

            float longitudinalDistance = Vector3.Dot(directionToPoint, egoVehicle.transform.forward);
            // Calculate the lateral distance (along the prefab's right direction)
            float lateralDistance = Vector3.Dot(directionToPoint, egoVehicle.transform.right);

            if (Mathf.Abs(lateralDistance) > halfLaneWidth)
            {
                moveDirection = egoVehicle.transform.right * -lateralDistance/Mathf.Abs(lateralDistance);
            }
            else
            {
                moveDirection = egoVehicle.transform.forward  * -longitudinalDistance/Mathf.Abs(longitudinalDistance);
            }
                // Instantiate the object at the random point
                spawnedObject = Instantiate(objectToSpawn, randomSpawnPoint, Quaternion.identity);
                Destroy(spawnedObject, destroyTime); // Destroy the object after 10 seconds
                Rigidbody rb = spawnedObject.GetComponent<Rigidbody>(); // Get the Rigidbody component of the spawned object
                if (rb != null && !testObstacle)
                {
                    float speed = Random.Range(0, maxSpeed); 
                    // Apply the velocity to the Rigidbody
                    rb.velocity = moveDirection.normalized * speed; 
                    // Rotate the object to face the drag direction
                    spawnedObject.transform.rotation = Quaternion.LookRotation(moveDirection);
                }
            isKeyPressed = false;
        }
        else
        {
            Ray ray = Camera.main.ScreenPointToRay(Input.mousePosition); // Create a ray from the mouse position
            RaycastHit hit;
            // If the ray hits an object
            if (Physics.Raycast(ray, out hit)) 
            {
                // Instantiate the object at the hit point
                spawnedObject = Instantiate(objectToSpawn, hit.point, Quaternion.identity); 
                Vector3 cameraRotation = Camera.main.transform.eulerAngles;
                spawnedObject.transform.rotation = Quaternion.Euler(0, cameraRotation.y, 0); // Set the spawned object rotation towards the main camera
                Destroy(spawnedObject, destroyTime); // Destroy the object after 10 seconds
                initialMousePosition = Input.mousePosition; // Store the initial mouse position
                initialRayHitPosition = hit.point; // Store the initial hit point

                if (testObstacle){
                    initialScaleY = spawnedObject.transform.localScale.y; // Save the initial y scale
                    spawnedObject.AddComponent<TextFollower>();
                }
                else
                {
                    // Instantiate the direction indicator at the hit point
                    directionIndicator = Instantiate(directionIndicatorPrefab, initialRayHitPosition, Quaternion.identity);
                    Destroy(directionIndicator, destroyTime); // Destroy the direction indicator after 10 seconds               
                }
            }
        }
    }

    // Function to handle a key release
    void HandleKeyRelease()
    {
        isKeyPressed = false; // Set the flag to false
        Ray ray = Camera.main.ScreenPointToRay(Input.mousePosition); // Create a ray from the mouse position
        RaycastHit hit;
        // If the ray hits an object
        if (Physics.Raycast(ray, out hit)) 
        {
            Vector3 finalMousePosition = Input.mousePosition; // Get the final mouse position
            Vector3 dragDirection = hit.point - initialRayHitPosition; // Calculate the drag direction

            dragDirection.y = 0; // Set the y component of the drag direction to 0

            Rigidbody rb = spawnedObject.GetComponent<Rigidbody>(); // Get the Rigidbody component of the spawned object
            if (rb != null && !testObstacle)
            {
                // Calculate the drag distance and speed
                float dragDistance = Vector3.Distance(initialMousePosition, finalMousePosition);
                float speed = Mathf.Min(dragDistance / maxDragDistance, 1f) * maxSpeed; 
                // Apply the velocity to the Rigidbody
                rb.velocity = new Vector3(dragDirection.x, 0, dragDirection.z).normalized * speed; 
                // Rotate the object to face the drag direction
                spawnedObject.transform.rotation = Quaternion.LookRotation(dragDirection);
            }
        }
        if(directionIndicator != null){
            Destroy(directionIndicator); // Destroy the direction indicator
        }
        testObstacle = false;
    }

    // Function to update the direction indicator
    void UpdateDirectionIndicator()
    {
        // If the direction indicator exists
        if (directionIndicator != null) 
        {
            Ray ray = Camera.main.ScreenPointToRay(Input.mousePosition); // Create a ray from the mouse position
            RaycastHit hit;
            // If the ray hits an object
            if (Physics.Raycast(ray, out hit)) 
            {
                // Position the direction indicator
                directionIndicator.transform.position = new Vector3(initialRayHitPosition.x, 2, initialRayHitPosition.z); 

                // Calculate the drag distance and scale
                float dragDistance = Vector3.Distance(initialMousePosition, Input.mousePosition); 
                float scale = Mathf.Min(dragDistance / maxDragDistance, 1f) * maxIndicatorSize; 
                // Scale the direction indicator
                directionIndicator.transform.localScale = new Vector3(scale, scale, scale); 

                // Calculate the direction
                Vector3 direction = hit.point - initialRayHitPosition;
                direction.y = 0; 
                // Rotate the direction indicator to face the direction
                directionIndicator.transform.rotation = Quaternion.LookRotation(direction) * Quaternion.Euler(0, 90, 0); 
            }
        }
        // If the spawned object does not exist and the direction indicator exists
        if (spawnedObject == null && directionIndicator != null)
        {
            Destroy(directionIndicator); // Destroy the direction indicator
        }
    }
    void UpdateTestObjectScale()
    {
        if(spawnedObject != null && testObstacle)
        {
            Ray ray = Camera.main.ScreenPointToRay(Input.mousePosition); // Create a ray from the mouse position
            RaycastHit hit;
            // If the ray hits an object
            if (Physics.Raycast(ray, out hit)) 
            {
                // Calculate the drag distance and scale
                float dragDistance = Vector3.Distance(initialMousePosition, Input.mousePosition); 
                float scale = Mathf.Min(dragDistance / maxDragDistance, 1f) * maxIndicatorSize; 

                // Apply the scale factor to the y scale of the object
                Vector3 scaleVect = spawnedObject.transform.localScale;
                scaleVect.y = initialScaleY * scale;
                spawnedObject.transform.localScale = scaleVect;
            }
        }
    }
}