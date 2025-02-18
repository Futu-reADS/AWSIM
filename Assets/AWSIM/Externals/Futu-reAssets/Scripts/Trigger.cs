using UnityEngine;

public class Trigger : MonoBehaviour
{
    public GameObject triggerObject;
    public GameObject objectToDisable;
    public GameObject obstacle;
    public bool resetState = false;
    // private Patrol patrolScript;

    void Start()
    {
        // patrolScript = obstacle.GetComponent<Patrol>();
        objectToDisable.SetActive(false);
        // Ensure the collider is set as a trigger
        Collider collider = GetComponent<Collider>();
        if (collider != null)
        {
            collider.isTrigger = true;
        }
    }

    void OnTriggerEnter(Collider other)
    {
        if (other.gameObject == triggerObject)
        {
            objectToDisable.SetActive(true);
            resetState = false;
        }
    }

    void OnTriggerExit(Collider other)
    {
        if (other.gameObject == triggerObject)
        {
            resetState = true;
        }
    }
}
