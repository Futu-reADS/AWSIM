using UnityEngine;
using UnityEngine.UI;
using System.Collections;

public class ToggleObjectWithKey : MonoBehaviour
{
    public GameObject objectToToggle; // Assign the object you want to toggle in the Inspector
    private Text statusText;
    private Coroutine statusCoroutine;

    public ObjectSpawner objectSpawner;

    void Start()
    {
        // Check if statusText is assigned, if not, create it
        if (statusText == null)
        {
            GameObject canvas = new GameObject("Canvas");
            canvas.AddComponent<Canvas>().renderMode = RenderMode.ScreenSpaceOverlay;
            statusText = new GameObject("StatusText").AddComponent<Text>();
            statusText.transform.SetParent(canvas.transform);
            statusText.font = Resources.GetBuiltinResource<Font>("Arial.ttf");
            statusText.alignment = TextAnchor.MiddleCenter;
            statusText.rectTransform.sizeDelta = new Vector2(400, 100);
            statusText.rectTransform.anchoredPosition = new Vector2(0, -200);
            statusText.gameObject.SetActive(false);
        }
    }

    void Update()
    {
        if (Input.GetKeyDown(KeyCode.E))
        {
            bool isActive = !objectToToggle.activeSelf;
            objectToToggle.SetActive(isActive); // Toggle the object's active state

            // Stop the previous coroutine if it's running
            if (statusCoroutine != null)
            {
                StopCoroutine(statusCoroutine);
            }

            // Start a new coroutine
            statusCoroutine = StartCoroutine(ShowStatus(isActive ? "Scenario Enabled" : "Scenario Disabled"));
        }
        else if (Input.GetKeyDown(KeyCode.L))
        {
            objectSpawner.longDestroyTime = !objectSpawner.longDestroyTime;
            // Stop the previous coroutine if it's running
            if (statusCoroutine != null)
            {
                StopCoroutine(statusCoroutine);
            }

            // Start a new coroutine
            statusCoroutine = StartCoroutine(ShowStatus(objectSpawner.longDestroyTime? "Object destroy time set to 5min (long)" : "Object destroy time set to default"));
        }
    }

    IEnumerator ShowStatus(string message)
    {
        statusText.text = message;
        statusText.gameObject.SetActive(true);
        yield return new WaitForSeconds(3);
        statusText.gameObject.SetActive(false);
    }
}
