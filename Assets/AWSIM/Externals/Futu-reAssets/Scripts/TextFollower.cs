using UnityEngine;
using UnityEngine.UI;

public class TextFollower : MonoBehaviour
{
    private Text instanceText; // The text instance
    private GameObject textObject; // The text GameObject

    void Start()
    {
        // Create a new Canvas GameObject
        GameObject canvasObject = new GameObject("Canvas");
        Canvas canvas = canvasObject.AddComponent<Canvas>();
        canvas.renderMode = RenderMode.ScreenSpaceOverlay;
        canvasObject.AddComponent<CanvasScaler>();
        canvasObject.AddComponent<GraphicRaycaster>();

        // Create a new Text GameObject
        textObject = new GameObject("Text");
        textObject.transform.SetParent(canvasObject.transform);

        // Add a Text component to the text object
        instanceText = textObject.AddComponent<Text>();
        instanceText.font = Resources.GetBuiltinResource<Font>("Arial.ttf"); // Set the font
        instanceText.fontSize = 16; // Set the font size
        instanceText.color = Color.white; // Set the font color

        // Position the text object
        RectTransform rectTransform = textObject.GetComponent<RectTransform>();
        rectTransform.localPosition = Vector3.zero;
        rectTransform.sizeDelta = new Vector2(200, 200);
    }

    void Update()
    {
        // Update the position of the text object to match the screen position of the existing object
        Vector3 screenPos = Camera.main.WorldToScreenPoint(transform.position);
        instanceText.transform.position = screenPos;

        // Update the text to display the parent object's size
        Vector3 size = transform.localScale;
        instanceText.text = $"Size: {size.y}";
    }

    void OnDestroy()
    {
        // Destroy the text object when the parent object is destroyed
        Destroy(textObject);
    }
}
