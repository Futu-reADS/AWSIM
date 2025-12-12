using UnityEngine;
using UnityEngine.UI;

public class SpawnInstructions : MonoBehaviour
{
    private Text instructionText;

    void Start()
    {
        // Create a new Canvas
        GameObject canvasObject = new GameObject("Canvas");
        Canvas canvas = canvasObject.AddComponent<Canvas>();
        canvas.renderMode = RenderMode.ScreenSpaceOverlay;
        canvasObject.AddComponent<CanvasScaler>();
        canvasObject.AddComponent<GraphicRaycaster>();

        // Create a new Text object
        GameObject textObject = new GameObject("Text");
        textObject.transform.SetParent(canvasObject.transform);

        // Set the position
        RectTransform rectTransform = textObject.AddComponent<RectTransform>();
        rectTransform.anchorMin = new Vector2(0, 1); // top left corner
        rectTransform.anchorMax = new Vector2(0, 1); // top left corner
        rectTransform.pivot = new Vector2(0, 1); // top left corner
        rectTransform.anchoredPosition = new Vector2(10, -10); // 10 pixels from the top left corner
        rectTransform.sizeDelta = new Vector2(600, 200); // adjust this as needed

        // Set the text properties
        instructionText = textObject.AddComponent<Text>();
        instructionText.text = "2025-12-13 build ParcelPal simulator\n" +
                            "Press 'H' to spawn human (10s), 'V' for vehicle (10s)\n" +
                            "Press 'O' for object (height determined by mouse dragging) (60s)\n" +
                            "Press 'P' for 1[m]-tall object (60s)\n" +
                            "Press 'Z' to spawn human swarming around ego vehicle (20s)" +
                            "Press 'E' to disable/enable scenario\n" +
                            "Press 'L' to enable/disable longer(5min) destroy time";
        instructionText.font = Resources.GetBuiltinResource<Font>("Arial.ttf");
        instructionText.fontSize = 16; // adjust this as needed
        instructionText.color = Color.white; // adjust this as needed
    }
}
