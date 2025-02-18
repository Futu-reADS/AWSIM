using UnityEngine;
using System.Collections.Generic;

public class RaycastTransparency : MonoBehaviour
{
    public Transform egoVehicle;
    public LayerMask obstacleLayer;
    public Camera cameraObject;
    public float fadeAmount = 0.5f;
    private Dictionary<Renderer, List<Color>> originalColors = new Dictionary<Renderer, List<Color>>();
    void Update()
    {
        Vector3 direction = egoVehicle.position - cameraObject.transform.position;
        Ray ray = new Ray(cameraObject.transform.position, direction);
        RaycastHit[] hits = Physics.RaycastAll(ray, direction.magnitude, obstacleLayer);

        HashSet<Renderer> currentHits = new HashSet<Renderer>();
        

        foreach (RaycastHit hit in hits)
        {
            Renderer renderer = hit.collider.GetComponent<Renderer>();
            if (renderer != null)
            {
                currentHits.Add(renderer);

                if (!originalColors.ContainsKey(renderer))
                {
                    List<Color> colors = new List<Color>();
                    foreach (Material mat in renderer.materials)
                    {
                        colors.Add(mat.color);
                    }
                    originalColors[renderer] = colors;
                }

                foreach (Material mat in renderer.materials)
                {
                    Color color = mat.color;
                    color.a = fadeAmount;
                    mat.color = color;
                }
            }
        }

        List<Renderer> toReset = new List<Renderer>();
        foreach (var entry in originalColors)
        {
            if (!currentHits.Contains(entry.Key))
            {
                toReset.Add(entry.Key);
            }
        }

        foreach (Renderer renderer in toReset)
        {
            List<Color> colors = originalColors[renderer];
            for (int i = 0; i < renderer.materials.Length; i++)
            {
                renderer.materials[i].color = colors[i];
            }
            originalColors.Remove(renderer);
        }
    }
}
