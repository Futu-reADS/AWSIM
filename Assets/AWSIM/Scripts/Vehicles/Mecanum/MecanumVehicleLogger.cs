using System.Collections;
using System.Collections.Generic;
using System.IO;
using System.Text;
using UnityEngine;

namespace AWSIM {

[RequireComponent(typeof(MecanumVehicle))]
[RequireComponent(typeof(MecanumWheel))]
public class MecanumVehicleLogger : MonoBehaviour
{
    private string positionDataFilePath;
    [SerializeField] MecanumWheel[] wheels;
    [SerializeField] MecanumVehicle vehicle;
    private StreamWriter file;
    void Start()
    {
#if false
        positionData = new List<string>();
        positionDataFilePath = Path.Combine(Application.persistentDataPath, "SimulationLog.csv");
#endif
        file = new StreamWriter(new FileStream(positionDataFilePath, FileMode.Create), Encoding.UTF8);
        // Write the header line to the position data file
        file.WriteLine("TimeStamp,WheelLfSpeed");
    }
    void Update()
    {
#if false 
       // Fetch the positions of the VR headset and stylus
        Vector3 currentHeadsetPosition = Head.transform.position;
        Vector3 currentStylusPosition = Stylus.transform.position;
        // Format the current positions into a string for the CSV file

        string positionDataLine = string.Format(
            "{0},{1},{2},{3},{4},{5},{6}",
            DateTime.Now,

            currentStylusPosition.x, currentStylusPosition.y, currentStylusPosition.z,
            currentHeadsetPosition.x, currentHeadsetPosition.y, currentHeadsetPosition.z
        );
        file.WriteLine(positionDataLine);
#endif
    }
    void OnApplicationQuit()
    {
        file.Flush();
        file.Close();
    }
}
}
