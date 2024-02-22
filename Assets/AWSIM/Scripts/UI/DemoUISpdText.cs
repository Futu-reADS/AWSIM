using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

namespace AWSIM
{
    public class DemoUISpdText : MonoBehaviour
    {
        [SerializeField]
        Text lonSpdText;

        [SerializeField]
        Text latSpdText;

        [SerializeField]
        Text angSpdText;
        
        [SerializeField]
        public MecanumVehicle vehicle;

        // Start is called before the first frame update
        void Start()
        {
            
        }

        // Update is called once per frame
        void Update()
        {
            Vector3 velLocal = vehicle.LocalVelocity;
            float angularVelocity = (-vehicle.AngularVelocity.y);

            lonSpdText.text = string.Format("Lon.spd[m/s]  : {0, 5:F2}", velLocal.z);
            latSpdText.text = string.Format("Lat.spd[m/s]  : {0, 5:F2}", -velLocal.x);
            angSpdText.text = string.Format("Ang.spd[rad/s]: {0, 5:F2}", angularVelocity);
        }
    }
}
