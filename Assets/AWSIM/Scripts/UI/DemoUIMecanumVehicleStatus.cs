using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

namespace AWSIM
{
    public class DemoUIMecanumVehicleStatus : MonoBehaviour
    {
        [SerializeField]
        Text dutyRatioText;
        [SerializeField]
        Text angularVelocityText;

        [SerializeField]
        MecanumWheel mecanumWheelFL;
        [SerializeField]
        MecanumWheel mecanumWheelFR;
        [SerializeField]
        MecanumWheel mecanumWheelRL;
        [SerializeField]
        MecanumWheel mecanumWheelRR;

        // Start is called before the first FRame update
        void Start()
        {
            dutyRatioText.text = "MecWl.Dty[%]\r\n\r\n";
            angularVelocityText.text = "MecWl.Spd[rad/s]\r\n\r\n";
        }

        // Update is called once per FRame
        void Update()
        {
            dutyRatioText.text = string.Format("MecWl.Dty[%]\r\n{0,3:+#0#;-#0#} {1,3:+#0#;-#0#}\r\n{2,3:+#0#;-#0#} {3,3:+#0#;-#0#}",
                mecanumWheelFL.GetMotorDuty()*100, mecanumWheelFR.GetMotorDuty()*100, mecanumWheelRL.GetMotorDuty()*100, mecanumWheelRR.GetMotorDuty()*100);
            angularVelocityText.text = string.Format("MecWl.Spd[rad/s]\r\n{0,3:+#0.00#;-#0.00#} {1,3:+#0.00#;-#0.00#}\r\n{2,3:+#0.00#;-#0.00#} {3,3:+#0.00#;-#0.00#}", 
                mecanumWheelFL.GetAngularSpeed(), mecanumWheelFR.GetAngularSpeed(), mecanumWheelRL.GetAngularSpeed(), mecanumWheelRR.GetAngularSpeed());
        }
    }
}
