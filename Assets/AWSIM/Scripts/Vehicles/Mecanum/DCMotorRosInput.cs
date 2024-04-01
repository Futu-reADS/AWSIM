using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using ROS2;
using System;

namespace AWSIM
{
    /// <summary>
    /// This class subscribes to the dutyRatioTopic and send the data to DCMotor class
    /// </summary>
    [RequireComponent(typeof(DCMotor))]
    public class DCMotorRosInput : MonoBehaviour
    {
        [SerializeField] string dutyRatioTopic = "/dcmotor/duty_ratio";
        [SerializeField] QoSSettings qosSettings = new QoSSettings();
        [SerializeField] DCMotor dcMotor;

        // subscribers.
        ISubscription<std_msgs.msg.Float64> dutyRatioSubscriber;

       void Reset()
        {
            if (dcMotor == null)
                dcMotor = GetComponent<DCMotor>();

            // initialize default QoS params.
            qosSettings.ReliabilityPolicy = ReliabilityPolicy.QOS_POLICY_RELIABILITY_RELIABLE;
            qosSettings.DurabilityPolicy = DurabilityPolicy.QOS_POLICY_DURABILITY_TRANSIENT_LOCAL;
            qosSettings.HistoryPolicy = HistoryPolicy.QOS_POLICY_HISTORY_KEEP_LAST;
            qosSettings.Depth = 1;
        }

        void Start()
        {
            var qos = qosSettings.GetQoSProfile();

            dcMotor.Voltage = 0;

            dutyRatioSubscriber
                = SimulatorROS2Node.CreateSubscription<std_msgs.msg.Float64>(
                    dutyRatioTopic, msg =>
                    {
                        dcMotor.Voltage =  Convert.ToSingle(msg) * dcMotor.MaximumVoltage;
                        Debug.Log("dcMotor.speed:"+dcMotor.Speed);
                    });
        }
       

        void OnDestroy()
        {
            SimulatorROS2Node.RemoveSubscription<std_msgs.msg.Float64>(dutyRatioSubscriber);
        }
    }
}