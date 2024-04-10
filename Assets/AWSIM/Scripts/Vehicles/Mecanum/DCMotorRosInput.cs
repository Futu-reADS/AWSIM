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
        [SerializeField] DCMotor dcMotor;
        [SerializeField] string dutyRatioTopic = "/dcmotor/duty_ratio";
        [SerializeField] string positionTopic = "/dcmotor/position";
        [SerializeField] QoSSettings qosSettings = new QoSSettings();

        // subscribers.
        ISubscription<std_msgs.msg.Float32> dutyRatioSubscriber;

        IPublisher<std_msgs.msg.Float32> positionPublisher;

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
                = SimulatorROS2Node.CreateSubscription<std_msgs.msg.Float32>(
                    dutyRatioTopic, msg =>
                    {
                        dcMotor.SetDuty(Convert.ToSingle(msg.Data));
                        //Debug.Log(string.Format("dcMotor({0}) voltage:{1} speed:{2}", dcMotor.GetLabel(), dcMotor.Voltage, dcMotor.Speed));
                    });

            positionPublisher = SimulatorROS2Node.CreatePublisher<std_msgs.msg.Float32>(positionTopic);
        }

        void Update()
        {
            std_msgs.msg.Float32 msg = new std_msgs.msg.Float32();
            msg.Data = dcMotor.getPosition();
            positionPublisher.Publish(msg);
        }
       

        void OnDestroy()
        {
            SimulatorROS2Node.RemoveSubscription<std_msgs.msg.Float64>(dutyRatioSubscriber);
        }
    }
}