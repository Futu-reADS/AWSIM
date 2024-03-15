using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using ROS2;
using System;

namespace AWSIM
{
    /// <summary>
    /// This class subscribes to the vehicleCommand and turnSignal  msg output from Autoware to ROS, 
    /// and after converting the msg, it inputs it to the Vehicle class of E2ESimualtor.
    /// </summary>
    [RequireComponent(typeof(MecanumWheel))]
    public class MecanumWheelRosInput : MonoBehaviour
    {
        [SerializeField] string vehicleEmergencyStampedTopic = "/control/command/emergency_cmd";
        [SerializeField] string cmdWheelSpeedCorrectionTopic = "/cmd_wheel_speed_correction";

        [SerializeField] QoSSettings qosSettings = new QoSSettings();
        [SerializeField] MecanumWheel wheel;

        // subscribers.
        ISubscription<std_msgs.msg.Float64> cmdWheelSpeedCorrectionSubscriber;

        // Latest Emergency value.
        // If emergency is true, emergencyDeceleration is applied to the vehicle's deceleration.
        // TODO: In case of reverse gear?
        bool isEmergency = false;
        float emergencyDeceleration = -3.0f; // m/s^2

        float cmdWheelSpeed;

        void Reset()
        {
            if (wheel == null)
                wheel = GetComponent<MecanumWheel>();

            // initialize default QoS params.
            qosSettings.ReliabilityPolicy = ReliabilityPolicy.QOS_POLICY_RELIABILITY_RELIABLE;
            qosSettings.DurabilityPolicy = DurabilityPolicy.QOS_POLICY_DURABILITY_TRANSIENT_LOCAL;
            qosSettings.HistoryPolicy = HistoryPolicy.QOS_POLICY_HISTORY_KEEP_LAST;
            qosSettings.Depth = 1;
        }

        /// <summary>
        /// Processes the TurnSignal to be applied to the vehicle from the latest turnIndicatorsSignal and hazardLightsSignal values.
        /// Priority : HAZARD > LEFT/RIGHT > NONE
        /// </summary>
        void UpdateVehicleTurnSignal()
        {
        }

        void Start()
        {
            var qos = qosSettings.GetQoSProfile();

            cmdWheelSpeed = 0;

            cmdWheelSpeedCorrectionSubscriber
                = SimulatorROS2Node.CreateSubscription<std_msgs.msg.Float64>(
                    cmdWheelSpeedCorrectionTopic, msg =>
                    {
                        wheel.motorVoltage =  Convert.ToSingle(msg);
                        Debug.Log("cmdWheelSpeed:"+cmdWheelSpeed);
                    });
        }

        

        void OnDestroy()
        {
            SimulatorROS2Node.RemoveSubscription<std_msgs.msg.Float64>(cmdWheelSpeedCorrectionSubscriber);
        }
    }
}