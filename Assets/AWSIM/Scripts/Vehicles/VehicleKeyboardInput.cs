using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using ROS2;
using System;

namespace AWSIM
{
    /// <summary>
    /// This is a sample class for controlling a vehicle with a keyboard.
    /// </summary>

    // ----- key binds -----
    // up arrow : Accelerate
    // down arrow : Deceleration
    // left/right arrow : Steering

    // (actions to keys below are not implemented)
    // D : Drive gear
    // P : Parking gear
    // R : Reverse gear
    // N : Neutral gear
    // 1 : Left turn signal
    // 2 : Right turn signal
    // 3 : Hazard
    // 4 : Turn signal off
    [RequireComponent(typeof(Vehicle))]
    public class VehicleKeyboardInput : MonoBehaviour
    {
        [SerializeField] Vehicle vehicle;

        [SerializeField] float maxAcceleration = 1.5f;
        [SerializeField] float maxSteerAngle = 35;

        [SerializeField] string joystickPluggerTopic = "/vehicle_interface/ifb_driver/joystick";

        IPublisher<std_msgs.msg.Bool> joystickPlugStatePublisher;

        [SerializeField] QoSSettings qosSettings = new QoSSettings();

        float joyCommandAcceleration = 0;
        float joyCommandSteerAngle = 0;

        public bool active = false;
        bool activePrev = false;

        public bool suppressJoyPlugEvent = false;

        void Reset()
        {
            if (vehicle == null)
                vehicle = GetComponent<Vehicle>();

            // initialize default QoS params.
            qosSettings.ReliabilityPolicy = ReliabilityPolicy.QOS_POLICY_RELIABILITY_RELIABLE;
            qosSettings.DurabilityPolicy = DurabilityPolicy.QOS_POLICY_DURABILITY_TRANSIENT_LOCAL;
            qosSettings.HistoryPolicy = HistoryPolicy.QOS_POLICY_HISTORY_KEEP_LAST;
            qosSettings.Depth = 1;
        }

        void Start()
        {
            var qos = qosSettings.GetQoSProfile();

            joystickPlugStatePublisher = SimulatorROS2Node.CreatePublisher<std_msgs.msg.Bool>(joystickPluggerTopic, qos);
        }


        void Update()
        {

            // get arrow inputs
            joyCommandSteerAngle = Input.GetAxis("Horizontal");
            joyCommandAcceleration = Input.GetAxis("Vertical");

            std_msgs.msg.Bool joystickPluggedMsg = new std_msgs.msg.Bool
            {
                Data = active && !suppressJoyPlugEvent
            };
            joystickPlugStatePublisher.Publish(joystickPluggedMsg);


            if (active)
            {
                if (!activePrev)
                {
                    vehicle.AccelerationInput = -3.0f;
                }
                else
                {
                    //vehicle.AccelerationInput = Mathf.Abs(joyCommandAcceleration);
                    float speed = vehicle.Speed;
                    if (0 < speed)
                    {
                        vehicle.AutomaticShiftInput = Vehicle.Shift.DRIVE;
                        if (joyCommandAcceleration < 0)
                        {
                            joyCommandAcceleration *= 3.0f;
                        }
                        vehicle.AccelerationInput = joyCommandAcceleration;
                    }
                    else if (speed == 0)
                    {
                        var shift = (0 < joyCommandAcceleration ? Vehicle.Shift.DRIVE :
                                                       joyCommandAcceleration < 0 ? Vehicle.Shift.REVERSE :
                                                       Vehicle.Shift.PARKING);
                        var accInput = Mathf.Abs(joyCommandAcceleration);
                        vehicle.AutomaticShiftInput = shift;
                        vehicle.AccelerationInput = accInput;
                        Debug.Log("[speed=0] shift:" + shift + " accInput:" + accInput);
                    }
                    else
                    {
                        vehicle.AutomaticShiftInput = Vehicle.Shift.REVERSE;
                        if (0 < joyCommandAcceleration)
                        {
                            joyCommandAcceleration *= 3.0f;
                        }
                        vehicle.AccelerationInput = -joyCommandAcceleration;
                    }
                    vehicle.SteerAngleInput = (float)joyCommandSteerAngle * Mathf.Rad2Deg;
                    Debug.Log("[UPD] Spd:" + speed + " Shift:" + vehicle.AutomaticShiftInput + " Acc:" + vehicle.AccelerationInput + " Steer:" + vehicle.SteerAngleInput);
                }
            }
            else
            {
                if (activePrev)
                {
                    vehicle.AccelerationInput = -3.0f;
                }
            }
            activePrev = active;

            // set gear
            /*
            if (Input.GetKey(KeyCode.D))
                vehicle.AutomaticShiftInput = Vehicle.Shift.DRIVE;
            else if (Input.GetKey(KeyCode.P))
                vehicle.AutomaticShiftInput = Vehicle.Shift.PARKING;
            else if (Input.GetKey(KeyCode.R))
                vehicle.AutomaticShiftInput = Vehicle.Shift.REVERSE;
            else if (Input.GetKey(KeyCode.N))
                vehicle.AutomaticShiftInput = Vehicle.Shift.NEUTRAL;

            // set turn signal
            if (Input.GetKey(KeyCode.Alpha1))
                vehicle.SignalInput = Vehicle.TurnSignal.LEFT;
            else if (Input.GetKey(KeyCode.Alpha2))
                vehicle.SignalInput = Vehicle.TurnSignal.RIGHT;
            else if (Input.GetKey(KeyCode.Alpha3))
                vehicle.SignalInput = Vehicle.TurnSignal.HAZARD;
            else if (Input.GetKey(KeyCode.Alpha4))
                vehicle.SignalInput = Vehicle.TurnSignal.NONE;
                */
        }
    }
}