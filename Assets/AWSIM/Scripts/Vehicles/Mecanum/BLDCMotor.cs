using System.Collections;
using System.Collections.Generic;
using UnityEngine;


namespace AWSIM
{
    /// <summary>
    /// Vehicle's mecanum wheel class.
    /// Receive sleep control, steering tire angle and acceleration from the vehicle class.
    /// </summary>
    public class DCMotor : MonoBehaviour
    {
        [SerializeField] float maximumTorque;
        [SerializeField] float maximumSpeed;
        [SerializeField] float armatureResistance;
        [SerializeField] float voltageConstant;
        [SerializeField] float torqueConstant;
        [SerializeField] float viscousFriction;
        [SerializeField] float motorInertia;

        public float voltage;
        public float current { get; private set; }
        public float speed;
        public float torque { get; private set; }

        void Reset()
        {
        }

        void Awake()
        {
        }


        void Update()
        {
            // Calculate current
            current = (voltage - voltageConstant * speed) / armatureResistance;
            torque = torqueConstant * current;
            if (torque < - maximumTorque) {
                torque = -maximumTorque;
            } else if (maximumTorque < torque) {
                torque = maximumTorque;
            }
        }

    }
}