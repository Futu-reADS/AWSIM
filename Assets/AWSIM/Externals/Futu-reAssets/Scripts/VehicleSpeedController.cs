using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace AWSIM
{
    /// <summary>
    /// The Vehicle Speed Controller that generates AccelerationInput to Vehicle object from speed command
    /// </summary>
    [RequireComponent(typeof(Vehicle))]
    public class VehicleSpeedController : MonoBehaviour
    {
        [SerializeField] Vehicle vehicle;
        [SerializeField] EstopButton estopButton;

        [Header("PI Controller Gains")]

        // proportional gain to generate acceleration command from difference in speed command versus actual speed.
        [Range(0, 10.0f)]
        [SerializeField] public float ProportionalGain = 1.5f;
        // integral gain to generate acceleration command from difference in speed command versus actual speed.
        [Range(0, 2.0f)]
        [SerializeField] public float IntegralGain = 0.125f;

        public float SpeedInput = 0.0f;
        public float integralTerm = 0;
        void Awake()
        {
        }

        void FixedUpdate()
        {
            if (estopButton.ToggleState)
            {
                // Clear integral on forced stoppage
                integralTerm = 0;
                return;
            }
            // Generate accelerationInput from SpeedInput
            var actualSpeed = ROS2Utility.UnityToRosPosition(vehicle.LocalVelocity).x;
            var deltaSpeed   = Mathf.Abs(SpeedInput) - Mathf.Abs(actualSpeed);
            vehicle.AccelerationInput = ProportionalGain * deltaSpeed + IntegralGain * integralTerm;
            integralTerm += IntegralGain * deltaSpeed;
            integralTerm = Mathf.Clamp(integralTerm, 0, 5.0f);
        }

        void OnEnable()
        {
            integralTerm = 0;
            Debug.Log("OnEnable() is called."); 
        }
        void OnDisable()
        {
            integralTerm = 0;
            Debug.Log("OnDisable() is called."); 
        }
    }
}
