using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace AWSIM
{
    /// <summary>
    /// The Vehicle Mode Manager that manages the control mode of ParcelPal
    /// </summary>
    [RequireComponent(typeof(Vehicle))]
    public class VehicleModeManager : MonoBehaviour
    {
        [SerializeField] Vehicle vehicle;
        [SerializeField] EstopButton estopButton;
        [SerializeField] VehicleKeyboardInput vehicleKeyboardInput;

        public enum ModeValue {
            AUTO = 1,
            MANUAL = 4,
            STOP = 5
        };
        public ModeValue ControlMode { get; private set; }

        void Awake()
        {
            ControlMode = ModeValue.AUTO;
        }

        void FixedUpdate()
        {
            if (estopButton.ToggleState)
            {
                ControlMode = ModeValue.STOP;
                return;
            }

            switch (ControlMode)
            {
                case ModeValue.AUTO:
                    if (vehicleKeyboardInput.active && !vehicleKeyboardInput.suppressJoyPlugEvent)
                    {
                        ControlMode = ModeValue.MANUAL;
                    }
                    break;
                case ModeValue.MANUAL:
                    if (!vehicleKeyboardInput.active || vehicleKeyboardInput.suppressJoyPlugEvent)
                    {
                        ControlMode = ModeValue.AUTO;
                    }
                    break;
                case ModeValue.STOP:
                    if (!estopButton.ToggleState)
                    {
                        ControlMode = ModeValue.MANUAL;
                    }
                    break;
            }
        }
    }
}
