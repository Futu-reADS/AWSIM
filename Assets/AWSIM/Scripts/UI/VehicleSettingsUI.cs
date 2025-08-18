using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

namespace AWSIM
{
    /// <summary>
    /// Provides a UI for the vehicle.
    /// - Speed meter (km/h)
    /// - Automatic gear shift
    /// - Switch to manual contorl toggle
    /// </summary>
    public class VehicleSettingsUI : MonoBehaviour
    {
        // vehicle components
        [SerializeField] Vehicle vehicle;
        [SerializeField] VehicleKeyboardInput keyboardInput;
        [SerializeField] VehicleRosInput rosInput;

        bool suppressJoyPlug = false;

        // Enable keyboard input when toggle is on
        public void OnSwitchVehicleControl(bool isOn)
        {
            //keyboardInput.enabled = isOn;
            keyboardInput.active = isOn;
            rosInput.enabled = !isOn;

            //joystick.isActiveKeyboard = isOn;
            //joystick.isPluggedKeyboard = (!suppressJoyPlug && joystick.isActiveKeyboard);

            //Debug.Log("keyboardInput.enabled:" + keyboardInput.enabled);
        }

        public void OnSwitchSuppressJoystickEvent(bool isOn)
        {
            keyboardInput.suppressJoyPlugEvent = isOn;
            //joystick.isPluggedKeyboard = (!suppressJoyPlug && joystick.isActiveKeyboard);
        }
    }
}