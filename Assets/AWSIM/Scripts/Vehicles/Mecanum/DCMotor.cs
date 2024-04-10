using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;


namespace AWSIM
{
    /// <summary>
    /// Generic DC Motor Class.
    /// </summary>
    [System.Serializable]
    public class DCMotor : MonoBehaviour
    {
        /// <summary>
        /// Maximum Torque [Nm]. For mecanum drive motor, 37.7[kg cm f] = 3.67[Nm]
        /// </summary>
        [SerializeField] [Tooltip("Maximum Torque [Nm]")] float maximumTorque;

        /// <summary>
        /// Maximum Speed [rad/s]. For mecanum drive motor, 250[rpm] = 26.17993878[rad/s]
        /// </summary>
        [SerializeField] [Tooltip("Maximum Torque [rad/s]")]  float maximumSpeed;

        /// <summary>
        /// Maximum voltage [V]. For mecanum drive motor 24[V]
        /// </summary>
        [SerializeField] [Tooltip("Maximum voltage [V]")] public float MaximumVoltage;

        /// <summary>
        /// Armature resistance [ohm]. For mecanum drive motor, 0.37[ohm]
        /// </summary>
        [SerializeField] [Tooltip("Armature resistance [ohm]")] public float ArmatureResistance;

        /// <summary>
        /// Back-EMF constant [V/(rad/s)]. Rough estimate 0.916 for a mecanum drive moter.
        /// </summary>
        [SerializeField] [Tooltip("Back EMF constant [V/(rad/s)]")] public float BackEmfConstant;

        /// <summary>
        /// Back-EMF constant [V/(rad/s)]. Rough estimate 0.916 for a mecanum drive moter.
        /// </summary>
        [SerializeField] [Tooltip("Torque constant [Nm/A]")] public float TorqueConstant;

        public float Torque { get; private set; }
        public float Voltage;
        public float Current { get; private set; }
        public float Speed;

        public float Position;

        double timestamp_last;

        float speedLpfTimeConst;
        string label;
        
        int countDisp = 0;

        void Reset()
        {
            Current = 0;
            Torque = 0;
            timestamp_last = 0;
            speedLpfTimeConst = 0.01f;
        }

        void Awake()
        {
            Current = 0;
            Torque = 0;
            timestamp_last = 0;
            speedLpfTimeConst = 0.01f;

            countDisp = 0;
        }

        public void FixedUpdate()
        {
            // Calculate current
            Current = (Voltage - BackEmfConstant * Speed) / ArmatureResistance;
            Torque = BackEmfConstant * Current;

            if (Torque < - maximumTorque) {
                Torque = -maximumTorque;
            } else if (maximumTorque < Torque) {
                Torque = maximumTorque;
            }

            double timestamp_now = Time.timeAsDouble;
            if (timestamp_last != 0) {
                Position += Convert.ToSingle(Speed * (timestamp_now - timestamp_last)*90/Mathf.PI);
                timestamp_last = timestamp_now;
            }
            if (countDisp == 0) {
                Debug.Log(string.Format("[{0}] Voltage:{1}", label, Voltage));
            }
            if (100 <= ++countDisp) {
                countDisp = 0;
            }
        }
        public void SetLabel(string _label) {
            label = _label;
        }
        public string GetLabel() {
            return label;
        }

        public void SetSpeed(float _speed) {
            if (_speed < -maximumSpeed) {
                _speed = -maximumSpeed;
            } else if (maximumSpeed < _speed) {
                _speed = maximumSpeed;
            }
            Speed = (1-Time.fixedDeltaTime / speedLpfTimeConst) * Speed + Time.fixedDeltaTime / speedLpfTimeConst * _speed;
        }

        public void SetDuty(float _duty) {
            if (_duty < -1.0f) {
                _duty = -1.0f;
            } else if (1.0f < _duty) {
                _duty = 1.0f;
            }
            Voltage = _duty * MaximumVoltage;
            Debug.Log(string.Format("[{0}] dcMotor.SetDuty({1}) is called.", label, _duty));
        }
        public float GetDuty() {
            return Voltage / MaximumVoltage;
        }

        /// <summary>
        /// setTargetVelocity - Set velocity of motor (stab for Phidget API)
        /// </summary>
        //public void SetTargetVelocity(float _velocity)
        //{
        //    Voltage = MaximumVoltage * _velocity;
        //    Debug.Log("");
        //}

        /// <summary>
        /// getPosition - Get axial angle of motor (stab for Phidget API)
        /// </summary>
        /// <returns></returns>
        public float getPosition()
        {
            return Position;
        }

    }
}