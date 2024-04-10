using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace AWSIM
{
    /// <summary>
    /// The Mecanum Vehicle class.
    /// </summary>
    // ----- VEHICLE DYNAMICS CONCEPT -----
    // This vehicle model was created for Autoware simulation,
    // and assuming that Autoware has already created a gas pedal map,
    // this vehicle model uses acceleration as an input value.
    // It has the following features.
    // - Acceleration (m/s^2) control as per command value.
    // - Turning geometry equivalent to 2-wheel model.
    // - 3D mesh (fbx) as road surface for vehicle driving, gradient resistance.
    // - Vehicle attitude change by unity physX engine.
    //   - Yaw, roll, pitch
    //   - Mass-spring damper suspension (WheelCollider's Suspension)
    //
    // ----- VEHICLE INPUT PARAMETER -----
    // The acceleration or deceleration of the vehicle is determined by AutomaticShiftInput and AccelerationInput.
    // And the vehicle will not move in the opposite direction of the Shift(D or R) input.
    // It has the following parameters
    // - Vehicle gear shift input (AT). PARKING, REVERSE, NEUTRAL, DRIVE.
    // - Acceleration input (m/s^2).
    // - Vehicle steering input. Tire angle (degree).
    // - Vehicle turn signal input. NONE, LEFT, RIGHT, HAZARD.
    // For example, here is how the SAMPLE behaves
    // SAMPLE 1,
    //   Shift = D
    //   Speed = Any value
    //   AccelerationInput > 0
    // it will accelerate with input values. (Gradient resistance is received)
    // --
    // SAMPLE 2,
    //   Shift = D
    //   Speed > 0
    //   AccelerationInput < 0
    // it will decelerate (like a brake.).
    // --
    // SAMPLE 3,
    //   Shift = D
    //   Speed <= 0
    //   AccelerationInput < 0
    // it will continuous stop.
    //--
    // 
    // ----- VEHICLE DYNAMICS PARAMETER -----
    // For more advanced vehicle configuration, the following parameters need to be provided.
    //   [Need Vehicle Params]                            [Using With Unity]
    // - vehicle weight (kg)                            : RigidBody mass
    // - Wheel base (m)                                 : Wheel position
    // - Tread width (m)                                : Wheel position
    // - Center of Mass position (x,y,z) (m)            : Rigidbody center of mass position
    // - Moment of inertia (pitch, roll, yaw) (kgm^2)   : RigiBody ineria
    // - Spring rate (N)                                : Wheel suspension
    // - Damper rate (N/s)                              : Wheel suspension
    // - wheel radius (m)                               : Wheel settings

    // TODO: Write detailed documentation about the vehicle.
    public class MecanumVehicle : MonoBehaviour
    {
        public enum Shift
        {
            PARKING = 0,
            REVERSE = 1,
            NEUTRAL = 2,
            DRIVE = 3,
        }

        public enum TurnSignal
        {
            NONE = 0,
            LEFT = 1,
            RIGHT = 2,
            HAZARD = 3,
        }

        /// <summary>
        /// Axle class.
        /// </summary>
        // TODO: Expanded number of wheels and visual support.
        // TODO: Selecting a wheel to apply steering.
        [Serializable]
        public class Axle
        {
            [SerializeField] MecanumWheel leftWheel;
            [SerializeField] MecanumWheel rightWheel;

            /// <summary>
            /// Left wheel of vehicle
            /// </summary>
            public MecanumWheel LeftWheel => leftWheel;

            /// <summary>
            /// Right wheel of vehicle
            /// </summary>
            public MecanumWheel RightWheel => rightWheel;
        }

        /// <summary>
        /// Kinamtic control parameter
        /// </summary>
        [Serializable]
        public class ControlParam
        {
            [SerializeField] public float wheelTreadLength = 0.3f;
            [SerializeField] public float wheelBaseLength = 0.3f;
            [SerializeField] public float wheelRadius = 0.075f;
            [SerializeField] public static bool useCmdVelFeedback = false;    // True: enable closed-loop feed back for cmd_vel  False: just use the duty command comming from phidget lib
            [SerializeField] public float Kp = 0.00001f;
            [SerializeField] public float Ki = 0.0f;
            [SerializeField] public float c = 1.0f; // 33.5f;

            public float wheelGeometry() {
                return (wheelTreadLength + wheelBaseLength) / 2.0f;
            }
            
        }

        public class Twist2D
        {
            public Vector2 Linear;
            public float Angular;
            public Twist2D() {
                Linear.x = 0.0f;
                Linear.y = 0.0f;
                Angular = 0.0f;
            }
            public static Twist2D operator +(Twist2D t1, Twist2D t2) {
                var ret = new Twist2D();
                ret.Linear = t1.Linear + t2.Linear;
                ret.Angular = t1.Angular + t2.Angular;
                return ret;
            }
            public static Twist2D operator -(Twist2D t1, Twist2D t2) {
                var ret = new Twist2D();
                ret.Linear = t1.Linear - t2.Linear;
                ret.Angular = t1.Angular - t2.Angular;
                return ret;
            }
            public static Twist2D operator *(float c, Twist2D t) {
                var ret = new Twist2D();
                ret.Linear = c*t.Linear;
               ret.Angular = c*t.Angular;
                return ret;
            }
        }

        [Header("Vehicle Settings")]

        // Center of mass of rigidbody.
        // If null, the default value will be used.
        [SerializeField] Transform centerOfMassTransform;

        // Set inertia?
        [SerializeField] bool useInertia;

        // Moment of inertia applied to the vehicle (x, y, z).
        // reference sample (in 1998): https://www.researchgate.net/publication/228945609_Measured_Vehicle_Inertial_Parameters-NHTSA
        [SerializeField] Vector3 inertia;

        [Header("Physics Settings (experimental)")]

        // Threshold for Rigidbody Sleep (m/s).
        [SerializeField] float sleepVelocityThreshold;

        // Time to Rigidbody Sleep (sec).
        [SerializeField] float sleepTimeThreshold;

        // Coefficient for prevent skidding while stopping.
        // Applies to each wheel.
        // TODO: A more accurate calculation method.
        [Range(0.05f, 1f)] [SerializeField] float SkiddingCancelRate;

        [Space()]
        [Header("Axles Settings")]
        [SerializeField] Axle frontAxle;
        [SerializeField] Axle rearAxle;

        [Header("Input Settings")]
        // Set value to clamp SteerAngleInput (degree).
        // -MaxSteerAngleInput <= SteerAngleInput <= MaxSteerAngleInput.
        [Range(0.01f, 80)]
        [SerializeField] float MaxSteerAngleInput = 35f;

        // Set value to clamp AccelerationInput (m/s^2).
        // -MaxAccelerationInput <= AccelerationInput <= MaxAccelerationInput.
        [Range(0.01f, 50)]
        [SerializeField] float MaxAccelerationInput = 10;

        // Closed-loop control parameter
        [Header("Control Parameters")]
        [SerializeField] ControlParam controlParam;

        [Header("Inputs")]

        /// <summary>
        /// Vehicle gear shift input (AT). PARKING, REVERSE, NEUTRAL, DRIVE.
        /// </summary>
        public Shift AutomaticShiftInput;

        /// <summary>
        /// Acceleration input (m/s^2).
        /// In the plane, output the force that will result in this acceleration.
        /// On a slope, it is affected by the slope resistance, so it does not match the input.
        /// </summary>
        // TODO: Compute first order lag
        public float AccelerationInput;

        /// <summary>
        /// Vehicle steering input. Tire angle (degree)
        /// Negative is left, positive is right turn tire angle.
        /// </summary>
        // TODO: Compute first order lag
        public float SteerAngleInput;

        /// <summary>
        /// Vehicle turn signal input. NONE, LEFT, RIGHT, HAZARD.
        /// </summary>
        public TurnSignal SignalInput;

        /// <summary>
        /// Acceleration(m/s^2) in the local coordinate system of the vehicle.
        /// </summary>
        public Vector3 LocalAcceleration { get; private set; }

        /// <summary>
        /// Vehicle speed (m/s)
        /// </summary>
        public float Speed { get; private set; }

        /// <summary>
        /// Vehicle steering angle (degree)
        /// </summary>
        public float SteerAngle => SteerAngleInput;

        /// <summary>
        /// Vehicle turn signal
        /// </summary>
        public TurnSignal Signal => SignalInput;

        /// <summary>
        /// Vehicle velocity (m/s)
        /// </summary>
        public Vector3 Velocity => m_rigidbody.velocity;

        /// <summary>
        /// Vehcile local velocity (m/s)
        /// </summary>
        public Vector3 LocalVelocity => m_transform.InverseTransformDirection(Velocity);

        /// <summary>
        /// Vehicle angular velocity (rad/s)
        /// </summary>
        public Vector3 AngularVelocity { get; private set; }

        public Twist2D CmdVel;

        [HideInInspector] public bool UseCmdVel = false;

        private float sleepTimer = 0.0f; ///Count the time until CanSleep is switched to true

        Twist2D estimatedVel;
        Twist2D integDiffVel;

        float[] wheelAcceleration;
        float[] lastWheelAccelaration;


        // Cache components.
        MecanumWheel[] wheels;
        Rigidbody m_rigidbody;
        Transform m_transform;

        // Cache previous frame values.
        Vector3 lastVelocity;
        Vector3 lastPosition;
        Quaternion lastRotation;

        // Sleep position & rotation
        Vector3 sleepPositon;
        Quaternion sleepRotation;
        bool lastSleep;

        void Awake()
        {
            m_rigidbody = GetComponent<Rigidbody>();
            m_transform = transform;
            Debug.Log("TIME:" + Time.time + "MecanumVehicles.Awake() is called");

            wheels = new MecanumWheel[] { frontAxle.LeftWheel, frontAxle.RightWheel, rearAxle.LeftWheel, rearAxle.RightWheel };
            CmdVel = new Twist2D();
            estimatedVel = new Twist2D();
            integDiffVel = new Twist2D();
            wheelAcceleration = new float[] { 0.0f, 0.0f, 0.0f, 0.0f };

            // Set center of mass position.
            if (centerOfMassTransform != null)
                m_rigidbody.centerOfMass = m_transform.InverseTransformPoint(centerOfMassTransform.position);

            // Set inertia values.
            if (useInertia)
                m_rigidbody.inertiaTensor = inertia;

            Debug.Log("TIME:" + Time.time + "MecanumVehicles.Awake() has stopped");
        }

        int ComputeVehicleStateCnt = 0;
        int EstimateVelocityDispCnt = 0;
        void FixedUpdate()
        {
            // Clamp input values.
            AccelerationInput = Mathf.Clamp(AccelerationInput, -MaxAccelerationInput, MaxAccelerationInput);
            SteerAngleInput = Mathf.Clamp(SteerAngleInput, -MaxSteerAngleInput, MaxSteerAngleInput);

            // Compute vehicle infomation.
            ComputeVehicleState();

            // Update Steering, WheelHit, CancelForceRate of the wheel.
            PreUpdateWheels();

            // Sleep ?
            var sleep = CanSleep();
            UpdateVehicleSleep(sleep);
            

            //Debug.Log("sleep:" + sleep);;
            if (sleep == false)
            {
                // Update wheel force.
                //var acceleration = AccelerationInput;
                //var acceleration = 0.1 * (CmdVelLinear.x - (Quaternion.Inverse(m_rigidbody.transform.rotation)*m_rigidbody.velocity).x);

                UpdateWheelsForce(); // Convert.ToSingle(acceleration));
            }

            // cache valpdatee for next frame.
            lastVelocity = m_rigidbody.velocity;
            lastPosition = m_transform.position;
            lastRotation = m_transform.rotation;
            lastSleep = sleep;

            // ----- inner methods -----
            void ComputeVehicleState()
            {
                //m_rigidbody.velocity = CmdVelLinear;
                //AngularVelocity = CmdVelAngular;
                if (ComputeVehicleStateCnt == 0) {
                    // Debug.Log("CmdVel:("+CmdVel.Linear.x+", "+CmdVel.Linear.y+", "+CmdVel.Angular+")");;
                    // Debug.Log("m_rigidbody.velocity:("+m_rigidbody.velocity.x+", "+m_rigidbody.velocity.y+", "+m_rigidbody.velocity.z+")");
                }
                if (++ComputeVehicleStateCnt >= 100) {
                    ComputeVehicleStateCnt = 0;
                }

                // Speed.
                Vector3 velocity = Velocity;
                Vector3 forward = m_transform.forward;
                Speed = Vector3.Dot(velocity, forward);

                // Local acceleration.
                Vector3 acceleration = (velocity - lastVelocity) / Time.deltaTime;
                LocalAcceleration = m_transform.InverseTransformDirection(acceleration);

                // Angular velocity.
                AngularVelocity = ((transform.rotation.eulerAngles - lastRotation.eulerAngles) / Time.deltaTime) * Mathf.Deg2Rad;
            }

            void PreUpdateWheels()
            {
                // Steer angle is front-only.
                //frontAxle.LeftWheel.UpdateWheelSteerAngle(SteerAngle);
                //fronMecanumVehicle.USE_MOTOR_DYNAMICStAxle.RightWheel.UpdateWheelSteerAngle(SteerAngle);

                foreach (var wheel in wheels)
                {
                    wheel.UpdateWheelHit();
                    //wheel.UpdateSkiddingCancelRate(SkiddingCancelRate);
                }

            }

            bool CanSleep()
            {
                return false;
                //return false;
                if (IsCanSleepVelocity() && IsCanSleepInput()) {
                    sleepTimer += Time.deltaTime;
                    if (sleepTimer >= sleepTimeThreshold)
                        return true;
                    else
                        return false;
                }
                else
                {
                    sleepTimer = 0.0f;
                    return false;
                }

                // Is less than sleepVelocityThreshold ?
                bool IsCanSleepVelocity()
                {
                    if (Mathf.Abs(Velocity.z) < sleepVelocityThreshold)
                        return true;
                    else
                        return false;
                }


                // Is input gear & acceleration can sleep ?
                bool IsCanSleepInput()
                {
                    int idx;

                    if (UseCmdVel) {
                        if (CmdVel.Linear.x == 0 && CmdVel.Linear.y == 0 && CmdVel.Angular == 0) {
                            return true;
                        }
                        return false;
                    } else {
                        for (idx=0; idx < wheels.Length; idx++) {
                            if (wheels[idx].dcMotor.Voltage != 0) {
                                return false;
                            }
                        }
                    }
                    return true;
                }

            }

            void UpdateVehicleSleep(bool isSleep)
            {
                if (isSleep == true && lastSleep == false)
                {
                    sleepPositon = transform.position;
                    sleepRotation = transform.rotation;
                }

                // Vehicle sleep.
                if (isSleep)
                {
                    if (m_rigidbody.IsSleeping())
                        return;

                    m_rigidbody.Sleep();
                    m_rigidbody.constraints = RigidbodyConstraints.FreezeAll;

                    transform.position = sleepPositon;
                    transform.rotation = sleepRotation;
                }
                else
                {
                    m_rigidbody.constraints = RigidbodyConstraints.None;

                    if (m_rigidbody.IsSleeping())
                    {
                        m_rigidbody.WakeUp();
                    }
                }


                // Wheel sleep.
                foreach (var wheel in wheels)
                {
                    if (wheel.IsSleep != isSleep)
                        wheel.UpdateWheelSleep(isSleep);
                }


            }

            void UpdateWheelsForce()
            {
                if (ControlParam.useCmdVelFeedback) {
                    CalculateAccelByFeedback();
                } else if (UseCmdVel) {
                    ComputeInverseKinematic(wheelAcceleration, CmdVel);    // open loop control
                    Debug.Log("CmdVel[ "+CmdVel.Linear.x+", " + CmdVel.Linear.y+", "+CmdVel.Angular+" ]");
                }

                int idx;
                for (idx=0; idx < wheels.Length; idx++) {
                    wheels[idx].UpdateSpeed();
                }
                for (idx=0; idx < wheels.Length; idx++) {
                    if (ControlParam.useCmdVelFeedback) {
                        wheels[idx].SetWheelAcceleration(wheelAcceleration[idx]);
                    } else if (UseCmdVel) {
                        wheels[idx].SetMotorDuty(wheelAcceleration[idx]);
                    }
                    wheels[idx].UpdateWheelForce();
                }
                if (false) {
                    Debug.Log("TIME:" + Time.time + ", wheelForce," +
                    wheels[0].acceleration    + "," + wheels[1].acceleration    + "," + wheels[2].acceleration    + "," + wheels[3].acceleration + ", wheelSpeed," +
                    wheels[0].dcMotor.Speed + "," + wheels[1].dcMotor.Speed + "," + wheels[2].dcMotor.Speed + "," + wheels[3].dcMotor.Speed);
                }
            }

            void CalculateAccelByFeedback() {
                EstimateVelocity();
                var diffVel = new Twist2D();
                var acceleration = new Twist2D();
                diffVel = CmdVel - estimatedVel;
                //Debug.Log("diffVel[ "+diffVel.Linear.x+", " + diffVel.Linear.y+", "+diffVel.Angular+" ]");
                integDiffVel = integDiffVel + Time.deltaTime * diffVel;
                acceleration = controlParam.Kp * diffVel + controlParam.Ki * integDiffVel;
                ComputeInverseKinematic(wheelAcceleration, acceleration); 
            }

            void ComputeInverseKinematic(float[] perWheelData, Twist2D t) {
                // FL, FR, RL, RR
                perWheelData[0] =  ((t.Linear.x - t.Linear.y - t.Angular*controlParam.wheelGeometry())/controlParam.wheelRadius)/controlParam.c;
                perWheelData[1] = -((t.Linear.x + t.Linear.y + t.Angular*controlParam.wheelGeometry())/controlParam.wheelRadius)/controlParam.c;
                perWheelData[2] =  ((t.Linear.x + t.Linear.y - t.Angular*controlParam.wheelGeometry())/controlParam.wheelRadius)/controlParam.c;
                perWheelData[3] = -((t.Linear.x - t.Linear.y + t.Angular*controlParam.wheelGeometry())/controlParam.wheelRadius)/controlParam.c;
            } 

            void EstimateVelocity() {
                var linear = ROS2Utility.UnityToRosPosition(LocalVelocity);
                estimatedVel.Linear.x = linear.x;
                estimatedVel.Linear.y = linear.y;
                estimatedVel.Angular = ROS2Utility.UnityToRosPosition(-AngularVelocity).z;
                
                if (EstimateVelocityDispCnt == 0) {
                    // Debug.Log("estimatedVel:("+estimatedVel.Linear.x+", "+estimatedVel.Linear.y+", "+linear.z+", "+estimatedVel.Angular+")");;
                }
                if (++EstimateVelocityDispCnt >= 100) {
                    EstimateVelocityDispCnt = 0;
                }

            }

        }
    }
}
