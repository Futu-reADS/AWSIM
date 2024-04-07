using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;


namespace AWSIM
{
    /// <summary>
    /// Vehicle's mecanum wheel class.
    /// Receive sleep control, steering tire angle and acceleration from the vehicle class.
    /// </summary>
    [RequireComponent(typeof(WheelCollider))]
    public class MecanumWheel : MonoBehaviour
    {
        [SerializeField] WheelCollider wheelCollider;
        [SerializeField] Transform wheelVisualTransform;
        [SerializeField] string label;

        /// <summary>
        /// Is the wheel in contact with the ground?
        /// </summary>
        public bool IsGrounded { get; private set; }

        /// <summary>
        /// Is the wheel in sleep?
        /// </summary>
        public bool IsSleep { get; private set; }

        /// <summary>
        /// Radius of wheel
        /// </summary>
        [SerializeField] float radius;

        /// <summary>
        /// Angle (in rad) of the axis of the roller that touches the ground with respect to the axis of the wheel 
        /// Unit: degree, positive value clock-wise (FL, RR: -45  FR, RL: 45)
        /// </summary>
        [SerializeField] float rollerAxisAngleDeg;
        float rollerAxisAngle;

        /// <summary>
        /// DC Motor
        /// </summary>
        public DCMotor dcMotor;
    
        public float acceleration;

        // latest wheel hit. This value updated according to the FixedUpdate() of the vehicle.
        WheelHit wheelHit;

        // Cached rigidbody of the vehicle to which the wheelcollider is attached.
        Rigidbody vehicleRigidbody;

        // Traction force multiplier relative to force from motor
        // | F_traction | = tractionCoeff x | F_wheel |
        float tractionForceMultiplier; 

        // Direction of roller's rotation axis in local coordinate
        Vector3 rollerRotationAxisLocal;

        // Direction of wheel's rotation axis in local coordinate
        Vector3 wheelRotationAxisLocal;

        // Direction of wheel's forward axis in local coordinate
        Vector3 wheelForwardAxisLocal;

        Quaternion qtWorldToVehicle;
        Quaternion qtVehicleToWorld;
        Quaternion qtRotAroundZAxisLocal;

        // K_factor
        float sinRollerAngle;
        float cosRollerAngle;

        // Coefficient for cancelling the skidding while stopping of the tire.
        float skiddingCancelRate;

        // Flag to determine whether to use motor model 
        //public bool useMotorModel = false;

        // Last drive force
        public Vector3 driveForce { get; private set; }


        void Reset()
        {
            Debug.Log("TIME:" + Time.time + ", Label:" + label + " starting MecanumWHeel.Reset()..");
            // Initializes the value of WheelCollider.
            // Set WheelCollider's Friction to zero in order to apply our own tire force.
            // TODO: Implement the Editor extension to populate and initialize the Inspector.
            //wheelCollider = GetComponent<WheelCollider>();
            wheelCollider.radius = 0.075f;
            wheelCollider.suspensionDistance = 0.2f;
            Debug.Log("TIME:" + Time.time + ", Label:" + label + " exiting MecanumWHeel.Reset()..");

        }

        void Awake()
        {
            Debug.Log("TIME:" + Time.time + ", Label:" + label + " starting MecanumWHeel.Awake()..");
            wheelCollider.ConfigureVehicleSubsteps(1000.0f, 1, 1);
            vehicleRigidbody = wheelCollider.attachedRigidbody;
            Debug.Log("TIME:" + Time.time + ", Label:" + label + " vehicleRigidbody:" + vehicleRigidbody);
            wheelCollider.motorTorque = 0.00001f;
            acceleration = 0;
            driveForce = Vector3.zero;

            rollerAxisAngle = rollerAxisAngleDeg * Mathf.PI / 180.0f;
            rollerRotationAxisLocal = new Vector3(Mathf.Cos(rollerAxisAngle), 0, -Mathf.Sin(rollerAxisAngle));
            wheelRotationAxisLocal = new Vector3(1.0f, 0, 0);
            wheelForwardAxisLocal = new Vector3(0, 0, 1.0f);
            sinRollerAngle = Mathf.Sin(rollerAxisAngle);
            cosRollerAngle = Mathf.Cos(rollerAxisAngle);
            Debug.Log("TIME:" + Time.time + ", Label:" + label + " exiting MecanumWHeel.Awake()..");
        }

        // for wheel rotation visual fields.
        float wheelPitchAngle = 0;
        float lastSteerAngle = 0;

        int updateDispCnt = 0;
        void Update()
        {

            if (updateDispCnt == 0) {
                //Debug.Log("Label:" + label + " MecanumWheel.FixedUpdate() is called.");
            }

            qtWorldToVehicle = transform.rotation; // vehicleRigidbody.transform.rotation;
            qtVehicleToWorld = Quaternion.Inverse(qtWorldToVehicle);
            qtRotAroundZAxisLocal = new Quaternion(0.0f, Mathf.Sin(rollerAxisAngle/2.0f), 0.0f, Mathf.Cos(rollerAxisAngle/2.0f));

            var vehicleVelocity = vehicleRigidbody.velocity;
            var localSpeed = vehicleRigidbody.transform.InverseTransformDirection(vehicleVelocity);

            if (updateDispCnt == 0) {
                //Debug.Log("dcMotor.Update() is about to be called.");
            }
            //dcMotor.Update();

            //UpdateSpeed();
            //UpdateWheelForce();
            //UpdateVisual(localSpeed.z, SteerAngle);
            UpdateVisual(localSpeed.z, 0);

            // TODO: Determine rotation from the speed of the wheels.
            void UpdateVisual(float speed, float steerAngle)
            {
                wheelCollider.GetWorldPose(out var pos, out _);
                wheelVisualTransform.position = pos;

                // wheel forward rotation(pitch).
                var additionalPitchAngle = (speed * Time.deltaTime / wheelCollider.radius) * Mathf.Rad2Deg;
                wheelPitchAngle += additionalPitchAngle;
                wheelPitchAngle %= 360;

                // Apply rotations to visual wheel object.
                wheelVisualTransform.localEulerAngles = new Vector3(wheelPitchAngle, steerAngle, 0);

                // Cache steer angle value for next update.
                lastSteerAngle = steerAngle;
            }
        }

        /// <summary>
        /// Update wheel skidding cancel force rate. called by Vehicle's FixedUpdate()
        /// </summary>
        public void UpdateSkiddingCancelRate(float newValue)
        {
            if (skiddingCancelRate != newValue)
                skiddingCancelRate = newValue;
        }

        /// <summary>
        /// Update wheel hit. called by Vehicle's FixedUpdate()
        /// </summary>
        public void UpdateWheelHit()
        {
            IsGrounded = wheelCollider.GetGroundHit(out wheelHit);
        }

        /// <summary>
        /// Update wheel sleep. called by Vehicle's FixedUpdate()
        /// </summary>
        /// <param name="isSleep">wheel sleep?</param>
        public void UpdateWheelSleep(bool isSleep)
        {
            // Strictly, it is different because you cannot put a Rigidbody to sleep.
            // However, can use the WheelCollider to make it as close to sleep as possible.
            // Related issues : https://forum.unity.com/threads/how-is-this-not-a-bug-wheel-collider-bugs.515156/
            // TODO: Improved WheelCollider Sleep.
            if (IsSleep != isSleep)
                IsSleep = isSleep;
        }

        /// <summary>
        /// Update wheel's rotational speed
        /// </summary>
        public void UpdateSpeed()
        {
            // Calculate wheel's position relative to vehicle's center expressed in world coordinate
            Vector3 positionRelativeToVehicle =
                vehicleRigidbody.transform.InverseTransformPoint(transform.position);
            Vector3 armFromCenter = vehicleRigidbody.transform.TransformVector(positionRelativeToVehicle);

            // calculate wheel's linear velocity in world coordinate
            Vector3 wheelSpeed = 
                vehicleRigidbody.velocity + 
                Vector3.Cross(vehicleRigidbody.angularVelocity, armFromCenter);

            // calculate wheel's linear velocity in wheel's coordinate
            Vector3 localWheelSpeed = transform.InverseTransformVector(wheelSpeed); 

            // calculate angular velocity of motor
            if (IsGrounded) {
                var wheelAxis = transform.TransformDirection(new Vector3(1.0f, 0, 0));
                var rollerAxis = Mathf.Cos(rollerAxisAngle) * wheelAxis - Mathf.Sin(rollerAxisAngle) * wheelHit.forwardDir;

                dcMotor.SetSpeed(-Vector3.Dot(wheelSpeed, rollerAxis) / sinRollerAngle / radius);

                //dcMotor.SetSpeed(-Vector3.Dot(localWheelSpeed, rollerRotationAxisLocal) / sinRollerAngle / radius);
            } else {
                dcMotor.SetSpeed(0.0f);
            }

            if (updateDispCnt == 0) {
                Debug.Log("TIM:" + Time.time + ", Label:" + label + ", dcMotor.Speed:" + dcMotor.Speed +
                ", wheelSpeed:" + wheelSpeed + ", localWheelSpeed:" + localWheelSpeed +
                ", vehicleRigidbody(.velocity:" + vehicleRigidbody.velocity +
                ", .angularVelocity:" + vehicleRigidbody.angularVelocity +
                ") , positionRelativeToVehicle:" + positionRelativeToVehicle);
            }
            dcMotor.Update();
        }

        /// <summary>
        /// Apply voltage to motor.
        /// </summary>
        public void SetMotorDuty(float duty)
        {
            dcMotor.SetDuty(duty);
            dcMotor.Update();
        }


        /// <summary>
        /// Apply voltage to motor.
        /// </summary>
        public void SetWheelAcceleration(float _acceleration)
        {
            acceleration = _acceleration;
        }
        
        /// <summary>
        /// Apply the force that the tire outputs to the forward and sideway.
        /// </summary>
        public void UpdateWheelForce(/*float acceleration*/)
        {
            if (updateDispCnt == 0) {
                //Debug.Log("Label:" + label + " Is wheel grounded?" + IsGrounded);
            }
            if (IsGrounded == false) {
                //vehicleRigidbody.velocity = new Vector3(0.0f,0.0f,0.0f);
                if (acceleration != 0) {
                    Debug.Log("TIM:" + Time.time + ", Label:" + label + ", duty:" + acceleration + " (NOT GROUNDED)");
                }
                if (5 <= ++updateDispCnt) {
                    updateDispCnt = 0;
                }
                return;
            }

            //var rollerAxis = transform.TransformVector(rollerRotationAxisLocal);    // from local to world
            var wheelAxis = transform.TransformDirection(new Vector3(1.0f, 0, 0));
            var rollerAxis = Mathf.Cos(rollerAxisAngle) * wheelAxis - Mathf.Sin(rollerAxisAngle) * wheelHit.forwardDir;

            // Apply drive force.
            // Apply a force that will result in the commanded acceleration.
            if (!MecanumVehicle.ControlParam.useCmdVelFeedback) {       // Simulate motor dynamics
                // Use backward discretization; motor model occilates when using forward discretization!!! 20240320 
                //float inertia = 40.0f * radius*radius;
                //float backwardDiscretizationCoeff = 1.0f +
                //   dcMotor.BackEmfConstant*dcMotor.TorqueConstant/dcMotor.ArmatureResistance/inertia * Time.fixedDeltaTime;
                float backwardDiscretizationCoeff = 1.0f;

                // Calculate traction force in world coordinate
                driveForce = (- dcMotor.Torque/backwardDiscretizationCoeff / radius * rollerAxis / sinRollerAngle );    // 0.6: relaxation parameter

                // Apply force
                vehicleRigidbody.AddForceAtPosition(driveForce, wheelHit.point, ForceMode.Acceleration);
                if (false /*updateDispCnt == 0*/) {
                    var localDriveForce = transform.InverseTransformVector(driveForce); 
                    Debug.Log("fixedDeltaTime:" + Time.fixedDeltaTime + ", backwardDiscretizationCoeff-1:" + (backwardDiscretizationCoeff-1.0f));
                    Debug.Log("TIM:" + Time.time + ", Label:" + label + ", duty:" + dcMotor.Voltage/dcMotor.MaximumVoltage
                                 + ", torque:" + dcMotor.Torque + ", driveForce:" + driveForce + ", localDriveForce:" + localDriveForce + ", speed:" + dcMotor.Speed);
                }
            } else {
                // Use PI controller for CmdVel
                driveForce = acceleration * rollerAxis; // ((qtWorldToVehicle * qtRotAroundZAxisLocal * qtVehicleToWorld) * wheelHit.forwardDir);
                vehicleRigidbody.AddForceAtPosition(driveForce, wheelHit.point, ForceMode.Acceleration);
                if (updateDispCnt == 0) {
                    var localDriveForce = transform.InverseTransformVector(driveForce); 
                    Debug.Log("TIM:" + Time.time + ", Label:" + label + ", acceleration:" + acceleration
                                 + ", wheelHit.forwardDir:" + wheelHit.forwardDir + ", driveForce:" + driveForce
                                 + ", localDriveForce:" + localDriveForce + ", speed:" + dcMotor.Speed);
                }
            }
            //Debug.Log("Label:" + label + ",rollerAxis:" + rollerAxis + ",forwardDir:" + /*wheelVisualTransform.InverseTransformVector(*/wheelHit.forwardDir);
             

            if (5 <= ++updateDispCnt) {
                updateDispCnt = 0;
            }

/*
            // Counteracts the sideway force of the tire.
            // TODO: more accurate calculation method.

            Vector3 GetSkiddingCancelForce()
            {
                var pointVelocity = wheelCollider.attachedRigidbody.GetPointVelocity(wheelHit.point);
                var wheelVelocity = pointVelocity - Vector3.Project(pointVelocity, wheelHit.normal);
                var localWheelVelocity = Vector3.zero;
                localWheelVelocity.y = Vector3.Dot(wheelHit.forwardDir, wheelVelocity);
                localWheelVelocity.x = Vector3.Dot(wheelHit.sidewaysDir, wheelVelocity);

                Vector2 cancelForce = -1 * skiddingCancelRate * wheelCollider.sprungMass * localWheelVelocity / Time.fixedDeltaTime;
                Vector3 skiddingCancelForce = wheelHit.sidewaysDir * cancelForce.x;

                return skiddingCancelForce;
            }
*/
        }


    }

}
