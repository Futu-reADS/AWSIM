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


        void Reset()
        {
            // Initializes the value of WheelCollider.
            // Set WheelCollider's Friction to zero in order to apply our own tire force.
            // TODO: Implement the Editor extension to populate and initialize the Inspector.
            wheelCollider = GetComponent<WheelCollider>();
            wheelCollider.radius = 0.075f;
            wheelCollider.suspensionDistance = 0.2f;

        }

        void Awake()
        {
            wheelCollider.ConfigureVehicleSubsteps(1000.0f, 1, 1);
            vehicleRigidbody = wheelCollider.attachedRigidbody;
            Debug.Log("label:" + label + " vehicleRigidbody:" + vehicleRigidbody);
            wheelCollider.motorTorque = 0.00001f;

            rollerAxisAngle = rollerAxisAngleDeg * Mathf.PI / 180.0f;
            rollerRotationAxisLocal = new Vector3(Mathf.Cos(rollerAxisAngle), 0, -Mathf.Sin(rollerAxisAngle));
            wheelRotationAxisLocal = new Vector3(1.0f, 0, 0);
            wheelForwardAxisLocal = new Vector3(0, 0, 1.0f);
            sinRollerAngle = Mathf.Sin(rollerAxisAngle);
            cosRollerAngle = Mathf.Cos(rollerAxisAngle);
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

            qtWorldToVehicle = vehicleRigidbody.transform.rotation;
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
                dcMotor.SetSpeed(-Vector3.Dot(localWheelSpeed, rollerRotationAxisLocal) / sinRollerAngle / radius);
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
        /// Apply the force that the tire outputs to the forward and sideway.
        /// </summary>
        public void UpdateWheelForce(float acceleration)
        {
            if (updateDispCnt == 0) {
                //Debug.Log("Label:" + label + " Is wheel grounded?" + IsGrounded);
            }
            if (IsGrounded == false) {
                //vehicleRigidbody.velocity = new Vector3(0.0f,0.0f,0.0f);
                if (acceleration != 0) {
                    Debug.Log("TIM:" + Time.time + ", Label:" + label + ", duty:" + acceleration + " (NOT GROUNDED)");
                }
                return;
            }
            if (updateDispCnt == 0) {
                //Debug.Log("Wheel is grounded.");
            }

            qtWorldToVehicle = vehicleRigidbody.transform.rotation;
            qtVehicleToWorld = Quaternion.Inverse(qtWorldToVehicle);
            qtRotAroundZAxisLocal = new Quaternion(0.0f, Mathf.Sin(-rollerAxisAngle/2.0f), 0.0f, Mathf.Cos(-rollerAxisAngle/2.0f));

            // Apply drive force.
            // Apply a force that will result in the commanded acceleration.
#if false
            // Motor model occilates!!! 20240320 
            // Update motor's state
            dcMotor.SetDuty(acceleration);
            dcMotor.Update();
            // Calculate roller's axis in world coordinate
            //var rollerAxis = vehicleRigidbody.transform.TransformVector(rollerRotationAxisLocal);
            var rollerAxis = transform.TransformVector(rollerRotationAxisLocal);    // from local to world
            //var rollerAxis = transform.InverseTransformVector(wheelHit.forwardDir);
            //var rollerAxis = qtWorldToVehicle * qtRotAroundZAxisLocal * qtVehicleToWorld*  wheelHit.forwardDir;
            // Calculate traction force in world coordinate
            var driveForce = (- dcMotor.Torque / radius * rollerAxis / sinRollerAngle );    // 0.6: relaxation parameter
            // Apply force
            vehicleRigidbody.AddForceAtPosition(driveForce, wheelHit.point, ForceMode.Acceleration);
            if (updateDispCnt == 0) {
                Debug.Log("TIM:" + Time.time + ", Label:" + label + ", duty:" + acceleration + ", rollerAxis:" + rollerAxis + ", torque:" + dcMotor.Torque + ", driveForce:" + driveForce + ", speed:" + dcMotor.Speed + ", radius:" + radius);
            }
#else
            var rollerAxis = transform.TransformVector(rollerRotationAxisLocal);    // from local to world
            var driveForce = acceleration * rollerAxis; // ((qtWorldToVehicle * qtRotAroundZAxisLocal * qtVehicleToWorld) * wheelHit.forwardDir);
            vehicleRigidbody.AddForceAtPosition(driveForce, wheelHit.point, ForceMode.Acceleration);
            if (acceleration != 0) {
                Debug.Log("TIM:" + Time.time + ", Label:" + label + ", acceleration:" + acceleration + ", wheelHit.forwardDir:" + wheelHit.forwardDir +
                 ", driveForce:" + driveForce);
            }
#endif

            updateDispCnt = updateDispCnt + 1;
            if (50 <= updateDispCnt) {
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
