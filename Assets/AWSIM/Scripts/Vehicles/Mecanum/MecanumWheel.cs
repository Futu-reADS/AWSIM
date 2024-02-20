using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace AWSIM
{
    /// <summary>
    /// Vehicle's wheel class.
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

/*
        /// <summary>
        /// Steering angle of the wheel.
        /// </summary>
        public float SteerAngle => wheelCollider.steerAngle;
*/

        // latest wheel hit. This value updated according to the FixedUpdate() of the vehicle.
        WheelHit wheelHit;

        // Cached rigidbody of the vehicle to which the wheelcollider is attached.
        Rigidbody vehicleRigidbody;

        // Coefficient for cancelling the skidding while stopping of the tire.
        float skiddingCancelRate;

        // angle (in rad) of area where mecanum wheel touches the ground 
        // FL, RR: -0.785  FR, RL: +0.785
        public float angleOfForceApplication;

        void Reset()
        {
            // Initializes the value of WheelCollider.
            // Set WheelCollider's Friction to zero in order to apply our own tire force.
            // TODO: Implement the Editor extension to populate and initialize the Inspector.
            wheelCollider = GetComponent<WheelCollider>();
            wheelCollider.radius = 0.05f;
            wheelCollider.suspensionDistance = 0.2f;

            angleOfForceApplication = 0;
        }

        void Awake()
        {
            wheelCollider.ConfigureVehicleSubsteps(1000.0f, 1, 1);
            vehicleRigidbody = wheelCollider.attachedRigidbody;
            wheelCollider.motorTorque = 0.00001f;
        }

        // for wheel rotation visual fields.
        float wheelPitchAngle = 0;
        float lastSteerAngle = 0;

        void Update()
        {
            var vehicleVelocity = vehicleRigidbody.velocity;
            var localSpeed = vehicleRigidbody.transform.InverseTransformDirection(vehicleVelocity);

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

/*
        /// <summary>
        /// Update wheel steer angle. called by Vehicle's FixedUpdate()
        /// </summary>
        public void UpdateWheelSteerAngle(float steerAngle)
        {
            // Set a non-zero value to stable the sleep behavior.
            if (steerAngle == 0)
                steerAngle = 0.00001f;

            if (wheelCollider.steerAngle != steerAngle)
                wheelCollider.steerAngle = steerAngle;
        }
*/

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
        /// Apply the force that the tire outputs to the forward and sideway.
        /// </summary>
        /// <param name="acceleration"></param>
        public void UpdateWheelForce(float acceleration)
        {
            //Debug.Log("Is wheel grounded?" + IsGrounded);
            if (IsGrounded == false) {
                //vehicleRigidbody.velocity = new Vector3(0.0f,0.0f,0.0f);
                return;
            }
            //Debug.Log("Wheel is grounded.");
            //acceleration = 0.0f;
            // Apply cancel force.
            //var lateralCancelForce = GetSkiddingCancelForce();
            //vehicleRigidbody.AddForceAtPosition(lateralCancelForce, wheelHit.point, ForceMode.Force);

            // Apply drive force.
            // Apply a force that will result in the commanded acceleration.
            Quaternion qtRotAroundZAxisLocal 
            = new Quaternion(0.0f, Mathf.Sin(angleOfForceApplication/2.0f), 0.0f, Mathf.Cos(angleOfForceApplication/2.0f));
            Quaternion qtRotAroundZAxisGlobal = vehicleRigidbody.transform.rotation * qtRotAroundZAxisLocal;

            var driveForce = acceleration * (qtRotAroundZAxisGlobal * wheelHit.forwardDir);
            vehicleRigidbody.AddForceAtPosition(driveForce, wheelHit.point, ForceMode.Acceleration);
            Debug.Log("Label:" + label + ", wheelHit.forwardDir:" + wheelHit.forwardDir + ", qtRot * wheelHit.forwardDir:" + qtRotAroundZAxisGlobal * wheelHit.forwardDir
              + ", driveForce:" + driveForce);
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