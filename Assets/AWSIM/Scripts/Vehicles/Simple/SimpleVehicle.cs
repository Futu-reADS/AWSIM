using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace AWSIM {
    [System.Serializable]
    public class SimpleVehicle : MonoBehaviour
    {
        [SerializeField] Rigidbody rigidbody;

        [SerializeField] Vector3 localForceToApply;

        [SerializeField] float heightOfForceApplicationPoint;

        [SerializeField] double timeToApplyForce;

        int count = -1;
        int countFixedUpdate = 0;

        // Start is called before the first frame update
        void Start()
        {
            rigidbody = GetComponent<Rigidbody>();
        }
        
        void Awake() {
            rigidbody = GetComponent<Rigidbody>();
        }

        // Update is called once per frame
        void FixedUpdate()
        {
            rigidbody.WakeUp();
            if ((countFixedUpdate % 100) == 0) {
                Debug.Log(string.Format("{0} FixedUpdate() is called", countFixedUpdate));
            }
            countFixedUpdate++;
            if (timeToApplyForce <= 0) {
                return;
            }

            rigidbody.AddForceAtPosition(
                transform.TransformVector(localForceToApply),
                transform.TransformPoint(Vector3.zero) + new Vector3(0, -0.5f + heightOfForceApplicationPoint, 0));
            timeToApplyForce -= Time.fixedDeltaTime;

            if (timeToApplyForce < 0) {
                timeToApplyForce = 0;
            }
        }

        void OnCollisionEnter(Collision collision) {
            Debug.Log(string.Format("[N]    collision.impulse:({0},{1},{2})", collision.impulse.x, collision.impulse.y, collision.impulse.z));
            count = 0;
        }
        void OnCollisionStay(Collision collision) {
            Debug.Log(string.Format("[S]{0}    collision.impulse:({1},{2},{3})", count, collision.impulse.x, collision.impulse.y, collision.impulse.z));
            count++;
        }
        void OnCollisionExit(Collision collision) {
            Debug.Log(string.Format("[X]{0}    collision.impulse:({1},{2},{3})", count, collision.impulse.x, collision.impulse.y, collision.impulse.z));
            count = -1;
        }
    };
}
