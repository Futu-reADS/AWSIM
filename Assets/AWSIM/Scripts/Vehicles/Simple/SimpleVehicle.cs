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
    };
}
