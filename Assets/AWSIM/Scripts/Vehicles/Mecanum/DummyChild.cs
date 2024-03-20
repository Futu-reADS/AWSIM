using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class DummyChild : MonoBehaviour
{
    public Quaternion rotation;
    public Quaternion localRotation;
    public Vector3 localPosition;
    public Vector3 e1_localRotation;
    public Vector3 e2_localRotation;
    public Vector3 e3_localRotation;
    public Vector3 e1_localInvRotation;
    public Vector3 e2_localInvRotation;
    public Vector3 e3_localInvRotation;
    public Vector3 e1_rotation;
    public Vector3 e2_rotation;
    public Vector3 e3_rotation;
    public Vector3 e1_invRotation;
    public Vector3 e2_invRotation;
    public Vector3 e3_invRotation;
    public Vector3 e1_TransformVector;
    public Vector3 e2_TransformVector;
    public Vector3 e3_TransformVector;
    public Vector3 e1_InverseTransformVector;
    public Vector3 e2_InverseTransformVector;
    public Vector3 e3_InverseTransformVector;
    
    
    // Start is called before the first frame update
    void Start()
    {
        
    }

    // Update is called once per frame
    void Update()
    {
        Vector3 e1, e2, e3;

        rotation = transform.rotation;
        localRotation = transform.localRotation;

        localPosition = transform.localPosition;

        e1 = new Vector3(1.0f, 0,0);
        e2 = new Vector3(0, 1.0f, 0);
        e3 = new Vector3(0, 0, 1.0f);

        // Expression in parent's coordinate
        e1_localRotation = transform.localRotation * e1;
        e2_localRotation = transform.localRotation * e2;
        e3_localRotation = transform.localRotation * e3;

        // Expression in child cooronate
        e1_localInvRotation = Quaternion.Inverse(transform.localRotation) * e1;
        e2_localInvRotation = Quaternion.Inverse(transform.localRotation) * e2;
        e3_localInvRotation = Quaternion.Inverse(transform.localRotation) * e3;

        e1_rotation = transform.rotation * e1;
        e2_rotation = transform.rotation * e2;
        e3_rotation = transform.rotation * e3;

        e1_invRotation = Quaternion.Inverse(transform.rotation) * e1;
        e2_invRotation = Quaternion.Inverse(transform.rotation) * e2;
        e3_invRotation = Quaternion.Inverse(transform.rotation) * e3;

        e1_TransformVector = transform.TransformVector(e1);
        e2_TransformVector = transform.TransformVector(e2);
        e3_TransformVector = transform.TransformVector(e3);

        e1_InverseTransformVector = transform.InverseTransformVector(e1);
        e2_InverseTransformVector = transform.InverseTransformVector(e2);
        e3_InverseTransformVector = transform.InverseTransformVector(e3);

    }
}
