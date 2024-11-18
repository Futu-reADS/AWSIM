using ROS2;
using UnityEngine;
using UnityEngine.UI;

namespace AWSIM
{
    public class EstopButton : MonoBehaviour
    {
        [SerializeField] string topic = "ifb_driver/estop"; // Updated topic name
        [SerializeField] QoSSettings qosSettings;
        [SerializeField, Range(1, 100)] int publishHz = 10; // Default to 10 Hz
        [SerializeField] Toggle toggle; // Add a reference to the toggle
        [SerializeField] GameObject vehicle2Stop; // Add a reference to the toggle
        IPublisher<std_msgs.msg.Bool> boolPublisher; // Change to publish boolean
        std_msgs.msg.Bool boolMsg;
        bool toggleState = false; // Boolean to toggle
        float publishInterval;
        float nextPublishTime;
        private Rigidbody rb;
        private float defaultDrag;
        private float defaultAngularDrag;
        private float estopDrag = 5f;

        #region [Life Cycle]
        void Awake()
        {
            var qos = qosSettings.GetQoSProfile();
            boolPublisher = SimulatorROS2Node.CreatePublisher<std_msgs.msg.Bool>(topic, qos);
            boolMsg = new std_msgs.msg.Bool();
            toggle.onValueChanged.AddListener(SetToggleState); // Add listener for toggle value change
            publishInterval = 1.0f / publishHz; // Calculate interval based on publishHz
        }

        void Start()
        {
            nextPublishTime = Time.time;
            if (vehicle2Stop != null)
            {
                rb = vehicle2Stop.GetComponent<Rigidbody>();
            }
            if (rb != null)
            {
                defaultDrag = rb.drag; // Set the drag value
                defaultAngularDrag = rb.angularDrag; // Set the drag value
            }
        }

        void OnDestroy()
        {
            SimulatorROS2Node.RemovePublisher<std_msgs.msg.Bool>(boolPublisher);
        }
        #endregion

        #region [Main Thread]
        void Update()
        {
            if (Time.time >= nextPublishTime)
            {
                PublishBoolean();
                nextPublishTime += publishInterval;
            }
            if (rb != null)
            {
                if (toggleState)
                {
                    if (rb.velocity.magnitude < 0.5f)
                    {
                        rb.isKinematic = true;
                    }
                    rb.drag = estopDrag; // Set the drag value 
                    rb.angularDrag = estopDrag; // Set the drag value 
                } else {
                    rb.drag = defaultDrag; // Set the drag value
                    rb.angularDrag = defaultAngularDrag; // Set the drag value
                    rb.isKinematic = false;
                }
            }
        }
        #endregion

        void PublishBoolean()
        {
            boolMsg.Data = toggleState;
            boolPublisher.Publish(boolMsg);
        }

        public void SetToggleState(bool state)
        {
            toggleState = state; // Set the toggle state
        }
    }
}
