// Patrol.cs
using UnityEngine;
using UnityEngine.AI;
using System.Collections;

public class WaitForSecondsOrStop : CustomYieldInstruction
{
    private float waitTime;
    private bool stop;

    public WaitForSecondsOrStop(float time)
    {
        waitTime = time;
        stop = false;
    }

    public void Stop()
    {
        stop = true;
    }

    public override bool keepWaiting
    {
        get { return !stop && (waitTime -= Time.deltaTime) > 0; }
    }
}

public class Patrol : MonoBehaviour {

    public GameObject triggerZone;
    public GameObject patrol;
    public GameObject patrolNode;
    public GameObject avoidanceObstacle;
    private int destPoint = 0;
    private bool stopFlag = false;
    private Vector3 initialPosition;
    private Quaternion initialRotation;
    private Transform[] points;
    private NavMeshAgent agent;
    private Animator animator;
    private Trigger triggerScript;
    private WaitForSecondsOrStop waitInstruction;


    void Start () {
        triggerScript = triggerZone.GetComponent<Trigger>();
        agent = GetComponent<NavMeshAgent>();
        animator = GetComponent<Animator>();

        initialPosition = transform.position;
        initialRotation = transform.rotation;
/*
        // Check if the GameObject this script is attached to has a NavMeshObstacle component
        NavMeshObstacle obstacle = avoidanceObstacle.GetComponent<NavMeshObstacle>();

        if (obstacle == null)
        {
            obstacle = avoidanceObstacle.AddComponent<NavMeshObstacle>();
            // obstacle.shape = NavMeshObstacleShape.Box;
            obstacle.carving = true;
            obstacle.carveOnlyStationary = false;
        }
*/
        // Get all child transforms, including the parent transform
        Transform[] allPoints = patrolNode.GetComponentsInChildren<Transform>();

        // Filter out the parent transform
        points = new Transform[allPoints.Length - 1];
        for (int i = 1; i < allPoints.Length; i++) {
            points[i - 1] = allPoints[i];
        }

        // Disabling auto-braking allows for continuous movement
        // between points (ie, the agent doesn't slow down as it
        // approaches a destination point).
        agent.autoBraking = true;

        GotoNextPoint();
    }


    void GotoNextPoint() {
        // Returns if no points have been set up
        if (points.Length == 0)
            return;

        if (destPoint > 0)
        {
            if (points[destPoint-1].name.Contains("Stop"))
                stopFlag = true;
        }

        // Set the agent to go to the currently selected destination.
        agent.destination = points[destPoint].position;

        // Choose the next point in the array as the destination,
        // cycling to the start if necessary.
        destPoint = (destPoint + 1) % points.Length;
    }

    IEnumerator WaitAtStop() {
        // Stop the agent
        agent.isStopped = true;
        
        // Update the Animator's Speed parameter
        animator.SetFloat("moveSpeed", 0);
        animator.SetFloat("rotateSpeed", 0);

        // Wait for 5 seconds
        yield return waitInstruction;

        // Resume the agent
        agent.isStopped = false;
        stopFlag = false;
        // Move to the next point
        // GotoNextPoint();
    }

    void resetState() {
        if (triggerScript.resetState)
        {
            transform.position = initialPosition;
            transform.rotation = initialRotation;
            destPoint = 0;
            agent.isStopped = false;
            stopFlag = false;
            if (waitInstruction != null)
                waitInstruction.Stop();
            patrol.SetActive(false);
        }
    }

    void Update () {        
        // Choose the next destination point when the agent gets
        // close to the current one.

        if (stopFlag)
        {
            waitInstruction = new WaitForSecondsOrStop(5f);
            StartCoroutine(WaitAtStop());
        }
        else if (!agent.pathPending && agent.remainingDistance < 0.5f)
        {
            GotoNextPoint();
        }

        // Calculate the speed of the NavMeshAgent
        float speed = agent.velocity.magnitude;
        float angularSpeed = agent.angularSpeed;
        // Update the Animator's Speed parameter
        animator.SetFloat("moveSpeed", speed);
        // animator.SetFloat("rotateSpeed", angularSpeed);
        resetState();
    }
}