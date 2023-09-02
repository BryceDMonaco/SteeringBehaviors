using System.Collections;
using System.Collections.Generic;
using UnityEngine;

// Implements math described in https://gamedevelopment.tutsplus.com/series/understanding-steering-behaviors--gamedev-12732
public class Steering : MonoBehaviour
{
    public enum SteeringType
    {
        SeekWithoutSteering,
        SeekWithSteering,
        FleeWithSteering
    }
    [SerializeField] private SteeringType seekType = SteeringType.SeekWithSteering;
    [SerializeField] private Transform target;
    [SerializeField] private Rigidbody myRigidbody;
    [SerializeField] private float slowDistance = 8f;  // When are this close or closer, start to slow down, must be >= stopDistance
    [SerializeField] private float stopDistance = 5f;  // When we are this close or closer, stop
    [SerializeField] private float maxVelocity = 5f;
    [SerializeField] private bool drawDebugLines = true;
    [SerializeField] private float debugLineLength = 3f;
    
    void Start()
    {
        if (myRigidbody == null)
        {
            myRigidbody = GetComponent<Rigidbody>();
        }

        if (slowDistance < stopDistance)
        {
            Debug.LogError("slowDistance must be greater than or equal to stop distance");
        }
    }

    void FixedUpdate()
    {
        switch (seekType)
        {
            case SteeringType.SeekWithoutSteering:
                SeekWithoutSteering();
                break;
            case SteeringType.SeekWithSteering:
                SeekWithSteering();
                break;
            case SteeringType.FleeWithSteering:
                FleeWithSteering();
                break;
            default:
                Debug.LogError("Unhandled steering type");
                break;
        }
    }

    /**
     * This is the simplest form of seeking behavior. The object will always 
     * move towards its target, direction change is immediate and not gradual,
     * meaning there is no steering. Note that the object does not rotate to
     * face its target, it just changes its velocity to move towards it.
     */
    void SeekWithoutSteering()
    {
        Vector3 myPos = transform.position;
        Vector3 targetPos = target.position;
        Vector3 velocity;

        if (Vector3.Distance(myPos, targetPos) <= stopDistance)
        {
            velocity = Vector3.zero;  // We are close enough, stop
        }
        else
        {
            velocity = (targetPos - myPos).normalized * maxVelocity;
        }

        if (drawDebugLines)
        {
            // Draw the direction vector/desired velocity
            Debug.DrawLine(myPos, myPos + (velocity.normalized * debugLineLength), Color.magenta);
        }

        myRigidbody.velocity = velocity;
    }

    /**
     * Seeking with steering behavior. When the target moves, this object will
     * gradually change its direction over time, in this implementation, a
     * higher mass for this object's rigidbody will mean it changes direction
     * slower. Note that the object does not rotate to face its target, it just
     * changes its velocity to move towards it.
     */
    void SeekWithSteering()
    {
        Vector3 myPos = transform.position;
        Vector3 targetPos = target.position;
        Vector3 velocity = myRigidbody.velocity;
        Vector3 steering;

        if (Vector3.Distance(myPos, targetPos) <= stopDistance)
        {
            velocity = Vector3.zero;  // We are close enough, stop
        }
        else
        {
            Vector3 desiredVelocity = (targetPos - myPos).normalized * maxVelocity;

            /*
             * This could be combined into one line, but breaking it up makes
             * the math easier to follow.
             */
            steering = desiredVelocity - velocity;
            steering = ClampVector(steering, maxVelocity);
            steering = steering / myRigidbody.mass;
            velocity = ClampVector(velocity + steering, maxVelocity);
        }

        if (drawDebugLines)
        {
            // Draw the desired direction vector
            Debug.DrawLine(myPos, myPos + (velocity.normalized * debugLineLength), Color.magenta);
            // Draw the current direction vector
            Debug.DrawLine(myPos, myPos + (myRigidbody.velocity.normalized * debugLineLength), Color.green);

        }

        myRigidbody.velocity = velocity;
    }

    /**
     * Seeking with steering behavior. When the target moves, this object will
     * gradually change its direction over time, in this implementation, a
     * higher mass for this object's rigidbody will mean it changes direction
     * slower. Note that the object does not rotate to face its target, it just
     * changes its velocity to move towards it.
     */
    void FleeWithSteering()
    {
        Vector3 myPos = transform.position;
        Vector3 targetPos = target.position;
        Vector3 velocity = myRigidbody.velocity;
        Vector3 steering;

        if (Vector3.Distance(myPos, targetPos) >= stopDistance)
        {
            velocity = Vector3.zero;  // We are far enough, stop
        }
        else
        {
            Vector3 desiredVelocity = (myPos - targetPos).normalized * maxVelocity;

            /*
             * This could be combined into one line, but breaking it up makes
             * the math easier to follow.
             */
            steering = desiredVelocity - velocity;
            steering = ClampVector(steering, maxVelocity);
            steering = steering / myRigidbody.mass;
            velocity = ClampVector(velocity + steering, maxVelocity);
        }

        if (drawDebugLines)
        {
            // Draw the desired direction vector
            Debug.DrawLine(myPos, myPos + (velocity.normalized * debugLineLength), Color.magenta);
            // Draw the current direction vector
            Debug.DrawLine(myPos, myPos + (myRigidbody.velocity.normalized * debugLineLength), Color.green);

        }

        myRigidbody.velocity = velocity;
    }

    Vector3 ClampVector(Vector3 vector, float maxValue)
    {
        return new Vector3(
            Mathf.Clamp(vector.x, -maxValue, maxValue),
            Mathf.Clamp(vector.y, -maxValue, maxValue),
            Mathf.Clamp(vector.z, -maxValue, maxValue));
    }
}
