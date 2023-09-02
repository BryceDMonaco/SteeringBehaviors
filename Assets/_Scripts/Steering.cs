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
        FleeWithSteering,
        WanderWithSteering,
        PursueWithSteering,
        EvadeWithSteering
    }
    [SerializeField] private SteeringType steerType = SteeringType.SeekWithSteering;
    [SerializeField] private Transform target;
    [SerializeField] private Rigidbody targetRigidbody;
    [SerializeField] private Rigidbody myRigidbody;
    [SerializeField] private float slowDistance = 8f;  // When are this close or closer, start to slow down, must be >= stopDistance
    [SerializeField] private float stopDistance = 5f;  // When we are this close or closer, stop
    [SerializeField] private float wanderCircleRadius = 3f;
    [SerializeField] private float maxVelocity = 5f;
    [SerializeField] private bool drawDebugLines = true;
    [SerializeField] private float debugLineLength = 3f;
    [SerializeField] private float wanderAngleChange = 5f;
    [SerializeField] private int pursuitPredictAheadIterationWaits = 10;
    private int currentPredictWaits = 0;

    private float wanderAngle = 0f;
    
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

        wanderAngle += GetNewWanderAngle();
    }

    void FixedUpdate()
    {
        Vector3 steering = Vector3.zero;

        switch (steerType)
        {
            case SteeringType.SeekWithSteering:
                steering = SeekWithSteering();
                break;
            case SteeringType.FleeWithSteering:
                steering = FleeWithSteering();
                break;
            case SteeringType.WanderWithSteering:
                steering = WanderWithSteering();
                break;
            case SteeringType.PursueWithSteering:
                /*
                 * If we are constantly predicting the target's position, we
                 * are just following it. Instead, only predict its position
                 * every pursuitPredictAheadIterationWaits iterations, which
                 * is every (pursuitPredictAheadIterationWaits * 
                 * Time.fixedDeltaTime) seconds
                 */
                if (currentPredictWaits >= pursuitPredictAheadIterationWaits)
                {
                    currentPredictWaits = 0;
                    steering = PursueWithSteering();
                }
                else
                {
                    currentPredictWaits++;
                }
                break;
            case SteeringType.EvadeWithSteering:
                steering = EvadeWithSteering();
                break;
            default:
                Debug.LogError("Unhandled steering type");
                break;
        }

        Vector3 velocity = ClampVector(myRigidbody.velocity + steering, maxVelocity);

        // Calculate arrival for these behaviors
        if (steerType == SteeringType.SeekWithSteering || steerType == SteeringType.PursueWithSteering)
        {
            
        }

        Vector3 myPos = transform.position;
        // If the target is null, we are wandering, don't need to calc anything
        Vector3 targetPos = target == null ? Vector3.zero : target.position;

        switch (steerType)
        {
            case SteeringType.SeekWithSteering:
            case SteeringType.PursueWithSteering:
                /*
                 * This will just fully stop the agent when it gets close. Fine
                 * for now, but will need to check to make sure no other
                 * steering behaviors are running when support for multiple
                 * at the same time is added, otherwise it will just ignore the
                 * others and not move.
                 */
                float distanceToTarget = Vector3.Distance(myPos, targetPos);
                if (distanceToTarget < slowDistance)
                {
                    velocity = velocity.normalized * maxVelocity * ((distanceToTarget - stopDistance) / (slowDistance - stopDistance));
                }

                break;
            case SteeringType.FleeWithSteering:
            case SteeringType.EvadeWithSteering:
                /*
                 * This will just fully stop the agent when it gets away. Fine
                 * for now, but will need to check to make sure no other
                 * steering behaviors are running when support for multiple
                 * at the same time is added, otherwise it will just ignore the
                 * others and not move.
                 */
                if (Vector3.Distance(myPos, targetPos) >= stopDistance)
                {
                    velocity = Vector3.zero;  // We are far enough, stop
                }

                break;
            default:
                break;
        }

        myRigidbody.velocity = velocity;
    }

    public void SetSteeringType (SteeringType type, Transform target)
    {
        steerType = type;
        this.target = target;
    }

    public SteeringType GetSteeringType ()
    {
        return steerType;
    }

    public float GetStopDistance ()
    {
        return stopDistance;
    }

    /*
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

            // Arrival
            float distanceToTarget = Vector3.Distance(myPos, targetPos);
            if (distanceToTarget < slowDistance)
            {
                velocity = velocity.normalized * maxVelocity * ((distanceToTarget - stopDistance) / (slowDistance - stopDistance));
            }
        }

        if (drawDebugLines)
        {
            // Draw the direction vector/desired velocity
            Debug.DrawLine(myPos, myPos + (velocity.normalized * debugLineLength), Color.magenta);
        }

        myRigidbody.velocity = velocity;
    }

    /*
     * Seeking with steering behavior. When the target moves, this object will
     * gradually change its direction over time, in this implementation, a
     * higher mass for this object's rigidbody will mean it changes direction
     * slower. Note that the object does not rotate to face its target, it just
     * changes its velocity to move towards it.
     */
    Vector3 SeekWithSteering()
    {
        Vector3 myPos = transform.position;
        Vector3 targetPos = target.position;
        Vector3 velocity = myRigidbody.velocity;
        Vector3 steering;

        if (Vector3.Distance(myPos, targetPos) <= stopDistance)
        {
            return Vector3.zero;  // We are close enough, do not steer
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

            return steering;
        }
    }

    /*
     * Fleeing with steering behavior. If the target comes within stopDistance
     * of this object, it will flee by gradually changing its direction over
     * time to go the opposite diretion of the target. In this implementation,
     * a higher mass for this object's rigidbody will mean it changes direction
     * slower. Note that the object does not rotate to face its target, it just
     * changes its velocity to move towards it.
     */
    Vector3 FleeWithSteering()
    {
        Vector3 myPos = transform.position;
        Vector3 targetPos = target.position;
        Vector3 velocity = myRigidbody.velocity;
        Vector3 steering;

        if (Vector3.Distance(myPos, targetPos) >= stopDistance)
        {
            return Vector3.zero;  // We are far enough, do not steer
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
            return steering;
        }
    }

    /*
     * Wanders randomly with steering behavior. This object has no target, 
     * instead it moves in a direction and is influenced over time to change
     * its direction. This gives the appearance of more realistic wandering
     * as opposed to sudden sharp direction changes.
     */
    Vector3 WanderWithSteering()
    {
        Vector3 myPos = transform.position;
        Vector3 velocity = myRigidbody.velocity;
        Vector3 steering;

        // Calculating the wander force
        wanderAngle += GetNewWanderAngle();
        Vector3 wanderCircleCenter = velocity.normalized * wanderCircleRadius;
        Vector3 wanderDisplacement = new Vector3(0, 0, -1) * wanderCircleRadius;
        wanderDisplacement = Quaternion.AngleAxis(wanderAngle, Vector3.up) * wanderDisplacement;
        Vector3 wanderForce = wanderCircleCenter + wanderDisplacement;


        /*
         * This could be combined into one line, but breaking it up makes
         * the math easier to follow.
         */
        steering = wanderForce;
        steering = ClampVector(steering, maxVelocity);
        steering = steering / myRigidbody.mass;
        return steering;
    }

    /*
     * Pursue a target with steering. Predicts the targets position based on
     * its current velocity and position and guesses farther ahead if it is
     * father away from the target. If this is run every FixedUpdate(), this
     * will just turn into a follow, be sure to run it less frequently if you
     * want to see it predict and move to that prediction for some time before
     * repredicting. Also uses steering to slowly influence its direction
     * change.
     */
    Vector3 PursueWithSteering()
    {
        Vector3 myPos = transform.position;
        Vector3 targetPos = target.position;
        Vector3 velocity = myRigidbody.velocity;
        Vector3 targetVelocity = targetRigidbody.velocity;
        Vector3 steering;
        Vector3 futureTargetPosition = targetPos;

        if (Vector3.Distance(myPos, targetPos) <= stopDistance)
        {
            return Vector3.zero;  // We are close enough, do not steer
        }
        else
        {
            float distanceBasedPredictAhead = Vector3.Distance(myPos, targetPos) / maxVelocity;
            futureTargetPosition = targetPos + (targetVelocity * distanceBasedPredictAhead);


            Vector3 desiredVelocity = (futureTargetPosition - myPos).normalized * maxVelocity;

            /*
             * This could be combined into one line, but breaking it up makes
             * the math easier to follow.
             */
            steering = desiredVelocity - velocity;
            steering = ClampVector(steering, maxVelocity);
            steering = steering / myRigidbody.mass;
            return steering;
        }
    }

    /*
     * Evade a target with steering. Predicts the targets position based on
     * its current velocity and position and guesses farther ahead if it is
     * father away from the target, then tries to flee that point. If this is
     * run every FixedUpdate(), this will just turn into a follow, be sure to
     * run it less frequently if you want to see it predict and move to that
     * prediction for some time before repredicting. Also uses steering to
     * slowly influence its direction change.
     */
    Vector3 EvadeWithSteering()
    {
        Vector3 myPos = transform.position;
        Vector3 targetPos = target.position;
        Vector3 velocity = myRigidbody.velocity;
        Vector3 targetVelocity = targetRigidbody.velocity;
        Vector3 steering;
        Vector3 futureTargetPosition = targetPos;

        if (Vector3.Distance(myPos, targetPos) >= stopDistance)
        {
            return Vector3.zero;  // We are far enough, do not steer
        }
        else
        {
            float distanceBasedPredictAhead = Vector3.Distance(myPos, targetPos) / maxVelocity;
            futureTargetPosition = targetPos + (targetVelocity * distanceBasedPredictAhead);


            Vector3 desiredVelocity = (myPos - futureTargetPosition).normalized * maxVelocity;

            /*
             * This could be combined into one line, but breaking it up makes
             * the math easier to follow.
             */
            steering = desiredVelocity - velocity;
            steering = ClampVector(steering, maxVelocity);
            steering = steering / myRigidbody.mass;
            return steering;
        }
    }

    /*
     * Clamp all values of a vector to +/- maxValue.
     */
    Vector3 ClampVector(Vector3 vector, float maxValue)
    {
        return new Vector3(
            Mathf.Clamp(vector.x, -maxValue, maxValue),
            Mathf.Clamp(vector.y, -maxValue, maxValue),
            Mathf.Clamp(vector.z, -maxValue, maxValue));
    }

    /*
     * Returns a randomly changed angle for wandering.
     */
    private float GetNewWanderAngle ()
    {
        return (Random.Range(0f, 1f) * wanderAngleChange) - (wanderAngleChange * 0.5f);
    }
}
