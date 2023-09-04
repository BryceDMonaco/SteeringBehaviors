using System.Collections;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;

// Implements math described in https://gamedevelopment.tutsplus.com/series/understanding-steering-behaviors--gamedev-12732
public class Steering : MonoBehaviour
{
    [System.Serializable]
    public class CommandPair
    {
        public Steering.SteeringBehavior command;
        public Transform target;

        public CommandPair(Steering.SteeringBehavior command, Transform target)
        {
            this.command = command;
            this.target = target;
        }
    }

    public enum SteeringBehavior
    {
        Seek,
        Flee,
        Wander,
        Pursue,
        Evade,
        CollisionAvoid
    }
    [SerializeField] private float slowDistance = 8f;  // When are this close or closer, start to slow down, must be >= stopDistance
    [SerializeField] private float stopDistance = 5f;  // When we are this close or closer, stop
    [SerializeField] private float wanderCircleRadius = 3f;
    [SerializeField] private float maxVelocity = 5f;
    [SerializeField] private float maxAvoidanceVelocity = 5f;
    [SerializeField] private bool drawDebugLines = true;
    [SerializeField] private float debugLineLength = 3f;
    [SerializeField] private float wanderAngleChange = 5f;
    [SerializeField] private float maxCollisionAvoidanceSeeAheadDistance = 10f;
    [SerializeField] private float maxCollisionAvoidanceRadiusClose =1.5f;
    [SerializeField] private float maxCollisionAvoidanceRadiusFar = 5f;

    [SerializeField] private float collisionFacingFovDeg = 45f;

    private Rigidbody myRigidbody;
    private Rigidbody targetRigidbody;

    private float wanderAngle = 0f;
    
    void Start()
    {
        myRigidbody = GetComponent<Rigidbody>();

        if (slowDistance < stopDistance)
        {
            Debug.LogError("slowDistance must be greater than or equal to stop distance");
        }

        wanderAngle += GetNewWanderAngle();
    }

    void FixedUpdate()
    {
        // Don't do anything here, a manager should handle it
    }

    public float GetMaxVelocity ()
    {
        return maxVelocity;
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
    Vector3 SeekWithoutSteering(Transform target)
    {
        Vector3 myPos = transform.position;
        Vector3 targetPos = target.position;
        Vector3 velocity;

        if (Vector3.Distance(myPos, targetPos) <= stopDistance)
        {
            return Vector3.zero;  // We are close enough, stop
        }
        else
        {
            velocity = (targetPos - myPos).normalized * maxVelocity;

            /*
            // Arrival
            float distanceToTarget = Vector3.Distance(myPos, targetPos);
            if (distanceToTarget < slowDistance)
            {
                velocity = velocity.normalized * maxVelocity * ((distanceToTarget - stopDistance) / (slowDistance - stopDistance));
            }
            */
        }

        if (drawDebugLines)
        {
            // Draw the direction vector/desired velocity
            Debug.DrawLine(myPos, myPos + (velocity.normalized * debugLineLength), Color.magenta);
        }

        return velocity;
    }

    /*
     * Seeking with steering behavior. When the target moves, this object will
     * gradually change its direction over time, in this implementation, a
     * higher mass for this object's rigidbody will mean it changes direction
     * slower. Note that the object does not rotate to face its target, it just
     * changes its velocity to move towards it.
     */
    public Vector3 Seek(Transform target)
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

            steering = desiredVelocity - velocity;

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
    public Vector3 Flee(Transform target)
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

            steering = desiredVelocity - velocity;
            
            return steering;
        }
    }

    /*
     * Wanders randomly with steering behavior. This object has no target, 
     * instead it moves in a direction and is influenced over time to change
     * its direction. This gives the appearance of more realistic wandering
     * as opposed to sudden sharp direction changes.
     */
    public Vector3 Wander()
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
    public Vector3 Pursue(Transform target)
    {
        Vector3 myPos = transform.position;
        Vector3 targetPos = target.position;
        Vector3 velocity = myRigidbody.velocity;
        Vector3 targetVelocity = target.GetComponent<Rigidbody>().velocity;
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

            steering = desiredVelocity - velocity;

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
    public Vector3 Evade(Transform target)
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

            steering = desiredVelocity - velocity;
            return steering;
        }
    }

    public Vector3 CollisionAvoidance()
    {
        Vector3 myPos = transform.position;
        Vector3 velocity = myRigidbody.velocity;

        Vector3 ahead = myPos + (velocity.normalized * maxCollisionAvoidanceSeeAheadDistance);

        List<Collider> hitsFromSphereClose = GetObstaclesWithinRadius(myPos, maxCollisionAvoidanceRadiusClose);
        List<Collider> hitsFromSphereFar;
        Transform obstacle = null;

        if (hitsFromSphereClose.Count > 0)
        {
            // Deal with closest obstacle first
            obstacle = hitsFromSphereClose[0].transform;
        } else if ((hitsFromSphereFar = GetObstaclesWithinRadius(myPos, maxCollisionAvoidanceRadiusFar)).Count > 0)
        {
            // If nothing is close, deal with the next obstacle in the far radius
            obstacle = hitsFromSphereFar[0].transform;
        } else
        {
            if (drawDebugLines)
            {
                // Draw the proxy circle
                Vector3 xAxisStart = new Vector3(myPos.x - maxCollisionAvoidanceRadiusFar, myPos.y, myPos.z);
                Vector3 xAxisEnd = new Vector3(myPos.x + maxCollisionAvoidanceRadiusFar, myPos.y, myPos.z);
                Vector3 zAxisStart = new Vector3(myPos.x, myPos.y, myPos.z - maxCollisionAvoidanceRadiusFar);
                Vector3 zAxisEnd = new Vector3(myPos.x, myPos.y, myPos.z + maxCollisionAvoidanceRadiusFar);
                Debug.DrawLine(xAxisStart, xAxisEnd, Color.green);
                Debug.DrawLine(zAxisStart, zAxisEnd, Color.green);

                // Draw line showing direction of my velocity
                Debug.DrawLine(myPos, myPos + velocity.normalized * 3f, Color.blue);
            }

            // No obstacles within either radius
            return Vector3.zero;
        }        

        if (drawDebugLines)
        {
            float sphereRadius = hitsFromSphereClose.Count > 0 ? maxCollisionAvoidanceRadiusClose : maxCollisionAvoidanceRadiusFar;

            // Draw the proxy circle
            Vector3 xAxisStart = new Vector3(myPos.x - sphereRadius, myPos.y, myPos.z);
            Vector3 xAxisEnd = new Vector3(myPos.x + sphereRadius, myPos.y, myPos.z);
            Vector3 zAxisStart = new Vector3(myPos.x, myPos.y, myPos.z - sphereRadius);
            Vector3 zAxisEnd = new Vector3(myPos.x, myPos.y, myPos.z + sphereRadius);
            Debug.DrawLine(xAxisStart, xAxisEnd, Color.green);
            Debug.DrawLine(zAxisStart, zAxisEnd, Color.green);

            // Draw line to obstacle
            //Debug.DrawLine(myPos, obstacle.position, Color.magenta);
            // Draw line showing direction of my velocity
            Debug.DrawLine(myPos, myPos + velocity.normalized * 3f, Color.blue);

        }

        if (CheckIfFacingTarget(myPos, velocity.normalized, obstacle.position))
        {
            // Draw line to obstacle
            Debug.DrawLine(myPos, obstacle.position, Color.magenta);

            // Only avoid the target if we are facing it
            Vector3 avoidanceForce = myPos - obstacle.position;
            // Crank up the force if an object is close to avoid getting stuck on it
            float closeForceMultiplier = hitsFromSphereClose.Count > 0 ? 5f : 1f;
            avoidanceForce = avoidanceForce.normalized * maxAvoidanceVelocity * closeForceMultiplier;

            return avoidanceForce;
        } else
        {
            // We are not facing the closest obstacle, ignore it
            return Vector3.zero;
        }
    }

    /*
     * Clamp all values of a vector to +/- maxValue.
     */
    public Vector3 ClampVector(Vector3 vector, float maxValue)
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

    /*
     * Checks if an object is facing a target, facing is considered to be
     * within +/- collisionFacingFovDeg/2 degrees of the myForwardDirection
     * direction.
     */
    private bool CheckIfFacingTarget (Vector3 myPosition, Vector3 myForwardDirection, Vector3 targetPosition)
    {
        Vector3 myForwardDirectionNrm = myForwardDirection.normalized;  // Just to be safe, force normalize it
        Vector3 directionToTarget = (targetPosition - myPosition).normalized;
        float dot = Vector3.Dot(myForwardDirectionNrm, directionToTarget);

        /*
         * Cosine of the fov angle, /2 because we want fov/2 degrees on both
         * sides of the forwardDirection vector. Converted to radians because
         * dot product is in radians.
         */
        float cosFovAngleRad = Mathf.Cos((collisionFacingFovDeg / 2) * Mathf.Deg2Rad);
        return dot > cosFovAngleRad;

    }

    private List<Collider> GetObstaclesWithinRadius (Vector3 center, float radius)
    {
        Collider[] hitsFromSphere = Physics.OverlapSphere(center, radius);

        // Convert the hits to list ordered by distance and containing only obstacles
        List<Collider> hits = new List<Collider>(hitsFromSphere);
        hits = hits.OrderBy(hit => Vector3.Distance(center, hit.transform.position)).ToList();
        List<Collider> obstacleHits = hits = hits.Where(hit => hit.CompareTag("Obstacle")).ToList();
        Debug.Log("Sphere of radius " + radius + " hit " + hits.Count + ", " + obstacleHits.Count + " were obstacles");

        return obstacleHits;

    }
}
