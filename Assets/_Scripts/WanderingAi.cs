using System.Collections;
using System.Collections.Generic;
using UnityEngine;

/*
 * Creates simple wandering by assigning random positions to seek to. This was
 * made before the wander behavior was implemented. Use wander instead, this
 * produces jerky, sudden direction changes.
 */
public class WanderingAi : MonoBehaviour
{
    [SerializeField] private Steering steering;
    [SerializeField] private Transform avoidTarget;
    [SerializeField] private Transform wanderTarget;
    [SerializeField] private float newTargetDistance = 5.1f;
    [SerializeField] private Vector3 wanderPosition = Vector3.zero;
    [SerializeField] private Vector2 WanderRange = new Vector2(40, 20); // +/-X, +/-Z

    private float fleeDistance;


    void Start()
    {
        GetNewWanderTarget();
        
        steering.SetSteeringBehavior(Steering.SteeringBehavior.Seek, wanderTarget);

        // We want the flee distance to always be slightly less than the stop distance so that this object always runs away a little bit before wandering again
        fleeDistance = steering.GetStopDistance() * .75f;
    }

    void Update()
    {
        Vector3 myPos = transform.position;
        Vector3 avoidTargetPos = avoidTarget.position;

        if (steering.GetSteeringBehavior() != Steering.SteeringBehavior.Flee && Vector3.Distance(myPos, avoidTargetPos) <= fleeDistance)
        {
            // If we are not already fleeing, and we are close enough to flee, flee
            steering.SetSteeringBehavior(Steering.SteeringBehavior.Flee, avoidTarget);

            // One we are done fleeing, we will have a new wander target
            GetNewWanderTarget();
        } else if (steering.GetSteeringBehavior() == Steering.SteeringBehavior.Flee && Vector3.Distance(myPos, avoidTargetPos) > fleeDistance)
        {
            // If we are fleeing, but we are far enough away, resume wandering
            steering.SetSteeringBehavior(Steering.SteeringBehavior.Seek, wanderTarget);
        } else if (Vector3.Distance(myPos, wanderPosition) < newTargetDistance)
        {
            GetNewWanderTarget();
        }

    }

    private void GetNewWanderTarget()
    {
        wanderPosition = new Vector3(
            Random.Range(-WanderRange.x, WanderRange.x),
            wanderPosition.y,
            Random.Range(-WanderRange.y, WanderRange.y));

        wanderTarget.position = wanderPosition;
    }
}
