using System.Collections;
using System.Collections.Generic;
using UnityEngine;

/**
 *  Handles the movement for a single gameobject.
 */
[RequireComponent(typeof(Steering))]
public class MovementManager : MonoBehaviour
{
    public enum PathFollowEndBehavior
    {
        LoopFromStart,
        ReverseOrderToStart,
        Stop
    }
    
    [SerializeField] private List<Steering.CommandPair> commands;
    [Header("Pursuit Settings")]
    [SerializeField] private int pursuitPredictAheadIterationWaits = 10;
    [Header("Path Follow Settings")]
    [SerializeField] Transform[] pathPoints;
    [SerializeField] private float pathFollowPointRadius = 3f;
    [SerializeField] private PathFollowEndBehavior pathGoBackBehavior = PathFollowEndBehavior.ReverseOrderToStart;
    [Header("Other Settings")]
    [SerializeField] private bool lookTowardsVelocity = true;

    private Steering steering;
    private Rigidbody myRigidbody;
    private float maxVelocity;
    private int currentPredictWaits = 0;
    [SerializeField] private int currentPathPointNdx = 0;
    private int pathFollowDirection = 1;  // 1 when going, -1 when going back



    void Start()
    {
        steering = GetComponent<Steering>();
        myRigidbody = GetComponent<Rigidbody>();
        maxVelocity = steering.GetMaxVelocity();
    }

    void FixedUpdate()
    {
        Vector3 steeringVector = Vector3.zero;

        foreach (Steering.CommandPair command in commands) {
            switch (command.command)
            {
                case Steering.SteeringBehavior.Seek:
                    if (Vector3.Distance(transform.position, command.target.position) < steering.GetSlowDistance())
                    {
                        steeringVector += steering.Arrival(command.target);
                    } else
                    {
                        steeringVector += steering.Seek(command.target);
                    }
                    break;
                case Steering.SteeringBehavior.Flee:
                    steeringVector += steering.Flee(command.target);
                    break;
                case Steering.SteeringBehavior.Wander:
                    steeringVector += steering.Wander();
                    break;
                case Steering.SteeringBehavior.Pursue:
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
                        steeringVector = steering.Pursue(command.target);
                    }
                    else
                    {
                        currentPredictWaits++;
                    }
                    break;
                case Steering.SteeringBehavior.Evade:
                    steeringVector = steering.Evade(command.target);
                    break;
                case Steering.SteeringBehavior.CollisionAvoid:
                    steeringVector += steering.CollisionAvoidance();
                    break;
                case Steering.SteeringBehavior.PathFollow:
                    bool stopAtLast = false;

                    if (Vector3.Distance(transform.position, pathPoints[currentPathPointNdx].position) <= pathFollowPointRadius)
                    {
                        if (pathGoBackBehavior == PathFollowEndBehavior.LoopFromStart)
                        {
                            currentPathPointNdx = (currentPathPointNdx + 1) % pathPoints.Length;
                        }
                        if (pathGoBackBehavior == PathFollowEndBehavior.ReverseOrderToStart)
                        {
                            if ((currentPathPointNdx + 1 == pathPoints.Length && pathFollowDirection == 1) ||
                                (currentPathPointNdx == 0 && pathFollowDirection == -1))
                            {
                                // Reverse direction
                                pathFollowDirection *= -1;
                            }

                            currentPathPointNdx += 1 * pathFollowDirection;
                        } else if (pathGoBackBehavior == PathFollowEndBehavior.Stop)
                        {
                            if (currentPathPointNdx < pathPoints.Length - 1)
                            {
                                currentPathPointNdx++;
                            } else
                            {
                                currentPathPointNdx = pathPoints.Length - 1;
                                stopAtLast = true;
                            }
                        }
                    }

                    if (stopAtLast)
                    {
                        steeringVector += steering.Arrival(pathPoints[currentPathPointNdx]);
                    } else
                    {
                        steeringVector += steering.Seek(pathPoints[currentPathPointNdx]);
                    }
                    break;
                case Steering.SteeringBehavior.LeaderFollow:
                    steeringVector += steering.LeaderFollow(command.target);
                    break;
                default:
                    Debug.LogError("Unhandled steering type in " + gameObject.name + ": " + command.command);
                    break;
            }

            steeringVector = steering.ClampVector(steeringVector, maxVelocity);
            steeringVector = steeringVector / myRigidbody.mass;
            Vector3 velocity = steering.ClampVector(myRigidbody.velocity + steeringVector, maxVelocity);

            myRigidbody.velocity = velocity;

            if (lookTowardsVelocity)
            {
                LookTowardsVelocity(velocity);
            }
        }
    }

    void LookTowardsVelocity (Vector3 velocity)
    {
        transform.rotation = Quaternion.LookRotation(velocity.normalized);
    }
}
