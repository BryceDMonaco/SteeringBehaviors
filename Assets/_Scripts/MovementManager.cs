using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class MovementManager : MonoBehaviour
{
    [SerializeField] private List<Steering.CommandPair> commands;
    [SerializeField] private int pursuitPredictAheadIterationWaits = 10;
    [SerializeField] Transform[] pathPoints;
    [SerializeField] private float pathFollowPointRadius = 3f;
    [SerializeField] private bool pathGoBack = true;

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
                    steeringVector += steering.Seek(command.target);
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
                    if (Vector3.Distance(transform.position, pathPoints[currentPathPointNdx].position) <= pathFollowPointRadius)
                    {
                        if (pathGoBack)
                        {
                            if ((currentPathPointNdx + 1 == pathPoints.Length && pathFollowDirection == 1) ||
                                (currentPathPointNdx == 0 && pathFollowDirection == -1))
                            {
                                pathFollowDirection *= -1;  // Reverse direction
                            }

                            currentPathPointNdx += 1 * pathFollowDirection;
                        } else
                        {
                            currentPathPointNdx = (currentPathPointNdx + 1) % pathPoints.Length;
                        }
                    }
                    steeringVector += steering.Seek(pathPoints[currentPathPointNdx]);
                    break;
                default:
                    Debug.LogError("Unhandled steering type");
                    break;
            }

            steeringVector = steering.ClampVector(steeringVector, maxVelocity);
            steeringVector = steeringVector / myRigidbody.mass;
            Vector3 velocity = steering.ClampVector(myRigidbody.velocity + steeringVector, maxVelocity);

            myRigidbody.velocity = velocity;
        }
    }
}
