using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.AI;

public class MovementManager : MonoBehaviour
{
    [SerializeField] private List<Steering.CommandPair> commands;
    [SerializeField] private int pursuitPredictAheadIterationWaits = 10;
    [SerializeField] private bool pathFollowNavAgent = false;
    [SerializeField] List<Transform> pathPoints = new List<Transform>();
    [SerializeField] private float pathFollowPointRadius = 3f;
    [SerializeField] private Steering.PathFollowBehavior pathFollowBehavior = Steering.PathFollowBehavior.FollowBack;
    [SerializeField] private GameObject waypointObj;

    private Steering steering;
    private Rigidbody myRigidbody;
    private float maxVelocity;
    private int currentPredictWaits = 0;
    [SerializeField] private int currentPathPointNdx = 0;
    private int pathFollowDirection = 1;  // 1 when going, -1 when going back
    private NavMeshAgent agent;
    private Vector3 lastPathFollowTargetPosition = Vector3.zero;

    void Start()
    {
        steering = GetComponent<Steering>();
        myRigidbody = GetComponent<Rigidbody>();
        maxVelocity = steering.GetMaxVelocity();

        if (pathFollowNavAgent)
        {
            agent = GetComponent<NavMeshAgent>();
        }
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
                    if (pathFollowNavAgent && (pathPoints.Count == 0 || lastPathFollowTargetPosition != command.target.position))
                    {
                        GenerateNewPath(command.target);
                    }

                    if (pathPoints.Count == 0)
                    {
                        Debug.Log("No path");
                        continue;
                    }

                    if (Vector3.Distance(transform.position, pathPoints[currentPathPointNdx].position) <= pathFollowPointRadius)
                    {
                        switch (pathFollowBehavior)
                        {
                            case Steering.PathFollowBehavior.FollowBack:
                                if ((currentPathPointNdx + 1 == pathPoints.Count && pathFollowDirection == 1) ||
                                (currentPathPointNdx == 0 && pathFollowDirection == -1))
                                {
                                    pathFollowDirection *= -1;  // Reverse direction
                                }

                                currentPathPointNdx += 1 * pathFollowDirection;
                                break;
                            case Steering.PathFollowBehavior.ReturnToStart:
                                currentPathPointNdx = (currentPathPointNdx + 1) % pathPoints.Count;
                                break;
                            case Steering.PathFollowBehavior.StopAtEnd:
                                if (currentPathPointNdx != pathPoints.Count - 1)
                                {
                                    currentPathPointNdx++;
                                }
                                break;
                            default:
                                Debug.Log("Unhandled PathFollowBehavior " + pathFollowBehavior);
                                break;
                        }
                    }
                    steeringVector += steering.Seek(pathPoints[currentPathPointNdx]);
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
        }
    }

    void GenerateNewPath (Transform target)
    {
        // Clear last path
        foreach (Transform waypoint in pathPoints)
        {
            Destroy(waypoint.gameObject);
        }

        pathPoints.Clear();

        lastPathFollowTargetPosition = target.position;
        NavMeshPath generatedPath = new NavMeshPath();
        bool success = agent.CalculatePath(target.position, generatedPath);

        Debug.Log("Path found=" + success + " Path size=" + generatedPath.corners.Length + " Status=" + generatedPath.status);

        foreach (Vector3 corner in generatedPath.corners)
        {
            GameObject waypoint = Instantiate(waypointObj, corner, Quaternion.identity);
            pathPoints.Add(waypoint.transform);
        }

        currentPathPointNdx = 0;
    }
}
