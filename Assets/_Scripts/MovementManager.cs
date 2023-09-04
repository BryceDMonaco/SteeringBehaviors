using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class MovementManager : MonoBehaviour
{
    [SerializeField] private List<Steering.CommandPair> commands;
    [SerializeField] private int pursuitPredictAheadIterationWaits = 10;

    private Steering steering;
    private Rigidbody myRigidbody;
    private float maxVelocity;
    private int currentPredictWaits = 0;



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
