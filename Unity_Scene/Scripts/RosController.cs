using System;
using Unity.Robotics;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using Robotraj = RosMessageTypes.Moveit.RobotTrajectoryMsg;


namespace Unity.Robotics.UrdfImporter.Control
{

    public enum RotationDirection { None = 0, Positive = 1, Negative = -1 };
    public enum ControlType { PositionControl };
        public class RosController : MonoBehaviour
    {
        private ArticulationBody[] articulationChain;
        public float stiffness;
        public int artnum = 0;
        public float damping;
        public float forceLimit;
        public float speed = 5f; // Units: degree/s
        public float torque = 100f; // Units: Nm or N
        public float acceleration = 5f;// Units: m/s^2 / degree/s^2
        public ControlType control = ControlType.PositionControl;

        float delayTime = 0.1f;
        float lastLoggedTime = 0;

        // Buffer to store the last received message
        Robotraj lastReceivedMessage = null;

        void Start()
        {
            articulationChain = this.GetComponentsInChildren<ArticulationBody>();
            int defDyanmicVal = 10;
            foreach (ArticulationBody joint in articulationChain)
            //for (int i = 0; i < 6; i++)
            {
                //ArticulationBody joint = articulationChain[i];
                joint.gameObject.AddComponent<JointControl>();
                joint.jointFriction = defDyanmicVal;
                joint.angularDamping = defDyanmicVal;
                ArticulationDrive currentDrive = joint.xDrive;
                currentDrive.forceLimit = forceLimit;
                joint.xDrive = currentDrive;
            }
            ROSConnection.GetOrCreateInstance().Subscribe<Robotraj>("joint_pos", BufferPos);


        }

        void BufferPos(Robotraj positionrob)
        {
            lastReceivedMessage = positionrob;
        }

        void Update()
        {
            
            if (lastReceivedMessage != null && Time.time - lastLoggedTime >= delayTime)
            {
                
                UpdatePos(lastReceivedMessage);
                lastLoggedTime = Time.time;
                lastReceivedMessage = null;
            }
        }
        
        void UpdatePos(Robotraj positionrob)
        {
            for (int i = 0; i < 6; i++)
            {
                JointControl current = articulationChain[i + 2].GetComponent<JointControl>();
                if (i == 0)
                {
                    current.angle = (float)positionrob.joint_trajectory.points[0].positions[2];
                }
                else if (i == 2)
                {
                    current.angle = (float)positionrob.joint_trajectory.points[0].positions[0];
                }
                else
                {
                    current.angle = (float)positionrob.joint_trajectory.points[0].positions[i];
                }

            }
            //
            //Debug.Log(positionrob.joint_trajectory.points[0].positions[artnum]);
            //Debug.Log(articulationChain[artnum]);

        }
    }

}


