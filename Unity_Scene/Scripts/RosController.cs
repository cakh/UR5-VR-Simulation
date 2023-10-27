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
        public float damping;
        public float forceLimit;
        public float speed = 5f; // Units: degree/s
        public float torque = 100f; // Units: Nm or N
        public float acceleration = 5f;// Units: m/s^2 / degree/s^2
        public ControlType control = ControlType.PositionControl;
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
                Debug.Log(joint);
            }
            
        }
        void Update()
        {
            ROSConnection.GetOrCreateInstance().Subscribe<Robotraj>("joint_pos", UpdatePos);
        }
        
        void UpdatePos(Robotraj positionrob)
        {
            for (int i = 0; i < 6; i++)
            {
                JointControl current = articulationChain[i+2].GetComponent<JointControl>();
                //Debug.Log(positionrob.joint_trajectory.points[0].positions[3]);
                current.angle = (float)positionrob.joint_trajectory.points[0].positions[i];
            }

        }
    }

}


