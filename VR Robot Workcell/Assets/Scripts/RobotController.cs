using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using UnityEngine.UI;

using RosMessageTypes.UnityMoveitPlanning;

public class RobotController : MonoBehaviour
{
    private ROSConnection communicator;

    public GameObject robot;

    public ArticulationBody[] armJoints;
    public ArticulationBody[] gripperJoints;

    private float[] mimicFactor;

    private string FKService = "arm_fk_service";

    private readonly float jointAssignmentWait = 0.075f;
    private readonly float poseAssignmentWait = 0.15f;

    public Slider[] jointSettings;

    // Start is called before the first frame update
    void Start()
    {
        communicator = ROSConnection.instance;
    }

    // Update is called once per frame
    void Update()
    {
    }

    // Awake is called when the script is loaded
    private void Awake() 
    {
        this.robot = this.gameObject;

        // ALxN = Arm Link x Name 
        string AL0N = "base_link/base_link_inertia/shoulder_link";
        string AL1N = AL0N + "/upper_arm_link";
        string AL2N = AL1N + "/forearm_link";
        string AL3N = AL2N + "/wrist_1_link";
        string AL4N = AL3N + "/wrist_2_link";
        string AL5N = AL4N + "/wrist_3_link";

        armJoints = new ArticulationBody[] {
            this.robot.transform.Find(AL0N).GetComponent<ArticulationBody>(),
            this.robot.transform.Find(AL1N).GetComponent<ArticulationBody>(),
            this.robot.transform.Find(AL2N).GetComponent<ArticulationBody>(),
            this.robot.transform.Find(AL3N).GetComponent<ArticulationBody>(),
            this.robot.transform.Find(AL4N).GetComponent<ArticulationBody>(),
            this.robot.transform.Find(AL5N).GetComponent<ArticulationBody>()
        };

        /*
            GL = Gripper Link
            BN = Base Name
            REF = reference Finger Link in URDF mimic
            IK = inner knuckle
            IF = inner finger
            M = mirror

            basically, right mimics left, using mimic factor for directions
            left_outer_knuckle will drives the rest of the joints
        */
        string GLBN   = AL5N  + "/flange/tool0/robotiq_arg2f_base_link";    // gripper base
        string GLREF  = GLBN  + "/left_outer_knuckle";                      // left_outer_knuckle
        string GLIK   = GLBN  + "/left_inner_knuckle";                      // left_inner_knuckle
        string GLIF   = GLREF + "/left_outer_finger/left_inner_finger";     // left_inner_finger
        string GLMREF = GLBN  + "/right_outer_knuckle";                     // right_outer_knuckle
        string GLMIK  = GLBN  + "/right_inner_knuckle";                     // right_inner_knuckle
        string GLMIF  = GLMREF + "/right_outer_finger/right_inner_finger";  // right_inner_finger

        gripperJoints = new ArticulationBody[] {
            this.robot.transform.Find(GLREF).GetComponent<ArticulationBody>(),
            this.robot.transform.Find(GLIK).GetComponent<ArticulationBody>(),
            this.robot.transform.Find(GLIF).GetComponent<ArticulationBody>(),
            this.robot.transform.Find(GLMREF).GetComponent<ArticulationBody>(),
            this.robot.transform.Find(GLMIK).GetComponent<ArticulationBody>(),
            this.robot.transform.Find(GLMIF).GetComponent<ArticulationBody>()
        };

        mimicFactor = new float[] { 1.000f,    // left outer knuckle    
                                    1.000f,    // left inner knuckle
                                   -1.000f,    // left inner finger
                                    1.000f,    // right outer knuckle 
                                    1.000f,    // right inner knuckle
                                   -1.000f};   // right inner finger

        GameObject sliderGroup = GameObject.Find("SliderGroup");
        jointSettings = new Slider[] {
            sliderGroup.transform.Find("ShoulderPanFKSlider").GetComponent<Slider>(),
            sliderGroup.transform.Find("ShoulderLiftFKSlider").GetComponent<Slider>(),
            sliderGroup.transform.Find("ElbowFKSlider").GetComponent<Slider>(),
            sliderGroup.transform.Find("Wrist1FKSlider").GetComponent<Slider>(),
            sliderGroup.transform.Find("Wrist2FKSlider").GetComponent<Slider>(),
            sliderGroup.transform.Find("Wrist3FKSlider").GetComponent<Slider>()
        };
    }

    public void RequestFK()
    {
        ForwardKinematicsRequest fk = new ForwardKinematicsRequest();
        fk.target = new URJointsMsg();
        fk.target.joint_00 = jointSettings[0].value * Mathf.Deg2Rad;
        fk.target.joint_01 = jointSettings[1].value * Mathf.Deg2Rad;
        fk.target.joint_02 = jointSettings[2].value * Mathf.Deg2Rad;
        fk.target.joint_03 = jointSettings[3].value * Mathf.Deg2Rad;
        fk.target.joint_04 = jointSettings[4].value * Mathf.Deg2Rad;
        fk.target.joint_05 = jointSettings[5].value * Mathf.Deg2Rad;

        communicator.SendServiceMessage<ForwardKinematicsResponse>(FKService, fk, HandleForwardKinematics);
    }

    private void HandleForwardKinematics(ForwardKinematicsResponse response)
    {
        if (response.trajectories.Length > 0)
        {
            Debug.Log("FK trajectory returned. Executing...");
            StartCoroutine(ExecuteTrajectories(response));
        }
        else
        {
            Debug.LogError("FK trajectory not found!");
        }
    }

    private IEnumerator ExecuteTrajectories(ForwardKinematicsResponse response)
    {
        if (response.trajectories != null)
        {
            // For every trajectory plan returned
            for (int poseIndex = 0; poseIndex < response.trajectories.Length; poseIndex++)
            {
                // For every robot pose in trajectory plan
                for (int jointConfigIndex = 0; jointConfigIndex < response.trajectories[poseIndex].joint_trajectory.points.Length; jointConfigIndex++)
                {
                    var jointPositions = response.trajectories[poseIndex].joint_trajectory.points[jointConfigIndex].positions;
                    float[] result = jointPositions.Select(r => (float)r * Mathf.Rad2Deg).ToArray();

                    // Set the joint values for every joint
                    for (int joint = 0; joint < armJoints.Length; joint++)
                    {
                        var xdrive = armJoints[joint].xDrive;
                        xdrive.target = result[joint];
                        armJoints[joint].xDrive = xdrive;
                    }
                    // Wait for robot to achieve pose for all joint assignments
                    yield return new WaitForSeconds(jointAssignmentWait);
                }

                // Wait for the robot to achieve the final pose from joint assignment
                yield return new WaitForSeconds(poseAssignmentWait);
            }
        }
    }
}

/*

*/