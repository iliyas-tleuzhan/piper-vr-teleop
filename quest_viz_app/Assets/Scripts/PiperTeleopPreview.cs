using UnityEngine;

public class PiperTeleopPreview : MonoBehaviour
{
    public PiperJointStateReceiver receiver;
    public PiperUrdfModel commandedArm;
    public PiperUrdfModel measuredArm;
    public QuestControllerGhost controllerGhost;
    public GameObject workspaceGrid;

    public bool showCommandedArm = true;
    public bool showMeasuredArm = true;
    public bool showControllerGhost = true;
    public bool showJointAxes = false;
    public bool showWorkspaceGrid = true;

    void Update()
    {
        if (commandedArm != null)
        {
            commandedArm.gameObject.SetActive(showCommandedArm);
            commandedArm.showJointAxes = showJointAxes;
        }
        if (measuredArm != null)
        {
            measuredArm.gameObject.SetActive(showMeasuredArm);
            measuredArm.showJointAxes = showJointAxes;
        }
        if (workspaceGrid != null) workspaceGrid.SetActive(showWorkspaceGrid);
        if (controllerGhost != null) controllerGhost.gameObject.SetActive(showControllerGhost);

        if (receiver == null || !receiver.HasPacket) return;
        var packet = receiver.LatestPacket;
        if (commandedArm != null) commandedArm.ApplyJointsDegrees(packet.commanded_joints_deg, receiver.interpolationSpeed);
        if (measuredArm != null) measuredArm.ApplyJointsDegrees(packet.measured_joints_deg, receiver.interpolationSpeed);
        if (controllerGhost != null) controllerGhost.ApplyPacket(packet);
    }
}
