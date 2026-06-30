using UnityEngine;

public class QuestControllerGhost : MonoBehaviour
{
    public Transform marker;
    public LineRenderer movementVector;
    public float positionScale = 1f;

    Vector3 _lastPosition;
    bool _hasLast;

    public void ApplyPacket(PiperJointStatePacket packet)
    {
        if (packet.controller_xyz == null || packet.controller_xyz.Length < 3) return;
        Vector3 pos = new Vector3(packet.controller_xyz[0], packet.controller_xyz[1], packet.controller_xyz[2]) * positionScale;
        if (marker == null) marker = transform;
        marker.localPosition = pos;
        if (movementVector != null)
        {
            movementVector.positionCount = 2;
            movementVector.SetPosition(0, marker.parent != null ? marker.parent.TransformPoint(_hasLast ? _lastPosition : pos) : (_hasLast ? _lastPosition : pos));
            movementVector.SetPosition(1, marker.parent != null ? marker.parent.TransformPoint(pos) : pos);
        }
        _lastPosition = pos;
        _hasLast = true;
    }
}
