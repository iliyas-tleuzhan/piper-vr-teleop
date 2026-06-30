using System.Text;
using UnityEngine;
using UnityEngine.UI;

public class DebugHud : MonoBehaviour
{
    public PiperJointStateReceiver receiver;
    public Text text;

    void Update()
    {
        if (text == null) return;
        if (receiver == null || !receiver.HasPacket)
        {
            text.text = "WAITING FOR TELEOP DATA\nCALIBRATE: Press A\nHOLD RIGHTGRIP TO MOVE";
            return;
        }

        var packet = receiver.LatestPacket;
        var sb = new StringBuilder();
        if (receiver.IsStale) sb.AppendLine("STALE DATA WARNING");
        sb.AppendLine(packet.state == "FAULT" ? $"FAULT: {packet.reason}" : packet.state);
        sb.AppendLine(packet.calibrated ? "CALIBRATED" : "CALIBRATE: Press A");
        sb.AppendLine(packet.deadman ? "ACTIVE DEADMAN" : "HOLD RIGHTGRIP TO MOVE");
        sb.AppendLine($"reason: {packet.reason}");
        sb.AppendLine($"mode: {packet.mode} / {packet.mapping_mode}");
        sb.AppendLine($"age: {receiver.LastPacketAgeSeconds:0.000}s");
        sb.AppendLine($"cmd: {Format(packet.commanded_joints_deg)}");
        sb.AppendLine($"meas: {Format(packet.measured_joints_deg)}");
        sb.AppendLine($"err: {TrackingError(packet.commanded_joints_deg, packet.measured_joints_deg):0.0} deg");
        text.text = sb.ToString();
    }

    static string Format(float[] values)
    {
        if (values == null || values.Length < 6) return "null";
        return $"[{values[0]:0.0}, {values[1]:0.0}, {values[2]:0.0}, {values[3]:0.0}, {values[4]:0.0}, {values[5]:0.0}]";
    }

    static float TrackingError(float[] commanded, float[] measured)
    {
        if (commanded == null || measured == null || commanded.Length < 6 || measured.Length < 6) return 0f;
        float sum = 0f;
        for (int i = 0; i < 6; i++)
        {
            float diff = commanded[i] - measured[i];
            sum += diff * diff;
        }
        return Mathf.Sqrt(sum);
    }
}
