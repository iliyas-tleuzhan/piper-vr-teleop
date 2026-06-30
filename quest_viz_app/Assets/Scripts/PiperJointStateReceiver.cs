using System;
using System.Net;
using System.Net.Sockets;
using System.Text;
using System.Threading;
using UnityEngine;

[Serializable]
public class PiperJointStatePacket
{
    public string type;
    public double timestamp;
    public float[] commanded_joints_deg;
    public float[] measured_joints_deg;
    public float[] controller_xyz;
    public string state;
    public string reason;
    public string mode;
    public string mapping_mode;
    public float sample_age_s;
    public string action;
    public bool calibrated;
    public bool deadman;
}

public class PiperJointStateReceiver : MonoBehaviour
{
    public int port = 5055;
    public float staleTimeoutSeconds = 0.5f;
    public float interpolationSpeed = 14f;

    public PiperJointStatePacket LatestPacket { get; private set; }
    public bool HasPacket { get; private set; }
    public bool IsStale => !HasPacket || Time.time - _lastPacketUnityTime > staleTimeoutSeconds;
    public float LastPacketAgeSeconds => HasPacket ? Time.time - _lastPacketUnityTime : float.PositiveInfinity;

    readonly object _lock = new object();
    UdpClient _client;
    Thread _thread;
    volatile bool _running;
    float _lastPacketUnityTime = -999f;
    PiperJointStatePacket _pendingPacket;

    void OnEnable()
    {
        _running = true;
        _client = new UdpClient(port);
        _thread = new Thread(ReceiveLoop) { IsBackground = true };
        _thread.Start();
    }

    void Update()
    {
        lock (_lock)
        {
            if (_pendingPacket != null)
            {
                LatestPacket = _pendingPacket;
                _pendingPacket = null;
                HasPacket = true;
                _lastPacketUnityTime = Time.time;
            }
        }
    }

    void ReceiveLoop()
    {
        var endpoint = new IPEndPoint(IPAddress.Any, port);
        while (_running)
        {
            try
            {
                byte[] bytes = _client.Receive(ref endpoint);
                string json = Encoding.UTF8.GetString(bytes).Trim();
                var packet = JsonUtility.FromJson<PiperJointStatePacket>(json);
                if (packet != null && packet.type == "piper_joint_state")
                {
                    lock (_lock)
                    {
                        _pendingPacket = packet;
                    }
                }
            }
            catch (SocketException)
            {
                if (_running) Debug.LogWarning("Piper visualization UDP receive failed.");
            }
            catch (ObjectDisposedException)
            {
                return;
            }
            catch (Exception exc)
            {
                Debug.LogWarning($"Ignored malformed Piper visualization packet: {exc.Message}");
            }
        }
    }

    void OnDisable()
    {
        _running = false;
        _client?.Close();
        _client = null;
        if (_thread != null && _thread.IsAlive)
        {
            _thread.Join(100);
        }
        _thread = null;
    }
}
