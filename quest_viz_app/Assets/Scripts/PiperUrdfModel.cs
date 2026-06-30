using System;
using System.Collections.Generic;
using System.IO;
using UnityEngine;
#if UNITY_EDITOR
using UnityEditor;
#endif

[Serializable]
public class PiperKinematicModel
{
    public string name;
    public string source_urdf;
    public string[] joint_order;
    public PiperLinkDescription[] links;
    public PiperJointDescription[] joints;
}

[Serializable]
public class PiperLinkDescription
{
    public string name;
    public PiperOrigin visual_origin;
    public string visual_mesh;
    public float[] mesh_scale;
    public bool fallback_geometry;
}

[Serializable]
public class PiperJointDescription
{
    public string name;
    public string type;
    public string parent;
    public string child;
    public PiperOrigin origin;
    public float[] axis;
    public PiperLimitDeg limit_deg;
    public float home_deg;
}

[Serializable]
public class PiperOrigin
{
    public float[] xyz;
    public float[] rpy;
}

[Serializable]
public class PiperLimitDeg
{
    public float lower;
    public float upper;
}

public class PiperUrdfModel : MonoBehaviour
{
    public string modelJsonFile = "piper_kinematic_model.json";
    public bool showJointAxes = false;
    public bool transparentGhost = false;
    public Material solidMaterial;
    public Material ghostMaterial;

    public PiperKinematicModel Model { get; private set; }

    readonly Dictionary<string, Transform> _links = new Dictionary<string, Transform>();
    readonly Dictionary<string, Transform> _jointTransforms = new Dictionary<string, Transform>();
    readonly Dictionary<string, PiperJointDescription> _jointByName = new Dictionary<string, PiperJointDescription>();
    readonly Dictionary<string, Quaternion> _jointOriginRotations = new Dictionary<string, Quaternion>();
    float[] _displayJoints = new float[6];

    void Start()
    {
        if (transform.childCount == 0)
        {
            LoadAndBuild();
        }
        else
        {
            LoadModelJson();
            CacheExistingJoints();
        }
    }

    public void LoadAndBuild()
    {
        ClearChildren();
        if (!LoadModelJson()) return;
        BuildHierarchy();
    }

    bool LoadModelJson()
    {
        string path = Path.Combine(Application.streamingAssetsPath, modelJsonFile);
        if (!File.Exists(path))
        {
            Debug.LogError($"Missing Piper kinematic model JSON: {path}");
            return false;
        }
        Model = JsonUtility.FromJson<PiperKinematicModel>(File.ReadAllText(path));
        return Model != null;
    }

    public void ApplyJointsDegrees(float[] targetDegrees, float smoothingSpeed = 20f)
    {
        if (targetDegrees == null || targetDegrees.Length < 6 || Model == null) return;
        for (int i = 0; i < 6; i++)
        {
            _displayJoints[i] = Mathf.Lerp(_displayJoints[i], targetDegrees[i], 1f - Mathf.Exp(-smoothingSpeed * Time.deltaTime));
            string jointName = Model.joint_order[i];
            if (!_jointTransforms.TryGetValue(jointName, out Transform jointTransform)) continue;
            PiperJointDescription joint = _jointByName[jointName];
            Vector3 axis = ToVector3(joint.axis, Vector3.forward).normalized;
            jointTransform.localRotation = _jointOriginRotations[jointName] * Quaternion.AngleAxis(_displayJoints[i], axis);
        }
    }

    void BuildHierarchy()
    {
        foreach (var link in Model.links)
        {
            var go = new GameObject(link.name);
            _links[link.name] = go.transform;
        }

        foreach (var joint in Model.joints)
        {
            if (!_links.TryGetValue(joint.parent, out Transform parent) || !_links.TryGetValue(joint.child, out Transform child)) continue;
            var jointGo = new GameObject(joint.name);
            jointGo.transform.SetParent(parent, false);
            jointGo.transform.localPosition = ToVector3(joint.origin?.xyz, Vector3.zero);
            jointGo.transform.localRotation = RpyToQuaternion(joint.origin?.rpy);
            child.SetParent(jointGo.transform, false);
            child.localPosition = Vector3.zero;
            child.localRotation = Quaternion.identity;
            _jointTransforms[joint.name] = jointGo.transform;
            _jointByName[joint.name] = joint;
            _jointOriginRotations[joint.name] = jointGo.transform.localRotation;
            if (showJointAxes && joint.type != "fixed") AddAxisMarker(jointGo.transform, ToVector3(joint.axis, Vector3.forward));
        }

        if (_links.TryGetValue("world", out Transform world))
        {
            world.SetParent(transform, false);
        }
        else if (_links.TryGetValue("base_link", out Transform baseLink))
        {
            baseLink.SetParent(transform, false);
        }

        foreach (var link in Model.links)
        {
            if (_links.TryGetValue(link.name, out Transform linkTransform))
            {
                AddVisual(link, linkTransform);
            }
        }
    }

    void CacheExistingJoints()
    {
        if (Model == null) return;
        foreach (var joint in Model.joints)
        {
            Transform found = FindDeepChild(transform, joint.name);
            if (found == null) continue;
            _jointTransforms[joint.name] = found;
            _jointByName[joint.name] = joint;
            _jointOriginRotations[joint.name] = found.localRotation;
        }
    }

    Transform FindDeepChild(Transform parent, string childName)
    {
        for (int i = 0; i < parent.childCount; i++)
        {
            Transform child = parent.GetChild(i);
            if (child.name == childName) return child;
            Transform found = FindDeepChild(child, childName);
            if (found != null) return found;
        }
        return null;
    }

    void AddVisual(PiperLinkDescription link, Transform parent)
    {
#if UNITY_EDITOR
        Mesh importedMesh = LoadEditorMesh(link.visual_mesh);
        if (importedMesh != null)
        {
            var meshGo = new GameObject($"{link.name}_visual_mesh");
            meshGo.transform.SetParent(parent, false);
            meshGo.transform.localPosition = ToVector3(link.visual_origin?.xyz, Vector3.zero);
            meshGo.transform.localRotation = RpyToQuaternion(link.visual_origin?.rpy);
            meshGo.transform.localScale = ToVector3(link.mesh_scale, Vector3.one);
            meshGo.AddComponent<MeshFilter>().sharedMesh = importedMesh;
            meshGo.AddComponent<MeshRenderer>().sharedMaterial = transparentGhost && ghostMaterial != null ? ghostMaterial : solidMaterial;
            return;
        }
#endif
        var visual = GameObject.CreatePrimitive(PrimitiveType.Cube);
        visual.name = $"{link.name}_visual_fallback";
        visual.transform.SetParent(parent, false);
        visual.transform.localPosition = ToVector3(link.visual_origin?.xyz, Vector3.zero);
        visual.transform.localRotation = RpyToQuaternion(link.visual_origin?.rpy);
        visual.transform.localScale = FallbackScale(link.name);
        var renderer = visual.GetComponent<Renderer>();
        renderer.sharedMaterial = transparentGhost && ghostMaterial != null ? ghostMaterial : solidMaterial;
    }

#if UNITY_EDITOR
    Mesh LoadEditorMesh(string visualMesh)
    {
        if (string.IsNullOrEmpty(visualMesh)) return null;
        string assetPath = $"Assets/{visualMesh}";
        Mesh direct = AssetDatabase.LoadAssetAtPath<Mesh>(assetPath);
        if (direct != null) return direct;
        foreach (Object asset in AssetDatabase.LoadAllAssetsAtPath(assetPath))
        {
            if (asset is Mesh mesh) return mesh;
        }
        return null;
    }
#endif

    Vector3 FallbackScale(string linkName)
    {
        if (linkName == "base_link") return new Vector3(0.16f, 0.16f, 0.08f);
        if (linkName == "link2") return new Vector3(0.30f, 0.055f, 0.055f);
        if (linkName == "link3") return new Vector3(0.055f, 0.26f, 0.055f);
        if (linkName == "link5") return new Vector3(0.045f, 0.13f, 0.045f);
        return new Vector3(0.07f, 0.07f, 0.07f);
    }

    void AddAxisMarker(Transform parent, Vector3 axis)
    {
        var marker = GameObject.CreatePrimitive(PrimitiveType.Cylinder);
        marker.name = "joint_axis";
        marker.transform.SetParent(parent, false);
        marker.transform.localScale = new Vector3(0.01f, 0.08f, 0.01f);
        marker.transform.localRotation = Quaternion.FromToRotation(Vector3.up, axis.normalized);
    }

    static Vector3 ToVector3(float[] values, Vector3 fallback)
    {
        if (values == null || values.Length < 3) return fallback;
        return new Vector3(values[0], values[1], values[2]);
    }

    static Quaternion RpyToQuaternion(float[] rpy)
    {
        if (rpy == null || rpy.Length < 3) return Quaternion.identity;
        return Quaternion.Euler(rpy[0] * Mathf.Rad2Deg, rpy[1] * Mathf.Rad2Deg, rpy[2] * Mathf.Rad2Deg);
    }

    void ClearChildren()
    {
        for (int i = transform.childCount - 1; i >= 0; i--)
        {
            Destroy(transform.GetChild(i).gameObject);
        }
        _links.Clear();
        _jointTransforms.Clear();
        _jointByName.Clear();
        _jointOriginRotations.Clear();
    }
}
