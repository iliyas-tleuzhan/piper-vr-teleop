using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;
using UnityEngine.UI;

public static class CreatePiperVizScene
{
    [MenuItem("Piper VR/Create Piper Viz Scene")]
    public static void CreateScene()
    {
        var scene = EditorSceneManager.NewScene(NewSceneSetup.EmptyScene, NewSceneMode.Single);

        var light = new GameObject("Directional Light").AddComponent<Light>();
        light.type = LightType.Directional;
        light.transform.rotation = Quaternion.Euler(50f, -30f, 0f);

        var camera = new GameObject("OpenXR Camera Rig");
        var cam = new GameObject("Main Camera").AddComponent<Camera>();
        cam.tag = "MainCamera";
        cam.transform.SetParent(camera.transform, false);
        cam.transform.localPosition = new Vector3(0f, 1.5f, -1.2f);
        cam.transform.localRotation = Quaternion.Euler(15f, 0f, 0f);

        var floor = GameObject.CreatePrimitive(PrimitiveType.Plane);
        floor.name = "Workspace Grid";
        floor.transform.localScale = new Vector3(1.2f, 1f, 1.2f);

        var receiver = new GameObject("Piper Joint State Receiver").AddComponent<PiperJointStateReceiver>();
        var preview = new GameObject("Piper Teleop Preview").AddComponent<PiperTeleopPreview>();
        preview.receiver = receiver;
        preview.workspaceGrid = floor;

        preview.commandedArm = CreateArm("Commanded Piper Arm", new Vector3(-0.35f, 0f, 0f), true);
        preview.measuredArm = CreateArm("Measured Piper Arm", new Vector3(0.35f, 0f, 0f), false);

        var ghost = GameObject.CreatePrimitive(PrimitiveType.Sphere);
        ghost.name = "Right Controller Ghost";
        ghost.transform.localScale = Vector3.one * 0.04f;
        preview.controllerGhost = ghost.AddComponent<QuestControllerGhost>();

        var canvasGo = new GameObject("Debug HUD");
        var canvas = canvasGo.AddComponent<Canvas>();
        canvas.renderMode = RenderMode.WorldSpace;
        canvasGo.AddComponent<CanvasScaler>();
        canvasGo.transform.position = new Vector3(0f, 1.45f, 0.7f);
        canvasGo.transform.rotation = Quaternion.Euler(0f, 180f, 0f);
        canvasGo.transform.localScale = Vector3.one * 0.004f;
        var textGo = new GameObject("HUD Text");
        textGo.transform.SetParent(canvasGo.transform, false);
        var text = textGo.AddComponent<Text>();
        text.font = Resources.GetBuiltinResource<Font>("Arial.ttf");
        text.fontSize = 24;
        text.color = Color.white;
        text.rectTransform.sizeDelta = new Vector2(760, 420);
        var hud = canvasGo.AddComponent<DebugHud>();
        hud.receiver = receiver;
        hud.text = text;

        EditorSceneManager.SaveScene(scene, "Assets/Scenes/PiperVizScene.unity");
    }

    static PiperUrdfModel CreateArm(string name, Vector3 position, bool ghost)
    {
        var go = new GameObject(name);
        go.transform.position = position;
        var model = go.AddComponent<PiperUrdfModel>();
        model.transparentGhost = ghost;
        model.LoadAndBuild();
        return model;
    }
}
