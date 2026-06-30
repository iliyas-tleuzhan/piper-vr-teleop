from pathlib import Path

from scripts.convert_piper_urdf_to_unity_json import convert_urdf


def test_fake_urdf_converts_joint_origin_axis_and_mesh(tmp_path: Path):
    urdf = tmp_path / "fake.urdf"
    urdf.write_text(
        """<?xml version="1.0"?>
<robot name="fake_piper">
  <link name="base_link">
    <visual>
      <origin xyz="0.1 0.2 0.3" rpy="0.4 0.5 0.6"/>
      <geometry>
        <mesh filename="package://agx_arm_description/agx_arm_urdf/piper/meshes/dae/base_link.dae" scale="1 2 3"/>
      </geometry>
    </visual>
  </link>
  <link name="link1"/>
  <joint name="joint1" type="revolute">
    <origin xyz="0 0 0.123" rpy="0 0 1.57"/>
    <parent link="base_link"/>
    <child link="link1"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.5707963" upper="1.5707963" effort="1" velocity="1"/>
  </joint>
</robot>
""",
        encoding="utf-8",
    )

    model = convert_urdf(urdf)

    assert model["name"] == "fake_piper"
    assert model["joint_order"] == ["joint1"]
    assert model["links"][0]["visual_mesh"] == "Models/Piper/dae/base_link.dae"
    assert model["links"][0]["visual_origin"]["xyz"] == [0.1, 0.2, 0.3]
    assert model["links"][0]["mesh_scale"] == [1.0, 2.0, 3.0]
    joint = model["joints"][0]
    assert joint["origin"]["xyz"] == [0.0, 0.0, 0.123]
    assert joint["origin"]["rpy"] == [0.0, 0.0, 1.57]
    assert joint["axis"] == [0.0, 1.0, 0.0]
    assert round(joint["limit_deg"]["lower"], 1) == -90.0
    assert round(joint["limit_deg"]["upper"], 1) == 90.0
