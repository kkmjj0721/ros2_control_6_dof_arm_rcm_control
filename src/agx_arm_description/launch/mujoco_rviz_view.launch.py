from pathlib import Path
import tempfile
import textwrap

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


PACKAGE_NAME = "agx_arm_description"


MUJOCO_VIEWER_CODE = """
import sys

try:
    import mujoco
    import mujoco.viewer
except Exception as exc:
    print(f"Failed to import mujoco: {exc}", file=sys.stderr)
    sys.exit(1)

model = mujoco.MjModel.from_xml_path(sys.argv[1])
data = mujoco.MjData(model)
mujoco.viewer.launch(model, data)
"""


def _candidate_package_roots():
    try:
        yield Path(get_package_share_directory(PACKAGE_NAME))
    except PackageNotFoundError:
        pass

    launch_file = Path(__file__).resolve()
    yield launch_file.parents[1]

    for parent in launch_file.parents:
        yield parent / "src" / PACKAGE_NAME

    yield Path.cwd() / "src" / PACKAGE_NAME


def _find_package_root():
    for candidate in _candidate_package_roots():
        if (candidate / "models" / "xml" / "piper.xml").is_file():
            return candidate

    raise FileNotFoundError(
        "Could not find agx_arm_description/models/xml/piper.xml. "
        "Run this launch from the workspace root or install the package models."
    )


def _load_robot_description(package_root):
    urdf_path = package_root / "models" / "urdf" / "piper" / "urdf" / "piper_description.urdf"
    mesh_dir = (package_root / "models" / "meshes").resolve()

    robot_description = urdf_path.read_text(encoding="utf-8")
    mesh_uri = mesh_dir.as_uri()
    robot_description = robot_description.replace(
        "package://agx_arm_description/agx_arm_urdf/piper/meshes/dae/",
        f"{mesh_uri}/dae/",
    )
    robot_description = robot_description.replace(
        "package://agx_arm_description/agx_arm_urdf/piper/meshes/",
        f"{mesh_uri}/",
    )
    return robot_description


def _write_default_rviz_config():
    rviz_config_path = Path(tempfile.gettempdir()) / "agx_arm_mujoco_rviz_view.rviz"
    rviz_config_path.write_text(
        textwrap.dedent(
            """\
            Panels:
              - Class: rviz_common/Displays
                Name: Displays
            Visualization Manager:
              Class: ""
              Displays:
                - Alpha: 0.5
                  Cell Size: 1
                  Class: rviz_default_plugins/Grid
                  Enabled: true
                  Name: Grid
                  Plane: XY
                  Value: true
                - Class: rviz_default_plugins/RobotModel
                  Description Source: Topic
                  Description Topic:
                    Depth: 5
                    Durability Policy: Transient Local
                    History Policy: Keep Last
                    Reliability Policy: Reliable
                    Value: /robot_description
                  Enabled: true
                  Name: RobotModel
                  Robot Description: robot_description
                  TF Prefix: ""
                  Update Interval: 0
                  Value: true
              Enabled: true
              Fixed Frame: base_link
              Global Options:
                Background Color: 48; 48; 48
                Fixed Frame: base_link
              Name: root
              Tools:
                - Class: rviz_default_plugins/Interact
                - Class: rviz_default_plugins/MoveCamera
                - Class: rviz_default_plugins/Select
              Value: true
            Window Geometry:
              Height: 900
              Width: 1200
            """
        ),
        encoding="utf-8",
    )
    return str(rviz_config_path)


def _launch_setup(context, *args, **kwargs):
    package_root = _find_package_root()
    mujoco_xml_path = package_root / "models" / "xml" / "piper.xml"
    robot_description = _load_robot_description(package_root)

    rviz_config = LaunchConfiguration("rviz_config").perform(context)
    if not rviz_config:
        rviz_config = _write_default_rviz_config()

    return [
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[{"robot_description": robot_description}],
        ),
        Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui",
            name="joint_state_publisher_gui",
            output="screen",
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            arguments=["-d", rviz_config],
        ),
        ExecuteProcess(
            cmd=["python3", "-c", MUJOCO_VIEWER_CODE, str(mujoco_xml_path)],
            name="mujoco_viewer",
            output="screen",
        ),
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "rviz_config",
                default_value="",
                description="Optional RViz config file. Defaults to a temporary RobotModel view.",
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
