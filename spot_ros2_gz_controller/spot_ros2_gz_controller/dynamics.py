import sys
import pinocchio as pin
from pinocchio.visualize import MeshcatVisualizer
from pathlib import Path

from ament_index_python.packages import get_package_share_directory

def main():
    package_share_dir = get_package_share_directory('spot_ros2_description')
    model_dir = Path(package_share_dir) / "models" / "spot"
    urdf_filename = model_dir / "model.urdf"
    
    model, collision_model, visual_model = pin.buildModelsFromUrdf(str(urdf_filename))
    print("model name: " + model.name)

    data = model.createData()
    # Sample a random configuration
    q = pin.randomConfiguration(model)
    print(f"q: {q.T}")

    # Perform the forward kinematics over the kinematic tree
    pin.forwardKinematics(model, data, q)

if __name__ == '__main__':
    main()