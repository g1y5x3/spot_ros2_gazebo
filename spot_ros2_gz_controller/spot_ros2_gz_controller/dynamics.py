import pinocchio
from pathlib import Path
from ament_index_python.packages import get_package_share_directory

def main():
    package_share_dir = get_package_share_directory('spot_ros2_description')
    pinocchio_model_dir = Path(package_share_dir) / "models" / "spot"
    urdf_filename = pinocchio_model_dir / "model.urdf"
    
    model = pinocchio.buildModelFromUrdf(str(urdf_filename))
    print("model name: " + model.name)

if __name__ == '__main__':
    main()