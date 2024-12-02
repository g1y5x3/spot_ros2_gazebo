import pinocchio
from pathlib import Path

def main():
    pinocchio_model_dir = Path(__file__).parent.parent.parent / "spot_ros2_description" / "models" / "spot"
    urdf_filename = pinocchio_model_dir / "model.urdf"
    model = pinocchio.buildModelFromUrdf(urdf_filename)
    print("model name: " + model.name)    

if __name__ == '__main__':
    main()