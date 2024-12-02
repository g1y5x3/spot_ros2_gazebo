import pinocchio as pin
from pathlib import Path

import pinocchio
import os

# This will show you where pinocchio is installed
print(f"Pinocchio package location: {os.path.dirname(pinocchio.__file__)}")

# To specifically find pinocchio_pywrap_default
try:
    from pinocchio import pinocchio_pywrap_default
    print(f"Pywrap default location: {pinocchio_pywrap_default.__file__}")
except ImportError as e:
    print(f"Import error: {e}")

print(pin.__version__)

model_dir = Path(__file__).parent.parent.parent / "spot_ros2_description" / "models" / "spot"
print(model_dir)

sdf_filename = model_dir / "model.sdf"
if not sdf_filename.exists():
    raise FileNotFoundError(f"Could not find model file at {sdf_filename}")

model = pin.RobotWrapper.BuildFromSDF(
    str(sdf_filename),
    [],
    pin.JointModelFreeFlyer)
print("model name: " + model.name)

# The rest of your code remains the same
# data = model.createData()
# q = pinocchio.randomConfiguration(model)
# print(f"q: {q.T}")

# pinocchio.forwardKinematics(model, data, q)

# for name, oMi in zip(model.names, data.oMi):
#     print("{:<24} : {: .2f} {: .2f} {: .2f}".format(name, *oMi.translation.T.flat))