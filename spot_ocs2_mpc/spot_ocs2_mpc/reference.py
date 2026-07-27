import math

from spot_effort_controller.constants import NOMINAL_STAND

from .contracts import NOMINAL_HEIGHT, validate_state


def clamp(value, lower, upper):
    return min(upper, max(lower, value))


def move_toward(current, target, maximum_step):
    return current + clamp(target - current, -maximum_step, maximum_step)


def body_to_world(vx, vy, yaw):
    cosine = math.cos(yaw)
    sine = math.sin(yaw)
    return cosine * vx - sine * vy, sine * vx + cosine * vy


class ReferenceGenerator:
    """Continuous, acceleration-limited planar base reference."""

    def __init__(
            self, max_vx=0.20, max_vy=0.15, max_wz=0.40,
            max_ax=0.30, max_ay=0.30, max_aw=0.60,
            nominal_height=NOMINAL_HEIGHT,
            default_joint_position=NOMINAL_STAND):
        limits = (max_vx, max_vy, max_wz, max_ax, max_ay, max_aw)
        if not all(math.isfinite(value) and value > 0.0 for value in limits):
            raise ValueError('all speed and acceleration limits must be finite')
        posture = tuple(float(value) for value in default_joint_position)
        if len(posture) != 12 or not all(
                math.isfinite(value) for value in posture):
            raise ValueError(
                'default joint position must contain 12 finite values')
        if not math.isfinite(nominal_height) or nominal_height <= 0.0:
            raise ValueError('nominal height must be finite and positive')
        self.max_vx = max_vx
        self.max_vy = max_vy
        self.max_wz = max_wz
        self.max_ax = max_ax
        self.max_ay = max_ay
        self.max_aw = max_aw
        self.nominal_height = float(nominal_height)
        self.default_joint_position = posture
        self.vx = 0.0
        self.vy = 0.0
        self.wz = 0.0
        self.x = None
        self.y = None
        self.yaw = None

    def anchor(self, x, y, yaw):
        if not all(math.isfinite(value) for value in (x, y, yaw)):
            raise ValueError('reference anchor must be finite')
        self.x, self.y, self.yaw = float(x), float(y), float(yaw)

    def update(self, command, estimated_yaw, dt, command_active=True):
        if self.x is None:
            raise RuntimeError('reference must be anchored first')
        if not math.isfinite(dt) or dt < 0.0:
            raise ValueError('reference dt must be finite and non-negative')
        desired = command if command_active else (0.0, 0.0, 0.0)
        if len(desired) != 3 or not all(
                math.isfinite(value) for value in desired):
            desired = (0.0, 0.0, 0.0)
        desired_vx = clamp(desired[0], -self.max_vx, self.max_vx)
        desired_vy = clamp(desired[1], -self.max_vy, self.max_vy)
        desired_wz = clamp(desired[2], -self.max_wz, self.max_wz)
        self.vx = move_toward(self.vx, desired_vx, self.max_ax * dt)
        self.vy = move_toward(self.vy, desired_vy, self.max_ay * dt)
        self.wz = move_toward(self.wz, desired_wz, self.max_aw * dt)
        world_vx, world_vy = body_to_world(
            self.vx, self.vy, estimated_yaw)
        self.x += world_vx * dt
        self.y += world_vy * dt
        self.yaw += self.wz * dt
        return world_vx, world_vy, self.wz

    def target_states(self, source_state, world_velocity, horizon):
        source = list(validate_state(source_state))
        if len(world_velocity) != 3 or horizon <= 0.0:
            raise ValueError('invalid target velocity or horizon')
        current = source.copy()
        future = source.copy()
        for target in (current, future):
            target[0:6] = [0.0] * 6
            target[0] = world_velocity[0]
            target[1] = world_velocity[1]
            target[8] = self.nominal_height
            target[10:12] = [0.0, 0.0]
            target[12:24] = self.default_joint_position
        current[6], current[7], current[9] = self.x, self.y, self.yaw
        future[6] = self.x + world_velocity[0] * horizon
        future[7] = self.y + world_velocity[1] * horizon
        future[9] = self.yaw + world_velocity[2] * horizon
        return tuple(current), tuple(future)
