import numpy as np
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from scipy.spatial.transform import Rotation as R
from pydrake.math import RollPitchYaw
from pydrake.common.eigen_geometry import Quaternion
# from pydrake.multibody.parsing import Parser
# from pydrake.multibody.plant import MultibodyPlant, JacobianWrtVariable

class RobotState:
    def __init__(self):

        # fixed order of the joints
        self.joint_names = [
            'front_left_hip_x', 'front_left_hip_y', 'front_left_knee',
            'front_right_hip_x', 'front_right_hip_y', 'front_right_knee',
            'rear_left_hip_x', 'rear_left_hip_y', 'rear_left_knee',
            'rear_right_hip_x', 'rear_right_hip_y', 'rear_right_knee'
        ]

        # joint states
        self.q = np.zeros(12)
        self.q_dot = np.zeros(12)
        
        # position and orientation states
        self.theta = np.zeros(3)
        self.p = np.zeros(3)
        self.omega = np.zeros(3)    # angular velocity
        self.p_dot = np.zeros(3)    # linear velocity
        
    def update_joints(self, msg: JointState):
        for i, name in enumerate(msg.name):
            if name in self.joint_names:
                idx = self.joint_names.index(name)
                self.q[idx] = msg.position[i]
                self.q_dot[idx] = msg.velocity[i]

    # TODO: estimate the state based on sensors inputs to replace ground truth
    # provided by gazebo 
    def update_pose(self, msg: Odometry):
        # position and pose velocity
        p = msg.pose.pose.position
        v = msg.twist.twist.linear

        self.p = np.array([p.x, p.y, p.z])
        self.p_dot = np.array([v.x, v.y, v.z])

        # orientation and angular velocity
        ang_q = msg.pose.pose.orientation
        ang_vel = msg.twist.twist.angular

        self.theta = self.quat_to_euler(ang_q)
        self.omega = np.array([ang_vel.x, ang_vel.y, ang_vel.z])

    def quat_to_euler(self, q):
        drake_quat = Quaternion(w=q.w, x=q.x, y=q.y, z=q.z)
        rpy = RollPitchYaw(drake_quat.rotation())
        
        return np.array([rpy.roll_angle(), rpy.pitch_angle(), rpy.yaw_angle()])

    def get_state_vec(self):
        """
        Returns the full 13-dimensional state vector required by the MPC controller.
    
        The state vector consists of:
        - θ (roll, pitch, yaw angles) [3]
        - p (position) [3]
        - ω (angular velocity) [3]
        - ṗ (linear velocity) [3]
        - g (gravity constant) [1]
    
        Returns:
            np.ndarray: 13-element state vector [θ, p, ω, ṗ, g]
        """
        state = np.zeros(13, dtype=np.float32)
    
        state[0:3] = self.theta  # [roll, pitch, yaw]
        state[3:6] = self.p
        state[6:9] = self.omega
        state[9:12] = self.p_dot
    
        # Add gravity
        state[12] = -9.81

        return state

    def __str__(self):
        output = "===== Robot State ===== \n"

        # Group joints by leg
        fl = f"FL: [{self.q[0]:.3f}, {self.q[1]:.3f},  {self.q[2]:.3f}]"
        fr = f"FR: [{self.q[3]:.3f}, {self.q[4]:.3f},  {self.q[5]:.3f}]"
        rl = f"RL: [{self.q[6]:.3f}, {self.q[7]:.3f},  {self.q[8]:.3f}]"
        rr = f"RR: [{self.q[9]:.3f}, {self.q[10]:.3f}, {self.q[11]:.3f}]"
   
        output += "joint positions (q):\n"
        output += f"{fl}\n{fr}\n{rl}\n{rr}\n"
   
        # Same format for velocities
        fl_v = f"FL: [{self.q_dot[0]:.3f}, {self.q_dot[1]:.3f},  {self.q_dot[2]:.3f}]"
        fr_v = f"FR: [{self.q_dot[3]:.3f}, {self.q_dot[4]:.3f},  {self.q_dot[5]:.3f}]"
        rl_v = f"RL: [{self.q_dot[6]:.3f}, {self.q_dot[7]:.3f},  {self.q_dot[8]:.3f}]"
        rr_v = f"RR: [{self.q_dot[9]:.3f}, {self.q_dot[10]:.3f}, {self.q_dot[11]:.3f}]"
   
        output += "\njoint velocities (q_dot):\n"
        output += f"{fl_v}\n{fr_v}\n{rl_v}\n{rr_v}\n"

        # Position and Velocity
        output += "\nposition (p):\n"
        output += f"[{self.p[0]:.3f}, {self.p[1]:.3f}, {self.p[2]:.3f}]\n"
   
        output += "\nvelocity (p_dot):\n" 
        output += f"[{self.p_dot[0]:.3f}, {self.p_dot[1]:.3f}, {self.p_dot[2]:.3f}]\n"

        # Orientation and Angular Velocity  
        output += "\norientation (theta):\n"
        output += f"[{self.theta[0]:.3f}, {self.theta[1]:.3f}, {self.theta[2]:.3f}]\n"

        output += "\nangular velocity (omega):\n"
        output += f"[{self.omega[0]:.3f}, {self.omega[1]:.3f}, {self.omega[2]:.3f}]\n"
        
        return output

if __name__ == "__main__":
    robot_state = RobotState()