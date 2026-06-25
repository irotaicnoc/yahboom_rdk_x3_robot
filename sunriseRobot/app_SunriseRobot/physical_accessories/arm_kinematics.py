import math
import xml.etree.ElementTree as ET

import numpy as np

import global_constants as gc


class ArmKinematics:
    """
    Analytical (closed-form) forward and inverse kinematics tailored to this specific 6-servo arm.

    This replaces the previous generic numerical optimiser (ikpy's inverse_kinematic_optimization), which
    searched iteratively for a solution on every call and was the cause of the laggy arm in IK mode. Because
    the geometry of this exact arm is known, the joint angles can be computed directly with trigonometry, in
    one shot, which is orders of magnitude faster and deterministic.

    Structure of the arm:
      - motor 0 (joint1) is the base yaw, rotating about the vertical axis.
      - motors 1, 2, 3 (joint2, joint3, joint4) are three parallel pitch joints forming a planar arm in a
        vertical plane: shoulder -> elbow -> wrist, plus a final fixed segment out to the controlled gripper
        point. The base yaw orients that plane.
      - motor 4 (gripper rotation) and motor 5 (gripper opening) do not move the gripper point, so they are
        irrelevant here and handled elsewhere.

    Conventions match the previous ikpy setup so callers see unchanged gripper coordinates:
      - position is (x, y, z) in the base frame, in metres: x = left/right, y = forward/backward, z = up/down.
      - a servo angle s (degrees) for motor i maps to its joint rotation q (radians) as q = radians(s - 90),
        i.e. s = 90 deg is the joint's zero/reference. Inversely, s = degrees(q) + 90.

    Redundancy: four joints positioning a 3D point leaves one extra degree of freedom, the gripper's pitch
    (tilt). inverse_kinematics() resolves it by HOLDING the current gripper pitch, which suits one-shot
    absolute moves (e.g. driving to a detected object). For live joystick teleop, resolved_rate() instead
    spreads the commanded motion across all four joints through the Jacobian and lets the pitch float, so the
    arm re-poses itself (flattens out, lifts the elbow) to keep following the stick. Holding the pitch
    rigidly traps the arm at full extension, where it jams a joint against its limit instead of rearranging.

    Forward and inverse kinematics use the same simplified planar model, so they are exact inverses of each
    other (entering IK mode causes no snap, and there is no drift). The model drops sub-degree URDF
    calibration offsets (a small base roll and < 1 mm link offsets); the resulting deviation from the raw
    URDF is a few millimetres (about 3 mm worst case), well under the arm's servo resolution, and invisible since the gripper position is an
    internal coordinate the user drives by hand.
    """

    # Geometry measured from urdf/arm.urdf (metres / radians). Used only if the URDF cannot be read.
    _FALLBACK_GEOMETRY = {'h': 0.10750, 'a2': 0.08285, 'a3': 0.08285, 'a4': 0.07385, 'delta4': 0.0083081}

    # Joint rotation limits (radians). Servos run 0..180 deg and q = servo - 90 deg, so q is in [-90, 90] deg
    # (which also matches the URDF joint limits of +-1.5708 rad). Used to clip resolved-rate motion.
    _Q_MIN = -math.pi / 2
    _Q_MAX = math.pi / 2

    def __init__(self, urdf_path: str = None, verbose: int = 0):
        self.verbose = verbose
        geometry = self._read_geometry(urdf_path or (gc.URDF_FOLDER_PATH + 'arm.urdf'))
        self.h = geometry['h']            # base -> shoulder height
        self.a2 = geometry['a2']          # shoulder -> elbow segment length
        self.a3 = geometry['a3']          # elbow -> wrist segment length
        self.a4 = geometry['a4']          # wrist -> controlled gripper point segment length
        self.delta4 = geometry['delta4']  # small in-plane mounting yaw baked into joint4

    def _read_geometry(self, urdf_path: str) -> dict:
        """Read the arm segment lengths from the URDF (single source of truth, same file ikpy used)."""
        try:
            joints = {}
            for joint in ET.parse(urdf_path).getroot().findall('joint'):
                origin = joint.find('origin')
                xyz = [float(v) for v in origin.get('xyz').split()]
                rpy = [float(v) for v in (origin.get('rpy') or '0 0 0').split()]
                joints[joint.get('name')] = {'xyz': xyz, 'rpy': rpy}
            geometry = {
                'h': joints['joint1']['xyz'][2] + joints['joint2']['xyz'][2],
                'a2': abs(joints['joint3']['xyz'][0]),
                'a3': abs(joints['joint4']['xyz'][0]),
                'a4': abs(joints['joint5']['xyz'][0]),
                'delta4': joints['joint4']['rpy'][2],
            }
            if self.verbose >= 2:
                print(f'Arm kinematics geometry read from URDF: {geometry}')
            return geometry
        except Exception as e:
            # lazy import so this module stays light and importable off-robot
            import utils
            utils.print_exception(exception=e, message=f'Could not read arm geometry from "{urdf_path}", '
                                                       f'falling back to measured values')
            return dict(self._FALLBACK_GEOMETRY)

    @staticmethod
    def _servos_to_q(servo_angles) -> list:
        # motors 0..3 (servo degrees) -> joint rotations q1..q4 (radians); q = radians(servo - 90)
        return [math.radians(servo_angles[i] - 90) for i in range(4)]

    @staticmethod
    def _q_to_servos(joint_angles) -> list:
        # joint rotations q1..q4 (radians) -> motors 0..3 (servo degrees); servo = degrees(q) + 90
        return [math.degrees(joint_angles[i]) + 90 for i in range(4)]

    def forward_kinematics(self, servo_angles) -> list:
        """
        Position (x, y, z) in metres of the gripper point, from the first four servo angles (degrees).
        Only motors 0..3 affect the gripper point, so servo_angles needs at least four entries.
        """
        return self._fk_from_joints(self._servos_to_q(servo_angles))

    def _fk_from_joints(self, joint_angles) -> list:
        # Gripper point (x, y, z) in metres from the joint rotations q1..q4 (radians). Shared by the public
        # forward kinematics and by the Jacobian for resolved-rate control (which perturbs the joint angles).
        q1, q2, q3, q4 = joint_angles
        # absolute in-plane angle of each segment (the final segment carries the small joint4 mounting yaw)
        angle_a2 = q2
        angle_a3 = q2 + q3
        angle_a4 = q2 + q3 + q4 + self.delta4
        # planar arm endpoint, expressed as wx (along the negative reach axis) and wy (radial reach)
        wx = -(self.a2 * math.cos(angle_a2) + self.a3 * math.cos(angle_a3) + self.a4 * math.cos(angle_a4))
        wy = -(self.a2 * math.sin(angle_a2) + self.a3 * math.sin(angle_a3) + self.a4 * math.sin(angle_a4))
        return [-wy * math.sin(q1), wy * math.cos(q1), self.h - wx]

    def inverse_kinematics(self, target, current_angles) -> list:
        """
        Servo angles (degrees) for motors 0..3 that put the gripper point at target = (x, y, z), holding the
        current gripper pitch. current_angles are the present servo angles (degrees) for motors 0..3 (used to
        read the pitch to hold and the elbow branch to stay on). If the target is out of reach, the closest
        achievable pose is returned. The returned angles still need clamping to the servo range by the caller
        (unchanged from before).
        """
        px, py, pz = target[0], target[1], target[2]
        q1c, q2c, q3c, q4c = self._servos_to_q(current_angles)

        # base yaw, and reduction of the 3D target to planar coordinates (radial wy, and wx along -reach)
        if abs(px) < 1e-9 and abs(py) < 1e-9:
            q1 = q1c          # gripper on the vertical axis: yaw is undefined, keep the current one
            wy = 0.0
        else:
            q1 = math.atan2(-px, py)
            wy = math.hypot(px, py)
            # The (q1, wy) split is two-to-one: spinning the base yaw by 180 deg and negating the radial
            # reach lands on the same gripper point. atan2/hypot always pick wy >= 0, which forces the base
            # to flip whenever the target sits behind the vertical axis (e.g. dragging the gripper from in
            # front of the base, across the centre, to behind it) -> the whole arm swings around. Instead
            # keep the base yaw continuous with the current pose and let wy carry the sign (the planar arm
            # leans to the far side). Without this the IK is not the inverse of the FK for wy < 0.
            if math.cos(q1 - q1c) < 0:
                q1 = q1 + math.pi if q1 < 0 else q1 - math.pi   # rotate by 180 deg, kept within [-pi, pi]
                wy = -wy
        wx = self.h - pz

        # the gripper pitch we hold: absolute in-plane angle of the final segment, from the current pose
        pitch = q2c + q3c + q4c + self.delta4

        # wrist centre = target minus the fixed-pitch final segment; the 2-link (a2, a3) must reach it
        wrist_x = wx + self.a4 * math.cos(pitch)
        wrist_y = wy + self.a4 * math.sin(pitch)
        dx, dy = -wrist_x, -wrist_y

        # 2-link planar inverse kinematics (law of cosines for the elbow, then the shoulder)
        cos_elbow = (dx * dx + dy * dy - self.a2 ** 2 - self.a3 ** 2) / (2 * self.a2 * self.a3)
        cos_elbow = max(-1.0, min(1.0, cos_elbow))      # clamp: out of reach -> closest achievable pose
        elbow = math.acos(cos_elbow)
        q3 = elbow if q3c >= 0 else -elbow              # keep the current elbow branch to avoid flips

        q2 = math.atan2(dy, dx) - math.atan2(self.a3 * math.sin(q3), self.a2 + self.a3 * math.cos(q3))
        q4 = pitch - q2 - q3 - self.delta4              # set the wrist to maintain the held pitch
        return self._q_to_servos([q1, q2, q3, q4])

    def resolved_rate(self, servo_angles, cartesian_velocity, dt,
                      base_damping: float = 0.01, max_damping: float = 0.08,
                      singularity_threshold: float = 0.02, limit_avoidance_gain: float = 0.6) -> list:
        """
        One step of resolved-rate (Jacobian) control for driving the gripper point with the joystick.

        Inputs: the current servo angles (degrees) for motors 0..3, the desired gripper-point velocity
        cartesian_velocity = (vx, vy, vz) in metres/second, and the timestep dt in seconds. Returns the next
        servo angles (degrees) for motors 0..3 (already within the servo range; the caller may still clamp).

        Unlike inverse_kinematics, which pins the gripper pitch and so jams a joint against its limit when a
        motion needs the arm to re-pose itself, this distributes the commanded velocity across all four joints
        via the damped-least-squares inverse of the Jacobian. A null-space term gently pulls the joints toward
        the centre of their range to keep clear of the limits without disturbing the gripper motion, and the
        damping is raised near singular (fully extended) configurations so the joint speeds stay bounded. This
        is what lets the arm flatten out / re-arrange to keep following the stick instead of getting stuck.
        """
        q = np.array([math.radians(servo_angles[i] - 90) for i in range(4)])
        jacobian = self._jacobian(q)                                   # 3x4: d(gripper point) / d(joints)
        jjt = jacobian @ jacobian.T                                    # 3x3
        # raise the damping as the arm nears a singularity (manipulability -> 0) so q_dot cannot blow up
        manipulability = math.sqrt(max(float(np.linalg.det(jjt)), 0.0))
        if manipulability >= singularity_threshold:
            damping = base_damping
        else:
            damping = base_damping + max_damping * (1.0 - manipulability / singularity_threshold)
        damped_inverse = jacobian.T @ np.linalg.inv(jjt + (damping ** 2) * np.eye(3))   # 4x3
        velocity = np.asarray(cartesian_velocity, dtype=float)
        # primary task: follow the commanded gripper-point velocity
        q_dot = damped_inverse @ velocity
        # secondary task in the Jacobian's null space: drift the joints toward the centre of their range
        # (q = 0, i.e. servo 90) to stay away from the limits, without perturbing the gripper-point motion
        null_space_projector = np.eye(4) - damped_inverse @ jacobian
        q_dot = q_dot + null_space_projector @ (-limit_avoidance_gain * q)
        q = np.clip(q + q_dot * dt, self._Q_MIN, self._Q_MAX)
        return [math.degrees(q[i]) + 90 for i in range(4)]

    def _jacobian(self, joint_angles, eps: float = 1e-6):
        # Numeric central-difference Jacobian (3x4) of the gripper point w.r.t. the four joint rotations.
        # The closed-form forward kinematics is exact and cheap, so finite differences are accurate and fast.
        jacobian = np.zeros((3, 4))
        for i in range(4):
            delta = np.zeros(4)
            delta[i] = eps
            p_plus = np.array(self._fk_from_joints(joint_angles + delta))
            p_minus = np.array(self._fk_from_joints(joint_angles - delta))
            jacobian[:, i] = (p_plus - p_minus) / (2 * eps)
        return jacobian
