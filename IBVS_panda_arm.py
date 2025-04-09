import numpy as np
import pybullet as p
import pybullet_data
import time
import math
import cv2
import csv

# Connect to PyBullet GUI
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -10)
p.setTimeStep(0.01)
plane_id = p.loadURDF("plane.urdf")

# Load Franka Panda
panda_model = "franka_panda/panda.urdf"
franka_id = p.loadURDF(panda_model, useFixedBase=True, flags=p.URDF_USE_SELF_COLLISION)

# Joint velocity limits
joint_velocity_limits = [150, 150, 150, 150, 180, 180, 180]
joint_velocity_limits = [v * (math.pi / 180) for v in joint_velocity_limits]

for i in range(len(joint_velocity_limits)):
    p.changeDynamics(bodyUniqueId=franka_id, linkIndex=i, maxJointVelocity=joint_velocity_limits[i])

# Camera intrinsics
fov, aspect, nearplane, farplane = 60, 1.0, 0.01, 100
projection_matrix = p.computeProjectionMatrixFOV(fov, aspect, nearplane, farplane)

# Add red ball
radius = 0.05
visual_shape_id = p.createVisualShape(p.GEOM_SPHERE, radius=radius, rgbaColor=[1, 0, 0, 1])
collision_shape_id = p.createCollisionShape(p.GEOM_SPHERE, radius=radius)
ball_id = p.createMultiBody(baseMass=0,
                            baseCollisionShapeIndex=collision_shape_id,
                            baseVisualShapeIndex=visual_shape_id,
                            basePosition=[0.5, 0, 0])

# Ball sliders
ball_sliders = {
    'x': p.addUserDebugParameter('Ball X', -2, 2, 1.5),  #initial position of 1.5 along x-axis
    'y': p.addUserDebugParameter('Ball Y', -2, 2, 0.0),
    'z': p.addUserDebugParameter('Ball Z', 0, 1, 0)
}
def update_ball_position():
    pos = [p.readUserDebugParameter(ball_sliders[axis]) for axis in ['x', 'y', 'z']]
    p.resetBasePositionAndOrientation(ball_id, pos, [0, 0, 0, 1])

# Joint sliders
joint_sliders = {}
joint_limits = {
    0: (-2.897, 2.897),
    1: (-1.762, 1.762),
    2: (-2.897, 2.897),
    3: (-2.017, 2.897),
    4: (-2.897, 2.897),
    5: (-0.017, 2.897),
    6: (-2.897, 2.897),
}
for i in range(len(joint_limits)):
    joint_sliders[i] = p.addUserDebugParameter(f'Joint {i+1}', joint_limits[i][0], joint_limits[i][1], 0)
def update_joints():
    for i in range(len(joint_limits)):
        joint_position = p.readUserDebugParameter(joint_sliders[i])
        p.setJointMotorControl2(franka_id, i, p.POSITION_CONTROL, targetPosition=joint_position)

# Get camera view matrix from end-effector 
def panda_camera():
    com_p, com_o, _, _, _, _ = p.getLinkState(franka_id, 11, computeForwardKinematics=True)
    rot_matrix = p.getMatrixFromQuaternion(com_o)
    rot_matrix = np.array(rot_matrix).reshape(3, 3)
    camera_vector = rot_matrix.dot((0, 0, 1))
    up_vector = rot_matrix.dot((0, 1, 0))
    view_matrix = p.computeViewMatrix(com_p, com_p + 0.1 * camera_vector, up_vector)
    img = p.getCameraImage(640, 480, view_matrix, projection_matrix)
    return img

# Detect red ball in image
def detect_red_ball(rgb_img):
    rgb = np.reshape(rgb_img[2], (480, 640, 4))[:, :, :3]
    bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)

    lower_red1 = np.array([0, 70, 50])
    upper_red1 = np.array([10, 255, 255])
    lower_red2 = np.array([170, 70, 50])
    upper_red2 = np.array([180, 255, 255])

    mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
    mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
    mask = cv2.bitwise_or(mask1, mask2)

    contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    target_pixel = None
    if contours:
        c = max(contours, key=cv2.contourArea)
        M = cv2.moments(c)
        if M['m00'] > 0:
            cx = int(M['m10'] / M['m00'])
            cy = int(M['m01'] / M['m00'])
            target_pixel = (cx, cy)
            cv2.circle(bgr, (cx, cy), 5, (0, 255, 0), -1)

    return target_pixel, bgr, mask

def get_image_jacobian(u, v, z=1.0, fx=600, fy=600):
    """
    Returns the image Jacobian L for a point (u, v)
    z is the estimated depth
    fx, fy are focal lengths (pixels)
    """
    # Here I multiplies the Jacobian by the focal length to keep everything in pixel units, 
    #which is often done in implementation for consistency with measurements in pixels. and
    # to keep everything numerically stabile
    L = np.array([
        [-fx / z, 0, u / z, u * v / fx, -(fx ** 2 + u ** 2) / fx, v],
        [0, -fy / z, v / z, (fy ** 2 + v ** 2) / fy, -u * v / fy, -u]
    ])
    return L

#dof = p.getNumJoints(franka_id) - 1
#joints = range(dof)

# IBVS control
def ibvs_control(target_pixel, image_size=(640, 480)):
    global error_history, v_cam_history

    if target_pixel is None:
        return

    cx, cy = image_size[0] // 2, image_size[1] // 2
    tx, ty = target_pixel
    error = np.array([cx-tx, cy-ty], dtype=np.float32)
    
    # P-controller gain
    Kp =10

    # Full visual velocity (2D error → 6D velocity)
    #z = 0.25  # estimated constant depth to the target
    
    #dynamic depth estimation
    # Get ball position in world (a method that workks only in simulation using some built-in functions in pybullet)
    ball_pos, _ = p.getBasePositionAndOrientation(ball_id)   
    # Get camera pose (EE pose)
    cam_pos, cam_ori = p.getLinkState(franka_id, 11, computeForwardKinematics=True)[:2]
    R_cam = np.array(p.getMatrixFromQuaternion(cam_ori)).reshape(3, 3)
    T_wc = np.eye(4)
    T_wc[:3, :3] = R_cam
    T_wc[:3, 3] = cam_pos  
    # Inverse of camera pose
    T_cw = np.linalg.inv(T_wc)   
    # Homogeneous ball position in world
    ball_h = np.array([*ball_pos, 1.0])
    ball_in_cam = T_cw @ ball_h   
    z = ball_in_cam[2]  # Depth in camera frame

    
    fx, fy = 600, 600
    L = get_image_jacobian(tx - cx, ty - cy, z, fx, fy)
    #L=get_image_jacobian(cx-tx, cy-ty , z, fx, fy)
    try:
        L_inv = np.linalg.pinv(L)  # 6x2 pseudo-inverse
    except np.linalg.LinAlgError:
        return
    
    v_cam_6d = - Kp * L_inv @ error  # camera-frame 6D velocity [vx, vy, vz, wx, wy, wz]
    
    # Store for plotting
    error_history.append(np.linalg.norm(error))
    #error_history.append(error)
    v_cam_history.append(v_cam_6d.tolist())
    
    #print("Visual error:", error)
    #print("v_cam_6d:", v_cam_6d)

    # Get transformation from camera to end-effector (rotation only as the camera is directly attached to the end effector in this simulation) 
    _, cam_quat, _, _, _, _ = p.getLinkState(franka_id, 11, computeForwardKinematics=True)
    R_cam2ee = np.array(p.getMatrixFromQuaternion(cam_quat)).reshape(3, 3)
    R6 = np.block([
        [R_cam2ee, np.zeros((3, 3))],
        [np.zeros((3, 3)), R_cam2ee]
    ])
    v_ee = R6 @ v_cam_6d  # transform to end-effector frame

    # Get the correct 7 joint indices
    controlled_joints = [i for i in range(p.getNumJoints(franka_id)) if p.getJointInfo(franka_id, i)[2] != p.JOINT_FIXED]
    
    # Get joint positions
    joint_states = p.getJointStates(franka_id, controlled_joints)
    joint_positions = [s[0] for s in joint_states]
    zero_vec = [0.0] * len(joint_positions)
    
    # Local position of the point on the end effector (usually [0,0,0] at the link origin)
    local_position = [0, 0, 0]
    
    # Compute Jacobian
    J_lin, J_ang = p.calculateJacobian(franka_id, 11, local_position, joint_positions, zero_vec, zero_vec)
    J = np.vstack((np.array(J_lin), np.array(J_ang)))  # 6x7

    try:
        J_pinv = np.linalg.pinv(J)  # 7x6
        q_dot = J_pinv @ v_ee       # joint velocities
    except np.linalg.LinAlgError:
        return
    
    
    #print("Jacobian J (6x7):\n", J)
    #print("Pseudo-inverse of Jacobian (J_pinv):\n", J_pinv)
    #print("L_inv (image Jacobian inverse):\n", L_inv)
    #print("q_dot:", q_dot)
    
    # Send joint velocity commands
    controlled_joints = [i for i in range(p.getNumJoints(franka_id)) if p.getJointInfo(franka_id, i)[2] != p.JOINT_FIXED]
    p.setJointMotorControlArray(
    bodyUniqueId=franka_id,
    jointIndices=controlled_joints,
    controlMode=p.VELOCITY_CONTROL,
    targetVelocities=q_dot.tolist()
)



# Home position (any reasonable start position based on the task...)
home_position = [0, -0.5, 0, -2, 0, 1.5, 0]  # Set to a neutral/home configuration

# Move the robot to the home position
for i in range(len(home_position)):
    p.setJointMotorControl2(franka_id, i, p.POSITION_CONTROL, targetPosition=home_position[i])

# Simulate for a few steps to let the robot settle into the home position before running the main simulation loop
for _ in range(100):
    p.stepSimulation()
    time.sleep(0.01)

# For diagnostics
error_history = []
v_cam_history = []
# Initialize the file to save data
data_file = "simulation_data.csv"
with open(data_file, mode='w', newline='') as f:
    writer = csv.writer(f)
    writer.writerow(['Time Step', 'Error', 'v_x', 'v_y', 'v_z', 'w_x', 'w_y', 'w_z'])  # Header

# Main loop
i = 0
while True:
    p.stepSimulation()
    #update_joints()
    update_ball_position()
    img = panda_camera()
    ball_pixel, bgr_img, mask = detect_red_ball(img)
    #print(ball_pixel)
    ibvs_control(ball_pixel)
    # Display
    cv2.imshow("Camera View", bgr_img)
    cv2.imshow("Red Mask", mask)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
    
    if len(error_history) > 0 and len(v_cam_history) > 0:
        with open(data_file, mode='a', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([i, error_history[-1]] + list(v_cam_history[-1]))


    
    i += 1
    time.sleep(0.01)

cv2.destroyAllWindows()
p.disconnect()