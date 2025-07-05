import sys
import ikpy
from ikpy.chain import Chain
import math
from controller import Supervisor, Camera, GPS, CameraRecognitionObject
import numpy as np

# Constants
IKPY_MAX_ITERATIONS = 20  # Maximum iterations for inverse kinematics calculations
POSITION_TOLERANCE = 0.005  # Tolerance for position accuracy
GRIPPER_TOLERANCE = 0.02  # Tolerance for gripper position

# Global variables
supervisor = None
timeStep = None
armChain = None
motors = []
gripper_motors = []
camera = None
gps = None
HOME_POSITION = (0, 1, 1)  # Home position coordinates
HOME_ORIENTATION = np.eye(3)  # Identity matrix for home orientation
BIN_ORIENTATION = np.array([[1, 0, 0], [0, -1, 0], [0, 0, 1]])  # End effector facing negative Y axis

# Define bin positions
BIN_POSITIONS = [
    (2, 0.75, 0.4),  # Position for bin 1
    (2, 0, 0.4),     # Position for bin 2
    (2, -0.75, 0.4)  # Position for bin 3
]

# Initialize the motors of the robotic arm
def initialize_motors():
    global motors
    motors = []
    for link in armChain.links:
        if 'motor' in link.name:
            motor = supervisor.getDevice(link.name)
            motor.setVelocity(1.0)
            position_sensor = motor.getPositionSensor()
            position_sensor.enable(timeStep)
            motors.append(motor)

# Initialize the gripper motors
def initialize_gripper_motors():
    global gripper_motors
    gripper_motor_names = ['finger_1_joint_1', 'finger_2_joint_1', 'finger_middle_joint_1']
    gripper_motors = []
    for name in gripper_motor_names:
        gripper_motor = supervisor.getDevice(name)
        if gripper_motor:
            gripper_motor.setVelocity(0.5)  # Reduced velocity for smoother movement
            position_sensor = gripper_motor.getPositionSensor()
            position_sensor.enable(timeStep)
            gripper_motors.append(gripper_motor)

# Setup the camera device
def setup_camera():
    global camera
    camera = supervisor.getDevice("camera")
    camera.enable(timeStep)
    camera.recognitionEnable(timeStep)

# Setup the GPS device
def setup_gps():
    global gps
    gps = supervisor.getDevice("gps")
    gps.enable(timeStep)

# Get the rotation matrix from the camera
def get_rotation_matrix():
    rotation_matrix = supervisor.getFromDef("camera").getOrientation()
    return [
        [rotation_matrix[0], rotation_matrix[1], rotation_matrix[2]],
        [rotation_matrix[3], rotation_matrix[4], rotation_matrix[5]],
        [rotation_matrix[6], rotation_matrix[7], rotation_matrix[8]]
    ]

# Transform coordinates from camera frame to world frame
def transform_coordinates(recognized_object):
    object_position_camera_frame = recognized_object.getPosition()
    camera_position_world_frame = gps.getValues()
    rotation_matrix = get_rotation_matrix()
    
    object_position_world_frame = [
        camera_position_world_frame[0] + sum(rotation_matrix[0][i] * object_position_camera_frame[i] for i in range(3)),
        camera_position_world_frame[1] + sum(rotation_matrix[1][i] * object_position_camera_frame[i] for i in range(3)),
        camera_position_world_frame[2] + sum(rotation_matrix[2][i] * object_position_camera_frame[i] for i in range(3))
    ]
    
    return object_position_world_frame

# Determine which bin to place the object based on its color
def determine_bin_position(recognized_object):
    # Get the number of colors detected
    num_colors = recognized_object.getNumberOfColors()

    # Get the colors
    colors = recognized_object.getColors() 
    # If there are colors determine the dominant color
    if num_colors > 0:      
        if colors[0] > colors[1] and colors[2]:  
            return BIN_POSITIONS[0]  
        elif colors[1] > colors[0] and colors[2]: 
            return BIN_POSITIONS[1]  
        elif colors[2] > colors[1] and colors[0]: 
            return BIN_POSITIONS[2] 
        else:
            return BIN_POSITIONS[0]  # Default to bin 1 if no color is dominant

    # If no colors were detected, return a default bin 
    return BIN_POSITIONS[0]

# Move the robotic arm to a specified position and orientation
def move_to_position(x, y, z, target_orientation):
    global armChain, motors
    x_offset = 0  
    y_offset = 0.09  
    z_offset = 0 
    
    adjusted_x = x - x_offset
    adjusted_y = y - y_offset
    adjusted_z = z - z_offset
    
    current_motor_positions = [m.getPositionSensor().getValue() for m in motors]
    initial_position = [0] + current_motor_positions[:6] + [0] 
    
    try:
        ikResults = armChain.inverse_kinematics(
            [adjusted_x, adjusted_y, adjusted_z], 
            target_orientation=target_orientation, 
            orientation_mode='all', 
            max_iter=IKPY_MAX_ITERATIONS, 
            initial_position=initial_position
        )
    except Exception as e:
        print(f"IK calculation failed: {e}")
        return False

    for i, motor in enumerate(motors):
        motor.setPosition(ikResults[i + 1])
    
    return wait_for_position_reached(ikResults)

# Wait for the robotic arm to reach the target position
def wait_for_position_reached(target_positions):
    global motors
    max_wait_cycles = 200  
    cycles = 0
    
    while cycles < max_wait_cycles:
        supervisor.step(timeStep)
        current_positions = [m.getPositionSensor().getValue() for m in motors]
        
        if all(abs(current_positions[i] - target_positions[i + 1]) < POSITION_TOLERANCE for i in range(len(motors))):
            return True
        
        cycles += 1
    
    print("Failed to reach target position within timeout")
    return False

# Open or close the gripper to grasp or release an object
def grasp_object(grasp=True):
    global gripper_motors
    target_position = 0.9 if grasp else 0.0495
    
    for motor in gripper_motors:
        motor.setPosition(target_position)
    
    max_wait_cycles = 200
    cycles = 0
    
    while cycles < max_wait_cycles:
        supervisor.step(timeStep)
        current_positions = [motor.getPositionSensor().getValue() for motor in gripper_motors]
        
        if all(abs(current_position - target_position) < GRIPPER_TOLERANCE for current_position in current_positions):
            return True
        
        cycles += 1
    
    print("Failed to reach gripper target position within timeout")
    return False

# Main function to perform the pick-and-place operation
def pick_and_place(recognized_object):
    # Transform detected object's coordinates to world frame
    object_position = transform_coordinates(recognized_object)
    # Make first movement to line up grasper with object
    move_to_position(object_position[0], object_position[1] - 0.1, object_position[2], HOME_ORIENTATION)
    # Shift forward to put object in optimal position for grasping
    move_to_position(object_position[0], object_position[1], object_position[2], HOME_ORIENTATION)
    # Grasp the object
    grasp_object(True)
    # Lift object to a safe height
    move_to_position(object_position[0], object_position[1], object_position[2] + 0.5, HOME_ORIENTATION)
    # Determine bin position based on color and move object there
    bin_position = determine_bin_position(recognized_object)
    # Move object to correct bin
    move_to_position(*bin_position, BIN_ORIENTATION)
    # Drop object
    grasp_object(False)
    # Move back to home position
    move_to_position(*HOME_POSITION, HOME_ORIENTATION)
    
    return True

# Main function to initialize and run the simulation
def main():
    global supervisor, timeStep, armChain
    supervisor = Supervisor()
    timeStep = int(4 * supervisor.getBasicTimeStep())
    
    filename = r'C:\Users\will\Documents\Irb4600-40.urdf'
    # Define kinematic chain from URDF
    armChain = Chain.from_urdf_file(filename, active_links_mask=[False, True, True, True, True, True, True, False])
    
    initialize_motors()
    initialize_gripper_motors()
    setup_camera()
    setup_gps()
    
    move_to_position(*HOME_POSITION, HOME_ORIENTATION)
    
    while supervisor.step(timeStep) != -1: 
        # Move the arm to the home position where the objects can be seen
        move_to_position(*HOME_POSITION, HOME_ORIENTATION)
        # Add all objects currently detected by the recognition to a list
        objects = camera.getRecognitionObjects()
        
        for obj in objects:
            if pick_and_place(obj):
                break  # Only handle one object at a time
        
    
    supervisor.simulationSetMode(Supervisor.SIMULATION_MODE_PAUSE)

if __name__ == "__main__":
    main()
