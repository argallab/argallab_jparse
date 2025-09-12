from xarm.wrapper import XArmAPI
import time
import pandas as pd
import ast
import matplotlib.pyplot as plt

# Connect to the xArm controller
arm = XArmAPI('192.168.1.199')  # Replace with your robot's IP address
time.sleep(0.5)

# Initialize the robot arm and set to position mode
# arm.set_mode(0)  # Start in position mode
# arm.set_state(0)

from ast import literal_eval

def safe_literal_eval(x):
    try:
        return literal_eval(x)
    except (ValueError, SyntaxError):
        return None

recording = False
replaying = True
trajectory = []

try:
    while True:
        # Get the status of the controller CO pins
        _, controller_co = arm.get_cgpio_digital()
        if not recording:
            # Start recording and switch to manual mode
            recording = True
            arm.set_mode(0)
            arm.set_state(state=0)
            arm.set_gripper_enable(True)
            # arm.set_gripper
            # print("Recording started. Switched to manual mode.")
            time.sleep(1)  # Debounce delay
        # elif recording:
        #         # Stop recording and switch back to position mode
        #         recording = False
        #         arm.set_mode(0)  # Switch back to position mode
        #         arm.set_state(0)
        #         print("Recording stopped. Switched to position mode.")
        #         time.sleep(0.5)  # Debounce delay

        # Button B (CO1): Record the current position (Low signal triggers)
        # TODO: make this it's own separate file 
        elif recording:
            code, position = arm.get_position()
            if code == 0:
                trajectory.append(position)
                print(f"Position recorded: {position}")

                # save trajectory
                # from IPython import embed; embed()
                df = pd.DataFrame( columns=['x', 'y', 'z', 'rx', 'ry', 'rz'])
                df2 = pd.DataFrame(trajectory, columns=['x', 'y', 'z', 'rx', 'ry', 'rz'])
                df_final = pd.concat([df, df2])
                # time.sleep(5)
            df_final.to_csv('output.csv')
            time.sleep(0.5)

        # Button C (CO2): Play the recorded trajectory (Low signal triggers)
        # TODO: this will loop because of the while loop; need to integrate with ROS instead
        # currently this will playback cartesian but if we want to playback caring about the joint states, can listen to the joint states topic
        #  and use arm.vc_set_joint_velocity instead!! and that means the mode needs to be set to 4 and not 2 like for the cartesian
        if replaying:
            arm.set_mode(0)
            arm.set_state(0)
            time.sleep(1)
            # time.sleep(2)
            filepath = '/home/jparse_ws/src/replay_test/reconstructed_data/old_demo/demiana_redo/JOY/pickandplace/old_demo_1_20250911-004136.csv'
            df = pd.read_csv(filepath)
            # traj = pd.read_csv(filepath)
            traj = df[['ee_x', 'ee_y', 'ee_z', 'ee_tx', 'ee_ty', 'ee_tz']]
            traj_relative = traj[::20].diff().dropna()


            df_as_list = traj_relative.values.tolist()
            df_as_list.append(traj.iloc[-1] - traj[::20].iloc[-1])

            gripper_pose = df[::20]['gripper']
            # from IPython import embed; embed()
            print("Playing back trajectory...")
            for pos, grip in zip(df_as_list, gripper_pose):
                if grip != 0.0:
                    arm.set_gripper_position(grip)
                # for grip in gripper_pose:
                print(pos)
                arm.set_position(*pos, wait=False, relative=True, radius=0)
                
            print("Trajectory playback complete.")
            time.sleep(0.5)
            replaying = False

except KeyboardInterrupt:
    print("Program interrupted")

finally:
    # Disconnect from the robot arm
    arm.disconnect()