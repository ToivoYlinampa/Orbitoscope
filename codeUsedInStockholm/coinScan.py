import csv
import serial
import logging
import time
import subprocess
import os
import cv2
import numpy as np
import datetime

# Global session counter
session_count = 0

# Setup basic logging
logging.basicConfig(level=logging.INFO)

# Define command keys for setting and getting positions
SET_COMMANDS = {'X': 'W', 'Y': 'E', 'Z': 'R', 'A': 'T', 'B': 'O', 'C': 'P'}
MOVE_COMMANDS = {'X': 'x', 'Y': 'y', 'Z': 'z', 'A': 'a', 'B': 'b', 'C': 'c'}
GET_COMMANDS = {'X': 'w', 'Y': 'e', 'Z': 'r', 'A': 't', 'B': 'o', 'C': 'p'}

def initialize_arduino():
    try:
        ser = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)
        ser.flushInput()
        print("Connected to Arduino.")
        time.sleep(1)  # Initial delay for Arduino stabilization
        logging.info("Connected to Arduino.")
        return ser
    except serial.SerialException as e:
        print(f"Error connecting to Arduino: {e}")
        logging.error(f"Error connecting to Arduino: {e}")
        return None

def send_command_to_arduino(ser, command):
    if ser:
        try:
            ser.write(command.encode())
            print(f"Sent '{command}' to Arduino.")
            time.sleep(0.2)  # Short delay before listening for a response
        except serial.SerialException as e:
            print(f"Failed to send command: {e}")
            logging.error(f"Failed to send command: {e}")

def read_response_from_arduino(ser):
    time.sleep(0.2)
    if ser:
        response = ser.readline().decode().strip()
        if response:
            print(f"Arduino says: {response}")
        return response

def wait_for_all_axes_movement_done(ser, axes_to_move):
    """ 
    Waits for a 'movement done' message from the Arduino for all axes that were commanded to move.
    :param ser: The serial connection to the Arduino.
    :param axes_to_move: A list of axes that are expected to report movement completion.
    """
    axes_remaining = set(axes_to_move)  # Keep track of axes that haven't confirmed movement completion
    
    while axes_remaining:
        if ser.in_waiting > 0:
            response = ser.readline().decode().strip()
            print(f"Arduino says: {response}")
            
            # Check if the response indicates an axis has finished moving
            for axis in axes_to_move:
                if f"{axis} movement done" in response:
                    axes_remaining.discard(axis)  # Remove axis from waiting list when movement is done
                    print(f"Axis {axis} has finished moving.")

    print("All axes have finished moving.")


def load_csv_data(filepath):
    try:
        with open(filepath, newline='') as csvfile:
            reader = csv.DictReader(csvfile)
            return list(reader)
    except Exception as e:
        print(f"Failed to load data from {filepath}: {e}")
    return []

def set_initial_positions(ser, positions):
    current_positions = {}
    for position in positions:
        if position['type'] == 'position':
            axis = position['axis']
            current_position = fetch_current_position(ser, axis)  # Fetch current position before setting new one
            if current_position != position['start_position']:  # Only send if different
                command = SET_COMMANDS[axis] + position['start_position']
                send_command_to_arduino(ser, command)
                read_response_from_arduino(ser)
            current_positions[axis] = position['start_position']  # Store current position for later use
    return current_positions

def fetch_current_position(ser, axis):
    time.sleep(0.5)  # Add delay before fetching the position
    command = GET_COMMANDS[axis]
    send_command_to_arduino(ser, command)
    response = read_response_from_arduino(ser)
    return response.split()[-1]  # Assuming the position is the last item in response

def calculate_and_apply_delay(ser, axis, target_position, speed):
    """
    Calculate the time to move to a new position based on the speed and apply a delay.

    :param ser: Serial connection to Arduino.
    :param axis: The axis for which the movement is intended ('X', 'Y', 'Z', etc.).
    :param target_position: The target position for the axis.
    :param speed: Speed of the axis in steps per second.
    """
    current_position = fetch_current_position(ser, axis)
    if current_position is None:
        print(f"Failed to fetch current position for axis {axis}.")
        return

    try:
        # Convert positions to integers for calculation
        current_position = int(current_position)
        target_position = int(target_position)
    except ValueError:
        print(f"Invalid positions received for axis {axis}. Ensure they are numerical.")
        return

    # Only send the command if the target position is different from the current position
    if current_position != target_position:
        # Calculate the number of steps needed
        steps_needed = abs(target_position - current_position)
        
        # Calculate the time in seconds it will take to complete the movement
        if speed > 0:
            time_to_move = steps_needed / speed
        else:
            print(f"Speed is zero or undefined for axis {axis}, setting minimum delay to avoid division by zero.")
            time_to_move = 1  # Default to 1 second if speed is zero or not provided

        # Apply the calculated time delay after movement starts
        move_command = f"{MOVE_COMMANDS[axis]}{target_position} {speed}"
        send_command_to_arduino(ser, move_command)
        wait_for_movement_done(ser)

def execute_sequence(ser, sequence_data, current_positions, session_folder):
    """
    Execute a movement and action sequence from loaded CSV data.
    :param ser: The serial connection to the Arduino.
    :param sequence_data: A list of dictionaries, each representing a sequence step.
    :param current_positions: A dictionary with the current positions of each axis.
    :param session_folder: The folder where session-specific files will be stored.
    """
    for step_index, movement in enumerate(sequence_data):
        print(f"Executing step {step_index + 1}/{len(sequence_data)}...")

        # Iterate over each axis to check if a movement command is needed
        for axis in 'XYZABC':  # Assuming control of up to six axes (X, Y, Z, A, B, C)
            pos_key = f"{axis}_position"
            speed_key = f"{axis}_speed"
            
            if pos_key in movement and movement[pos_key] != current_positions[axis]:
                target_position = int(movement[pos_key])
                speed = int(movement.get(speed_key, 100))  # Default speed if not specified

                # Calculate and apply the delay dynamically based on the required movement
                calculate_and_apply_delay(ser, axis, target_position, speed)

                # Update the current position in the dictionary after movement completion
                current_positions[axis] = movement[pos_key]

                print(f"Moved axis {axis} to position {movement[pos_key]} at speed {speed}.")

                if axis == 'A':
                    print("Adding two-second delay after A-axis movement.")
                    time.sleep(2)

        # Check for image capture commands
        if 'image' in movement and movement['image'] != '0':
            image_code = movement['image']
            handle_image_capture(image_code, ser, session_folder)

        # After each image capture series, move to the C position and wait for it to complete
        if 'C_position' in movement:
            target_c_position = int(movement['C_position'])
            c_speed = int(movement.get('C_speed', 100))  # Default speed if not specified

            # Move to the C position
            print(f"Moving to the C position: {target_c_position} at speed {c_speed}")
            calculate_and_apply_delay(ser, 'C', target_c_position, c_speed)

            # Update the current C position after movement completion
            current_positions['C'] = movement['C_position']

        print(f"Step {step_index + 1} completed.")

    print("Sequence execution completed.")


def execute_sequence_reverse(ser, sequence_data, current_positions, session_folder):
    """
    Execute a movement and action sequence from loaded CSV data in reverse order.
    :param ser: The serial connection to the Arduino.
    :param sequence_data: A list of dictionaries, each representing a sequence step.
    :param current_positions: A dictionary with the current positions of each axis.
    :param session_folder: The folder where session-specific files will be stored.
    """
    for step_index, movement in enumerate(reversed(sequence_data)):
        print(f"Executing step {len(sequence_data) - step_index}/{len(sequence_data)}...")

        # Iterate over each axis to check if a movement command is needed
        for axis in 'XYZABC':  # Assuming control of up to six axes (X, Y, Z, A, B, C)
            pos_key = f"{axis}_position"
            speed_key = f"{axis}_speed"
            
            if pos_key in movement and movement[pos_key] != current_positions[axis]:
                target_position = int(movement[pos_key])
                speed = int(movement.get(speed_key, 100))  # Default speed if not specified

                # Calculate and apply the delay dynamically based on the required movement
                calculate_and_apply_delay(ser, axis, target_position, speed)

                # Update the current position in the dictionary after movement completion
                current_positions[axis] = movement[pos_key]

                print(f"Moved axis {axis} to position {movement[pos_key]} at speed {speed}.")

                if axis == 'A':
                    print("Adding two-second delay after A-axis movement.")
                    time.sleep(2)

        # Check for image capture commands
        if 'image' in movement and movement['image'] != '0':
            image_code = movement['image']
            handle_image_capture(image_code, ser, session_folder)

        # After each image capture series, move to the C position and wait for it to complete
        if 'C_position' in movement:
            target_c_position = int(movement['C_position'])
            c_speed = int(movement.get('C_speed', 100))  # Default speed if not specified

            # Move to the C position
            print(f"Moving to the C position: {target_c_position} at speed {c_speed}")
            calculate_and_apply_delay(ser, 'C', target_c_position, c_speed)

            # Update the current C position after movement completion
            current_positions['C'] = movement['C_position']

        print(f"Step {len(sequence_data) - step_index} completed.")

    print("Reverse sequence execution completed.")


def handle_image_capture(image_code, ser, session_folder):
    """
    Handle image capture based on the code in the sequence data.
    :param image_code: The code indicating the type of image capture required.
    :param ser: The serial connection to the Arduino.
    :param session_folder: Directory where images are saved.
    """
    global session_count
    session_count += 1
    cstack_folder = os.path.join(session_folder, f"CStack_{session_count}")
    os.makedirs(cstack_folder, exist_ok=True)
    
    if image_code == '1':
        base_filename = f"image_1_{datetime.datetime.now().strftime('%Y%m%d_%H%M%S')}"
        image_folder = os.path.join(cstack_folder, 'images')  # Specific folder for images
        ensure_folder_exists(image_folder)  # Make sure the folder exists
        jpeg_filename, raw_filename = capture_image(base_filename, image_folder, ser)
        
        if jpeg_filename and raw_filename:
            print(f"Captured and saved JPEG: {jpeg_filename}")
            print(f"Captured and saved RAW: {raw_filename}")
        else:
            print("Failed to capture images.")
    elif image_code == '2':
        print("Launching CStack process...")
        CStack(ser, cstack_folder)

import os
import time
import subprocess

def capture_image(base_filename, jpeg_folder, ser):
    jpeg_filename = os.path.join(jpeg_folder, f"{base_filename}.jpg")
    max_attempts = 1
    attempt = 0

    while attempt < max_attempts:
        print(f"Attempting to capture image: {base_filename}")
        
        # Step 1: Set camera configuration
        config_result = subprocess.run([
            '/usr/bin/gphoto2',
            '--set-config', 'iso=4000',
            '--set-config', 'shutterspeed=1/60',
            '--set-config', 'aperture=22',
            # You can remove this line if using manual focus:
            # '--set-config', 'autofocusdrive=1'
        ], text=True, capture_output=True)
        print(f"Config stdout: {config_result.stdout}")
        print(f"Config stderr: {config_result.stderr}")

        # Give camera a moment to apply settings before capture
        time.sleep(1)

        # Step 2: Capture image
        time.sleep(1)  # <-- Added this delay before capture
        capture_result = subprocess.run([
            '/usr/bin/gphoto2',
            '--wait-event=6s',
            '--capture-image-and-download',
            '--filename', jpeg_filename,
            '--force-overwrite',
            '--no-keep'
        ], text=True, capture_output=True)

        print(f"Capture stdout: {capture_result.stdout}")
        print(f"Capture stderr: {capture_result.stderr}")

        if os.path.exists(jpeg_filename):
            print("JPEG image captured successfully.")
            return jpeg_filename, None
        else:
            print(f"Error: Failed to capture image. Attempt {attempt + 1} of {max_attempts}. Restarting camera...")
            restart_camera(ser)
            kill_camera_processes()
            attempt += 1

    print("Error: Failed to capture JPEG image after multiple attempts. Please check the camera.")
    return jpeg_filename, None




def restart_camera(ser):
    """Restart the camera by turning it off, waiting, and turning it back on."""
    send_command_to_arduino(ser, 'f')  # Turn off camera
    time.sleep(3)  # Wait for the camera to power down
    send_command_to_arduino(ser, 'F')  # Turn on camera
    time.sleep(2)  # Wait for the camera to stabilize
    kill_camera_processes()  # Free camera for `gphoto2`
    time.sleep(1)
    print("Camera has been restarted.")


def kill_camera_processes():
    result = subprocess.run(['ps', 'aux'], capture_output=True, text=True)
    processes = [line for line in result.stdout.splitlines() if 'gphoto2' in line]
    for proc in processes:
        pid = proc.split()[1]
        subprocess.run(['sudo', 'kill', '-9', pid])
        print("Killed process with PID:", pid)

def canny_value(image_path):
    try:
        img = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)
        if img is not None:
            edges = cv2.Canny(img, 100, 200)
            return np.mean(edges)
        else:
            raise ValueError(f"Failed to load image from {image_path}")
    except Exception as e:
        print(f"Error reading image {image_path}: {str(e)}")
        return 0.0

def ensure_folder_exists(folder_path):
    """Ensure the folder exists, create if it does not."""
    os.makedirs(folder_path, exist_ok=True)
    print(f"Folder '{folder_path}' is ready.")

def initialize_session_folder():
    """Initializes a new session folder for the current session."""
    global session_count
    session_count += 1
    timestamp = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    session_folder = os.path.join(os.getcwd(), f"session{session_count}_{timestamp}")
    ensure_folder_exists(session_folder)
    return session_folder

def wait_for_file(file_path, timeout=20):
    old_size = -1
    start_time = time.time()
    while time.time() - start_time < timeout:
        if not os.path.exists(file_path):
            print(f"Waiting for file {file_path} to exist...")
            time.sleep(0.5)
            continue

        try:
            new_size = os.path.getsize(file_path)
            if new_size == old_size and new_size != 0:
                print(f"File {file_path} is ready.")
                return True
            else:
                old_size = new_size
                time.sleep(0.5)
        except OSError as e:
            print(f"Error accessing file {file_path}: {e}")
            time.sleep(0.5)
    print(f"Final check for file existence: {os.path.exists(file_path)}")
    raise RuntimeError(f"Timeout waiting for file {file_path} to be ready.")

def zseries(ser, session_folder):
    if not ser or not ser.is_open:
        print("Arduino not connected.")
        return

    # Capture first CStack image at the starting Z-height
    starting_folder = os.path.join(session_folder, "CStack_Starting_Height")
    os.makedirs(starting_folder, exist_ok=True)
    initial_c_position = CStack(ser, starting_folder)  # Capture initial C position after CStack

    # Move C axis back to the original position using relative movement
    send_command_to_arduino(ser, f"C{-initial_c_position} 300")
    wait_for_movement_done(ser)

    # Move Z-axis 300 steps forward and create a subfolder
    forward_folder = os.path.join(session_folder, "CStack_Forward_300")
    os.makedirs(forward_folder, exist_ok=True)
    send_command_to_arduino(ser, "Z300 300")
    wait_for_movement_done(ser)

    # Capture second CStack image after moving forward
    forward_c_position = CStack(ser, forward_folder)

    # Move C axis back to the original position using relative movement
    send_command_to_arduino(ser, f"C{-forward_c_position} 300")
    wait_for_movement_done(ser)

    # Wait for 1 second
    time.sleep(1)

    # Move Z-axis 600 steps backward and create a subfolder
    backward_folder = os.path.join(session_folder, "CStack_Backward_600")
    os.makedirs(backward_folder, exist_ok=True)
    send_command_to_arduino(ser, "Z-600 300")
    wait_for_movement_done(ser)

    # Capture third CStack image after moving backward
    backward_c_position = CStack(ser, backward_folder)

    # Move C axis back to the original position using relative movement
    send_command_to_arduino(ser, f"C{-backward_c_position} 300")
    wait_for_movement_done(ser)

    # Move Z-axis back to the starting position (300 steps forward)
    send_command_to_arduino(ser, "Z300 300")
    wait_for_movement_done(ser)

    print("Z-series sequence completed.")













def CStack(ser, session_folder):
    if not ser or not ser.is_open:
        print("Arduino not connected.")
        return 0

    high_threshold = 0.002
    low_threshold_exit = 0.0005
    big_steps_back = 300
    speed_back = 100
    small_steps_back = 150
    small_steps_forward = 150
    speed_forward = 50
    start_position = 2135

    raw_folder = os.path.join(session_folder, "CStack_RAW")
    os.makedirs(raw_folder, exist_ok=True)
    print(f"Images will be saved in: {session_folder}")

    base_filename = "initial_image"
    jpeg_filename, _ = capture_image(base_filename, raw_folder, ser)  # Unpack the tuple
    if jpeg_filename is None or not wait_for_file(jpeg_filename):
        print("Error: Initial JPEG file was not ready or failed to capture in time.")
        return 0

    initial_canny = canny_value(jpeg_filename)
    print(f"Initial Canny Value: {initial_canny:.4f}")

    c_position = start_position

    if initial_canny >= high_threshold:
        print("Initial image is in focus. Moving backwards to find the start of the sharp area.")
        
        for i in range(38):
            send_command_to_arduino(ser, f"C-{big_steps_back} {speed_back}")
            wait_for_all_axes_movement_done(ser, ["C"])
            time.sleep(1)
            c_position -= big_steps_back

            new_base_filename = f"backward_image_{i+1}"
            jpeg_filename, _ = capture_image(new_base_filename, raw_folder, ser)  # Unpack the tuple
            if jpeg_filename is None or not wait_for_file(jpeg_filename):
                print(f"Error: Backward image {i+1} file was not ready in time.")
                continue

            backward_canny = canny_value(jpeg_filename)
            print(f"Backward Image {i+1} Canny Value: {backward_canny:.4f}")
            if backward_canny < low_threshold_exit:
                print("Exited sharp area, preparing to move forward.")
                break
        
        out_of_focus_count = 0
        while out_of_focus_count < 12:
            send_command_to_arduino(ser, f"C{small_steps_forward} {speed_forward}")
            wait_for_all_axes_movement_done(ser, ["C"])
            time.sleep(1)
            c_position += small_steps_forward

            new_base_filename = f"forward_image_{i+1}"
            jpeg_filename, _ = capture_image(new_base_filename, raw_folder, ser)  # Unpack the tuple
            if jpeg_filename is None or not wait_for_file(jpeg_filename):
                print(f"Error: Forward image {i+1} file was not ready in time.")
                continue

            forward_canny = canny_value(jpeg_filename)
            print(f"Forward Image {i+1} Canny Value: {forward_canny:.4f}")
            if forward_canny < low_threshold_exit:
                out_of_focus_count += 1
            else:
                out_of_focus_count = 0

            i += 1

    else:
        print("Initial image not in focus. Trying 1 5 big steps backward.")
        sharp_area_found = False
        for i in range(25):
            send_command_to_arduino(ser, f"C-{big_steps_back} {speed_back}")
            wait_for_all_axes_movement_done(ser, ["C"])
            time.sleep(1)
            c_position -= big_steps_back

            new_base_filename = f"backward_image_{i+1}"
            jpeg_filename, _ = capture_image(new_base_filename, raw_folder, ser)  # Unpack the tuple
            if jpeg_filename is None or not wait_for_file(jpeg_filename):
                print(f"Error: Backward image {i+1} file was not ready in time.")
                continue

            backward_canny = canny_value(jpeg_filename)
            print(f"Backward Image {i+1} Canny Value: {backward_canny:.4f}")
            if backward_canny >= high_threshold:
                print("Sharp area found going backwards.")
                sharp_area_found = True
                out_of_focus_count = 0
                while out_of_focus_count < 10:
                    send_command_to_arduino(ser, f"C-{small_steps_back} {speed_back}")
                    wait_for_all_axes_movement_done(ser, ["C"])
                    time.sleep(1)
                    c_position -= small_steps_back

                    new_base_filename = f"backward_image_detailed_{i+1}"
                    jpeg_filename, _ = capture_image(new_base_filename, raw_folder, ser)  # Unpack the tuple
                    if jpeg_filename is None or not wait_for_file(jpeg_filename):
                        print(f"Error: Backward image detailed {i+1} file was not ready in time.")
                        continue

                    detailed_backward_canny = canny_value(jpeg_filename)
                    print(f"Backward Detailed Image {i+1} Canny Value: {detailed_backward_canny:.4f}")
                    if detailed_backward_canny < low_threshold_exit:
                        out_of_focus_count += 1
                    else:
                        out_of_focus_count = 0

                    i += 1
                break

        if not sharp_area_found:
            print("No sharp area found. Checking if return to start position is needed.")

            if c_position != start_position:
                print("Returning to start position and trying forward.")
                send_command_to_arduino(ser, f"c{start_position} {speed_back}")
                wait_for_all_axes_movement_done(ser, ["C"])
                c_position = start_position
            else:
                print("Already at the start position, skipping redundant command.")

            for k in range(10):
                send_command_to_arduino(ser, f"C{small_steps_forward} {speed_forward}")
                wait_for_all_axes_movement_done(ser, ["C"])
                time.sleep(1)
                c_position += small_steps_forward

                new_base_filename = f"forward_search_image_{k+1}"
                jpeg_filename, _ = capture_image(new_base_filename, raw_folder, ser)  # Unpack the tuple
                if jpeg_filename is None or not wait_for_file(jpeg_filename):
                    print(f"Error: Forward search image {k+1} file was not ready in time.")
                    continue
                
                forward_canny = canny_value(jpeg_filename)
                print(f"Forward Search Image {k+1} Canny Value: {forward_canny:.4f}")
                if forward_canny >= high_threshold:
                    print("Sharp area found during forward search.")
                    out_of_focus_count = 0
                    j = k
                    while out_of_focus_count < 12:
                        send_command_to_arduino(ser, f"C{small_steps_forward} {speed_forward}")
                        wait_for_all_axes_movement_done(ser, ["C"])
                        time.sleep(1)
                        c_position += small_steps_forward

                        new_base_filename = f"forward_image_{j+1}"
                        jpeg_filename, _ = capture_image(new_base_filename, raw_folder, ser)  # Unpack the tuple
                        if jpeg_filename is None or not wait_for_file(jpeg_filename):
                            print(f"Error: Forward image {j+1} file was not ready in time.")
                            continue
                        
                        forward_canny = canny_value(jpeg_filename)
                        print(f"Forward Image {j+1} Canny Value: {forward_canny:.4f}")

                        if forward_canny < low_threshold_exit:
                            out_of_focus_count += 1
                        else:
                            out_of_focus_count = 0

                        j += 1
                    break

    # Ensure we return to the start position at the end
    if c_position != start_position:
        print("Returning to starting position.")
        send_command_to_arduino(ser, f"c{start_position} {speed_back}")
        wait_for_all_axes_movement_done(ser, ["C"])
        c_position = start_position

    print("CStack imaging process completed.")
    return c_position






def execute_sequence_no_lasers(ser, sequence_data, current_positions, session_folder):
    """
    Execute a movement and action sequence from loaded CSV data without performing laser-detector checks.
    Sends all movement commands at once for each step and waits for all axes to finish moving.
    """
    for step_index, movement in enumerate(sequence_data):
        print(f"Executing step {step_index + 1}/{len(sequence_data)}...")
        
        # Send movement commands for all axes at once
        axes_to_move = []
        for axis in 'XYZABC':
            pos_key = f"{axis}_position"
            speed_key = f"{axis}_speed"
            
            if pos_key in movement:
                if movement[pos_key] is None:
                    print(f"Warning: {pos_key} is None in step {step_index + 1}, skipping axis {axis}")
                    continue
                if movement[pos_key] == "":
                    print(f"Warning: {pos_key} is empty string in step {step_index + 1}, skipping axis {axis}")
                    continue
                if movement[pos_key] != current_positions[axis]:
                    try:
                        target_position = int(movement[pos_key])
                    except ValueError:
                        print(f"Error: Cannot convert {movement[pos_key]} to int at step {step_index + 1} for axis {axis}")
                        continue
                    
                    speed = int(movement.get(speed_key, 100))
                    move_command = f"{MOVE_COMMANDS[axis]}{target_position} {speed}"
                    send_command_to_arduino(ser, move_command)
                    print(f"Sent movement command for axis {axis}: {move_command}")
                    axes_to_move.append(axis)
                    current_positions[axis] = movement[pos_key]

        # Wait for all axes that were commanded to move
        if axes_to_move:
            print(f"Waiting for axes to finish moving: {', '.join(axes_to_move)}")
            wait_for_all_axes_movement_done(ser, axes_to_move)
        
        # Check for image capture commands
        if 'image' in movement and movement['image'] != '0':
            image_code = movement['image']
            handle_image_capture(image_code, ser, session_folder)

        print(f"Step {step_index + 1} completed.")

    print("Sequence execution completed without laser-detector checks.")














def main():
    print("Starting program")
    session_folder = initialize_session_folder()
    print(f"Session images and data will be saved in: {session_folder}")
    ser = initialize_arduino()
    if ser:
        positions = load_csv_data('instructions.csv')
        sequence_data = load_csv_data('sequence.csv')

        current_positions = set_initial_positions(ser, positions)

        while True:
            action = input("Do you want to 'run sequence', 'run sequence backwards', 'run sequence no lasers', 'zseries', 'save positions', 'image' or 'exit'? ").lower()
            if action == 'exit':
                print("Exiting...")
                break

            elif action == 'run sequence':
                session_folder = initialize_session_folder()
                execute_sequence(ser, sequence_data, current_positions, session_folder)
            
            elif action == 'run sequence backwards':
                session_folder = initialize_session_folder()
                execute_sequence_reverse(ser, sequence_data, current_positions, session_folder)

            elif action == 'run sequence no lasers':
                session_folder = initialize_session_folder()
                execute_sequence_no_lasers(ser, sequence_data, current_positions, session_folder)

            elif action == 'zseries':
                zseries(ser, session_folder)

            elif action == 'image':
                image_folder = session_folder
                ensure_folder_exists(image_folder)
                jpeg_filename, raw_filename = capture_image('latest_image', image_folder)
                if jpeg_filename and raw_filename:
                    print(f"Captured and saved JPEG: {jpeg_filename}")
                    print(f"Captured and saved RAW: {raw_filename}")
                else:
                    print("Failed to capture images.")

            elif action == 'save positions':
                for position in positions:
                    if position['type'] == 'position':
                        position['start_position'] = fetch_current_position(ser, position['axis'])

                try:
                    with open('instructions.csv', 'w', newline='') as csvfile:
                        fieldnames = ['type', 'axis', 'start_position', 'start_index', 'direction', 'end_index']
                        writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
                        writer.writeheader()
                        writer.writerows(positions)
                    print("Updated positions saved to 'instructions.csv'.")
                except Exception as e:
                    print(f"Failed to save updated positions: {e}")

if __name__ == '__main__':
    main()