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

# Variables to track consecutive laser errors
consecutive_laser_errors = 0

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
    """
    Reads all available lines from the Arduino serial buffer and returns them as a list.
    """
    lines = []
    while ser.in_waiting > 0:
        response = ser.readline().decode().strip()
        if response:
            print(f"Arduino says: {response}")
            lines.append(response)
    return lines

def wait_for_movement_done(ser):
    """ Waits for a 'movement done' message from the Arduino. """
    while True:
        if ser.in_waiting > 0:
            response = ser.readline().decode().strip()
            print(f"Arduino says: {response}")
            if "movement done" in response:
                break

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
    responses = read_response_from_arduino(ser)
    
    for response in responses:
        if "Current" in response:
            try:
                position = int(response.split()[-1])
                return position
            except ValueError:
                print(f"Invalid response received for axis {axis}: {response}")
    
    print(f"Failed to fetch current position for axis {axis}.")
    return None


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

def execute_sequence(ser, sequence_data, current_positions, session_folder, start_index, end_index, direction):
    global consecutive_laser_errors
    if direction == 'r':
        sequence_range = range(start_index, end_index - 1, -1)
    else:
        sequence_range = range(start_index, end_index + 1)

    for step_index in sequence_range:
        movement = sequence_data[step_index]
        print(f"Executing step {step_index + 1}/{len(sequence_data)}...")

        for axis in 'XYZABC':  # Assuming control of up to six axes (X, Y, Z, A, B, C)
            pos_key = f"{axis}_position"
            speed_key = f"{axis}_speed"
            
            if pos_key in movement and movement[pos_key] != current_positions[axis]:
                target_position = int(movement[pos_key])
                speed = int(movement.get(speed_key, 100))  # Default speed if not specified

                calculate_and_apply_delay(ser, axis, target_position, speed)
                current_positions[axis] = movement[pos_key]

                print(f"Moved axis {axis} to position {movement[pos_key]} at speed {speed}.")

        if 'image' in movement and movement['image'] != '0':
            image_code = movement['image']
            handle_image_capture(image_code, ser, session_folder)

            # Remove or conditionally apply C axis reset to 10
            # current_c_position = fetch_current_position(ser, 'C')
            # if current_c_position is not None and int(current_c_position) != 10:
            #     calculate_and_apply_delay(ser, 'C', 10, 100)
            #     current_positions['C'] = 10
            #     print("Moved axis C to position 10 after image capture.")

        send_command_to_arduino(ser, 'D')
        responses = read_response_from_arduino(ser)

        laser_errors = [resp for resp in responses if "Laser" in resp and "NOT DETECTED" in resp]
        
        if laser_errors:
            consecutive_laser_errors += 1
            if consecutive_laser_errors >= 2:
                # Stop all motors if the error occurs twice in a row
                stop_all_motors(ser)
                print("Laser NOT DETECTED twice consecutively. Stopping all motors.")
                break
        else:
            consecutive_laser_errors = 0

        print(f"Step {step_index + 1} completed.")

    current_c_position = fetch_current_position(ser, 'C')
    if current_c_position is not None and int(current_c_position) != 10:
        calculate_and_apply_delay(ser, 'C', 10, 100)
        current_positions['C'] = 10
        print("Reset axis C to position 10 after sequence completion.")



def stop_all_motors(ser):
    """ Send stop command to all motors """
    for axis in 'XYZABC':
        send_command_to_arduino(ser, f'{axis.lower()}0')
        print(f"Stopped motor {axis}")

def handle_image_capture(image_code, ser, session_folder):
    """
    Handle image capture based on the code in the sequence data.
    """
    global session_count
    session_count += 1
    cstack_folder = os.path.join(session_folder, f"CStack_{session_count}")
    os.makedirs(cstack_folder, exist_ok=True)
    
    if image_code == '1':
        base_filename = f"image_1_{datetime.datetime.now().strftime('%Y%m%d_%H%M%S')}"
        image_folder = os.path.join(cstack_folder, 'images')
        ensure_folder_exists(image_folder)
        jpeg_filename, raw_filename = capture_image(base_filename, image_folder)
        
        if jpeg_filename and raw_filename:
            print(f"Captured and saved JPEG: {jpeg_filename}")
            print(f"Captured and saved RAW: {raw_filename}")
        else:
            print("Failed to capture images.")
    elif image_code == '2':
        print("Launching CStack process...")
        CStack(ser, cstack_folder)


def capture_image(base_filename, raw_folder):
    raw_filename = os.path.join(raw_folder, f"{base_filename}.cr2")
    jpeg_filename = os.path.join(raw_folder, f"{base_filename}.jpg")

    print(f"Expected to save RAW image at: {raw_filename}")
    print(f"Expected to save JPEG image at: {jpeg_filename}")

    capture_result = subprocess.run([
        '/usr/bin/gphoto2', '--capture-image-and-download',
        '--filename', f"{raw_folder}/{base_filename}.%C",
        '--force-overwrite'
    ], text=True, capture_output=True)

    print(f"gphoto2 stdout: {capture_result.stdout}")
    print(f"gphoto2 stderr: {capture_result.stderr}")

    # Verify file existence
    jpeg_exists = os.path.exists(jpeg_filename)
    raw_exists = os.path.exists(raw_filename)
    print(f"JPEG exists: {jpeg_exists}, RAW exists: {raw_exists}")

    if jpeg_exists and raw_exists:
        print("JPEG and RAW images captured successfully.")
    return jpeg_filename, raw_filename

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

def CStack(ser, session_folder):
    if not ser or not ser.is_open:
        print("Arduino not connected.")
        return

    high_threshold = 0.02
    low_threshold_exit = 0.0005
    big_steps_back = 200
    speed_back = 100
    small_steps_forward = 100
    speed_forward = 50

    raw_folder = os.path.join(session_folder, "CStack_RAW")
    os.makedirs(raw_folder, exist_ok=True)
    print(f"Images will be saved in: {session_folder}")

    base_filename = "initial_image"
    jpeg_filename, raw_filename = capture_image(base_filename, raw_folder)
    if jpeg_filename is None or not wait_for_file(jpeg_filename):
        print("Error: Initial JPEG file was not ready or failed to capture in time.")
        return

    initial_canny = canny_value(jpeg_filename)
    print(f"Initial Canny Value: {initial_canny:.4f}")

    if initial_canny >= high_threshold:
        print("Initial image is in focus, moving backwards to find out-of-focus start.")
        i = 0
        while True:
            ser.write(f"C-{big_steps_back} {speed_back}\n".encode())
            wait_for_movement_done(ser)
            time.sleep(1)
            new_base_filename = f"backward_image_{i+1}"
            jpeg_filename, raw_filename = capture_image(new_base_filename, raw_folder)
            if jpeg_filename is None or not wait_for_file(jpeg_filename):
                print(f"Error: Backward image {i+1} file was not ready in time.")
                continue

            backward_canny = canny_value(jpeg_filename)
            print(f"Backward Image {i+1} Canny Value: {backward_canny:.4f}")
            if backward_canny < low_threshold_exit:
                print("Exited sharp area, preparing for detailed forward capture.")
                break
            i += 1

    j = 0
    additional_captures = 3
    while True:
        ser.write(f"C{small_steps_forward} {speed_forward}\n".encode())
        wait_for_movement_done(ser)
        time.sleep(1)
        new_base_filename = f"forward_image_{j+1}"
        jpeg_filename, raw_filename = capture_image(new_base_filename, raw_folder)
        if jpeg_filename is None or not wait_for_file(jpeg_filename):
            print(f"Error: Forward image {j+1} file was not ready in time.")
            continue

        forward_canny = canny_value(jpeg_filename)
        print(f"Forward Image {j+1} Canny Value: {forward_canny:.4f}")

        if forward_canny < low_threshold_exit and additional_captures > 0:
            additional_captures -= 1
        elif additional_captures == 0:
            break

        j += 1

    print("CStackNoLasers imaging process completed.")

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
            action = input("Do you want to 'run sequence', 'save positions', 'image' or 'exit'? ").lower()
            if action == 'exit':
                print("Exiting...")
                break

            elif action == 'run sequence':
                direction = 'f'  # Default direction
                for position in positions:
                    if position['type'] == 'sequence':
                        direction = position['direction']
                        start_index = int(position['start_index'])
                        end_index = int(position['end_index'])
                session_folder = initialize_session_folder()
                execute_sequence(ser, sequence_data, current_positions, session_folder, start_index, end_index, direction)

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
