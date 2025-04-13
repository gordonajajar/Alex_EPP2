#!/usr/bin/env python3

import serial
import time
import threading
from pynput import keyboard

# Configuration
SERIAL_PORT = '/dev/ttyACM0'
BAUD_RATE = 9600

# Movement commands (simple single-character commands)
CMD_FORWARD = 'w'
CMD_BACKWARD = 's'
CMD_LEFT = 'a'
CMD_RIGHT = 'd'
CMD_STOP = 'q'
CMD_OPEN_CLAW = 'o'
CMD_CLOSE_CLAW = 'i'
CMD_GET_INFO = 'p'

# Initialize key states
key_states = {
    'w': False,  # Forward
    's': False,  # Backward
    'a': False,  # Left
    'd': False,  # Right
    'o': False,  # Open claw
    'i': False,  # Close claw
    'p': False,  # Get color and distance
}

# Serial connection
ser = None

# Set up a lock for thread-safe serial communication
serial_lock = threading.Lock()

# Function to send command to Arduino
def send_command(command):
    with serial_lock:
        try:
            ser.write(f"{command}\n".encode())
            print(f"Sent: {command}")
        except Exception as e:
            print(f"Error sending command: {e}")

# Movement functions
def move_forward():
    send_command(CMD_FORWARD)

def move_backward():
    send_command(CMD_BACKWARD)

def turn_left():
    send_command(CMD_LEFT)

def turn_right():
    send_command(CMD_RIGHT)

def stop_robot():
    send_command(CMD_STOP)

def open_claw():
    send_command(CMD_OPEN_CLAW)

def close_claw():
    send_command(CMD_CLOSE_CLAW)

def get_info():
    send_command(CMD_GET_INFO)

# Function that continuously sends movement commands based on key states
def movement_loop():
    last_state = dict(key_states)
    
    while True:
        # Check if key states have changed
        if key_states != last_state:
            # Process movement based on current key states
            if key_states['w']:
                move_forward()
            elif key_states['s']:
                move_backward()
            elif key_states['a']:
                turn_left()
            elif key_states['d']:
                turn_right()
            elif not any([key_states[k] for k in ['w', 'a', 's', 'd']]):
                # Only stop movement motors if no movement keys are pressed
                stop_robot()
            
            # Process claw commands
            if key_states['o'] and not last_state['o']:
                open_claw()
                key_states['o'] = False  # Reset flag after execution
            
            if key_states['i'] and not last_state['i']:
                close_claw()
                key_states['i'] = False  # Reset flag after execution

            if key_states['p'] and not last_state['p']:
                close_claw()
                key_states['p'] = False  # Reset flag after execution
                
            # Update last state
            last_state = dict(key_states)
            
        time.sleep(0.1)  # Small delay to avoid flooding the serial port

# Keyboard event listeners
def on_press(key):
    try:
        k = key.char.lower()
        if k in key_states:
            key_states[k] = True
    except AttributeError:
        pass

def on_release(key):
    try:
        k = key.char.lower()
        if k in key_states and k in ['w', 'a', 's', 'd']:
            key_states[k] = False
            
        # Check for exit command (Esc key)
        if key == keyboard.Key.esc:
            stop_robot()
            print("Exiting program...")
            return False  # Stop the listener
    except AttributeError:
        pass

# Main function
def main():
    global ser
    
    try:
        # Connect to Arduino
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
        print(f"Connected to Arduino on {SERIAL_PORT}")
        time.sleep(2)  # Allow time for Arduino connection to initialize
        
        # Start the movement control loop in a separate thread
        movement_thread = threading.Thread(target=movement_loop, daemon=True)
        movement_thread.start()
        
        # Print instructions
        print("\nRobot Control Instructions:")
        print("W - Move forward")
        print("S - Move backward")
        print("A - Turn left")
        print("D - Turn right")
        print("O - Open claw")
        print("I - Close claw")
        print("P - Get color and distance")
        print("ESC - Exit")
        
        # Set up the keyboard listener
        with keyboard.Listener(on_press=on_press, on_release=on_release) as listener:
            listener.join()
            
    except Exception as e:
        print(f"Error: {e}")
    finally:
        if 'ser' in globals() and ser is not None and ser.is_open:
            ser.close()
            print("Serial connection closed")

if __name__ == "__main__":
    main()
