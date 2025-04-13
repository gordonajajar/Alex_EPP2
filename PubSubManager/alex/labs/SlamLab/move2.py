#!/usr/bin/env python3
import serial
import time
import threading
import curses

# Configuration
SERIAL_PORT = '/dev/ttyACM0'
BAUD_RATE = 9600

# Command map
MOVEMENT_KEYS = ['w', 'a', 's', 'd']
ONE_TIME_KEYS = ['o', 'i', 'p']
CMD_MAP = {
    'w': 'w',  # Forward
    's': 's',  # Backward
    'a': 'a',  # Left
    'd': 'd',  # Right
    'q': 'q',  # Stop
    'o': 'o',  # Open claw
    'i': 'i',  # Close claw
    'p': 'p',  # Get info
}

# Thread-safe serial access
serial_lock = threading.Lock()
ser = None

def send_command(cmd, stdscr=None):
    with serial_lock:
        try:
            ser.write(f"{cmd}\n".encode())
            if stdscr:
                stdscr.addstr(10, 0, f"Sent: {cmd}        ")  # Overwrite previous line
                stdscr.clrtoeol()
                stdscr.refresh()
            else:
                print(f"Sent: {cmd}")
        except Exception as e:
            if stdscr:
                stdscr.addstr(10, 0, f"Error sending command: {e}")
                stdscr.clrtoeol()
                stdscr.refresh()
            else:
                print(f"Error sending command: {e}")

def control_robot(stdscr):
    stdscr.nodelay(True)
    stdscr.clear()
    stdscr.addstr("Remote Robot Control\n\n")
    stdscr.addstr("W - Forward\n")
    stdscr.addstr("S - Backward\n")
    stdscr.addstr("A - Turn Left\n")
    stdscr.addstr("D - Turn Right\n")
    stdscr.addstr("O - Open Claw\n")
    stdscr.addstr("I - Close Claw\n")
    stdscr.addstr("P - Get Info\n")
    stdscr.addstr("Q - Stop\n")
    stdscr.addstr("ESC - Exit\n")
    stdscr.refresh()

    last_movement_command = None
    last_key_time = time.time()
    key_held = False

    max_y, _ = stdscr.getmaxyx()
    sent_msg_row = max_y - 1  # Bottom row for Sent messages

    serial_thread = threading.Thread(target=read_from_serial, args=(stdscr,), daemon=True)
    serial_thread.start()

    while True:
        try:
            key = stdscr.getch()
            movement_command = None

            if key != -1:
                ch = chr(key).lower()
                last_key_time = time.time()

                if ch in MOVEMENT_KEYS:
                    movement_command = CMD_MAP[ch]
                    key_held = True

                elif ch in ONE_TIME_KEYS:
                    send_command(CMD_MAP[ch], stdscr, msg_row=sent_msg_row)

                elif ch == 'q':
                    send_command(CMD_MAP['q'], stdscr, msg_row=sent_msg_row)
                    key_held = False
                    last_movement_command = None

                elif key == 27:  # ESC
                    send_command(CMD_MAP['q'], stdscr, msg_row=sent_msg_row)
                    break

            # Handle holding logic
            if key_held and movement_command and movement_command != last_movement_command:
                send_command(movement_command, stdscr, msg_row=sent_msg_row)
                last_movement_command = movement_command

            # If no key has been pressed for 200ms, stop the robot
            if key_held and time.time() - last_key_time > 0.2:
                send_command(CMD_MAP['q'], stdscr, msg_row=sent_msg_row)
                key_held = False
                last_movement_command = None

            time.sleep(0.05)

        except Exception as e:
            stdscr.move(sent_msg_row, 0)
            stdscr.clrtoeol()
            stdscr.addstr(sent_msg_row, 0, f"[Control error]: {e}")
            stdscr.refresh()
            break

def read_from_serial(stdscr, start_row=12):
    row = start_row
    while True:
        try:
            if ser.in_waiting:
                line = ser.readline().decode('utf-8', errors='ignore').strip()
                if line:
                    max_y, max_x = stdscr.getmaxyx()
                    max_scroll_row = max_y - 3
                    msg = f"[Arduino]: {line}"[:max_x - 1]

                    stdscr.move(row, 0)
                    stdscr.clrtoeol()
                    stdscr.addstr(row, 0, msg)
                    stdscr.refresh()

                    row += 1
                    if row >= max_scroll_row:
                        row = start_row
        except Exception as e:
            stdscr.move(max_y - 2, 0)
            stdscr.clrtoeol()
            stdscr.addstr(max_y - 2, 0, f"[Serial read error]: {e}")
            stdscr.refresh()
            break

# Main function
def main():
    global ser
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
        print(f"Connected to Arduino on {SERIAL_PORT}")
        time.sleep(2)  # Wait for Arduino to initialize

    
        curses.wrapper(control_robot)

    except Exception as e:
        print(f"Error: {e}")

    finally:
        if ser and ser.is_open:
            ser.close()
            print("Serial connection closed")

if __name__ == "__main__":
    main()

