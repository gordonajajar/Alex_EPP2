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

# Send command to Arduino (output via curses only)
def send_command(cmd, stdscr=None, msg_row=20):
    with serial_lock:
        try:
            ser.write(f"{cmd}\n".encode())
            if stdscr:
                stdscr.move(msg_row, 0)
                stdscr.clrtoeol()
                stdscr.addstr(msg_row, 0, f"[Sent]: {cmd}")
                stdscr.refresh()
        except Exception as e:
            if stdscr:
                stdscr.move(msg_row, 0)
                stdscr.clrtoeol()
                stdscr.addstr(msg_row, 0, f"[Error sending]: {e}")
                stdscr.refresh()

def read_from_serial(stdscr, start_row=12):
    row = start_row
    max_y, max_x = stdscr.getmaxyx()
    max_scroll_row = max_y - 3

    while True:
        try:
            while ser.in_waiting:
                raw = ser.readline().decode('utf-8', errors='ignore').strip()
                if raw:
                    for line in raw.splitlines():
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

    max_y, _ = stdscr.getmaxyx()
    sent_msg_row = max_y - 1
    current_key = None

    serial_thread = threading.Thread(target=read_from_serial, args=(stdscr,), daemon=True)
    serial_thread.start()

    while True:
        try:
            key = stdscr.getch()

            key = stdscr.getch()

            if key != -1:
                ch = chr(key).lower()

                if ch in MOVEMENT_KEYS:
                    if ch != current_key:
                        send_command(CMD_MAP[ch], stdscr, msg_row=sent_msg_row)
                        current_key = ch

                    # flush remaining repeat keys of same type only
                    while True:
                        next_key = stdscr.getch()
                        if next_key == -1 or chr(next_key).lower() != ch:
                            break

                elif ch in ONE_TIME_KEYS:
                    send_command(CMD_MAP[ch], stdscr, msg_row=sent_msg_row)

                elif ch == 'q':
                    send_command(CMD_MAP['q'], stdscr, msg_row=sent_msg_row)
                    current_key = None

                elif key == 27:  # ESC
                    send_command(CMD_MAP['q'], stdscr, msg_row=sent_msg_row)
                    break

            else:
                if current_key:
                    send_command(CMD_MAP['q'], stdscr, msg_row=sent_msg_row)
                    current_key = None

            time.sleep(0.05)

        except Exception as e:
            stdscr.move(sent_msg_row, 0)
            stdscr.clrtoeol()
            stdscr.addstr(sent_msg_row, 0, f"[Control error]: {e}")
            stdscr.refresh()
            break

def main():
    global ser
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
        time.sleep(2)  # Allow Arduino to boot
        curses.wrapper(control_robot)  # Ensures reset if no crash

    except Exception as e:
        print(f"[Startup error]: {e}")  # Outside curses

    finally:
        if ser and ser.is_open:
            ser.close()

        # Reset terminal cleanly even if error occurred
        try:
            curses.endwin()
        except:
            pass

if __name__ == "__main__":
    main()

