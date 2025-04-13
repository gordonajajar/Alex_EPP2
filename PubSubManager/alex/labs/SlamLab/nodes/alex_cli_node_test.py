# This node is an example of a simple publisher monitors the input from the user and publishes commands to the "arduino/send" topic.

# Import Python Native Modules. We require the Barrier class from threading to synchronize the start of multiple threads.
from threading import Barrier
import signal

# Import the required pubsub modules. PubSubMsg class to extract the payload from a message.
from pubsub.pub_sub_manager import ManagedPubSubRunnable, PubSubMsg
from pubsub.pub_sub_manager import publish, subscribe, unsubscribe, getMessages, getCurrentExecutionContext

# Import the command parser from the control module
from control.alex_control import parseUserInput

import curses, time

# Constants
ARDUINO_SEND_TOPIC = "arduino/send"

def cliThread(setupBarrier:Barrier=None, readyBarrier:Barrier=None):
    """
    Starts a command thread that interacts with the user. Publishes commands to the "arduino/send" topic for the send thread to handle sending the commands to the Arduino.
    
    Args:
        setupBarrier (Barrier, optional): A threading barrier to synchronize initial setup steps. Defaults to None.
        readyBarrier (Barrier, optional): A threading barrier to synchronize the start of the thread. Defaults to None.
    
    The function performs the following steps:
    1. Sets up the execution context.
    2. Waits for setup to complete if barriers are provided.
    3. Initiates a user interaction loop to receive and parse commands.
    4. Exits gracefully when an exit condition is met.

    Note:
        input is a blocking call, so the thread will wait for user input before proceeding. This means that even if the exit condition is met while waiting for input, the thread remains blocked until input is received (i.e., the user enters a command).

    """

    # Perform any setup here
    pass
    ctx:ManagedPubSubRunnable = getCurrentExecutionContext()

    # Perform any setup here
    setupBarrier.wait() if readyBarrier != None else None

    print(f"CLI Thread Ready. Publishing to {ARDUINO_SEND_TOPIC}")
    try:
        curses.wrapper(runTeleop, ctx)
    except Exception as e:
        print("Exception in run", e)
    # Wait for all Threads ready
    readyBarrier.wait() if readyBarrier != None else None
    
    # Shutdown and exit the thread gracefully
    ctx.doExit()
    print("Exiting Command Thread")
    pass


def runTeleop(stdscr, ctx):
    print("RUN TELEOP ENTERED")
    try:
        stdscr.nodelay(True)
        curses.cbreak()
        moving = False
        last_key_seen = time.time()
        last_key = ""
        movement_list = [ord('w'),ord('a'),ord('s'),ord('d')]

        while(not ctx.isExit()):
            key = stdscr.getch()
            now = time.time()

            if moving and now - last_key_seen > 0.12:
                parseResult = parseUserInput('q', exitFlag=ctx.exitEvent)

            elif key in movement_list:
                last_key_seen = now
                if not moving:
                    parseResult = parseUserInput(key, exitFlag=ctx.exitEvent)
                    moving = True
                    last_key = key

            else:
                parseResult = parseUserInput(key, exitFlag=ctx.exitEvent)

            #input_str = input("Command (p=color, d=dist, o=open, i=close, w=forward, s=reverse, a=turn left, d=turn right, q=stop, c=clear stats, g=get stats v=exit)\n")
            #parseResult = parseUserInput(input_str, exitFlag=ctx.exitEvent)
            # if the parse result is None then the user entered an invalid command
            if parseResult == None:
                # print("Invalid command. Please try again.")
                continue
            else:
                # if the parse result is not None then the user entered a valid command
                # and the command has been published to the "arduino/send" topic
                publish(ARDUINO_SEND_TOPIC, tuple(parseResult))

            time.sleep(0.02)
            # [Optional: Consider enforcing the user to wait for the arduino to respond before sending the next command]
        pass

    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"CLI Thread Exception: {e}")
        pass

