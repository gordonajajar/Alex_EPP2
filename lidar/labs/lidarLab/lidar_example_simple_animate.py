# ensure headless compatibility
import matplotlib
matplotlib.use('TKagg')

import matplotlib.axes
import matplotlib.pyplot as plt
import numpy as np
from lidar.alex_lidar import  BAUDRATE, lidarConnect, lidarDisconnect, process_scan, lidarStatus, startScan, performSingleScan



PORT = "/dev/ttyUSB0"   # Please change this to the correct port for your device if necessary
BAUDRATE = 115200

def convert_to_cartesian(angles, distances):
    # Convert the scanAngles and scanDistances into X and Y coordinates
    # Hint, Pythagoras' theorem can be used to convert polar to cartesian coordinates
    # We've provided you with the value of PI
    VAL_PI = np.pi

    Xs = []
    Ys = []
    for angle, distance in zip(angles, distances):
        # TODO: Fill in the code to convert the polar coordinates to cartesian coordinates
        # Hint, Pythagoras' theorem can be used to convert polar to cartesian coordinates
        cartesian_X = distance*np.cos((angle*VAL_PI)/180)
        cartesian_Y = distance*np.sin((angle*VAL_PI)/180)

        Xs.append(cartesian_X)
        Ys.append(cartesian_Y)

        pass
    return Xs, Ys

def inner_plot(ax, lidar, scan_mode):
    # Get a single scan using the library
    scanResults = performSingleScan(lidar, scan_mode)

    # Get the scan angles and distances from the scanResults tuple
    scanAngles = scanResults[0]
    scanDistances = scanResults[1]

    # Convert the scan data to cartesian coordinates
    Xs, Ys = convert_to_cartesian(scanAngles, scanDistances)
    title="LiDAR single scan"
    xlabel="X Coordinates (mm)"
    ylabel="Y Coordinates (mm)"
    
    # Scatter plot keyword arguments
    plot_keyword_arguments = {
        's': 1,  # size of the points
        'c': 'blue',  # color of the points
        'alpha': 1  # transparency of the points
    }
    ax.scatter(Xs, Ys, **plot_keyword_arguments)
    ax.scatter(0, 0, c='red', s=100, marker='x')
    ax.set_aspect('equal', 'box')
    ax.set_title("LiDAR single scan")
    ax.set_xlabel("X Coordinates (mm)")
    ax.set_ylabel("Y Coordinates (mm")

    # This time, we limit the plot to the range of the scan data
    x_limit_mm = 2000
    y_limit_mm = 2000

    ax.set_xlim(-x_limit_mm, x_limit_mm)
    ax.set_ylim(-y_limit_mm, y_limit_mm)




def plot_single_scan():
    """
    Main function to demonstrate a plotting a single scan using the LiDAR device.
    
    """
    # Connect to lidar
    print("====== LiDAR Plot test ======")
    lidar = lidarConnect(port=PORT, baudrate=BAUDRATE, wait=2)
    print("Connected to LiDAR device on port", PORT)
    print("Baudrate:", BAUDRATE)
    print("")

    print("====== Device information ======")
    # Retrieve and print the LiDAR device information
    print("Retrieving LiDAR device information...")
    status = lidarStatus(lidar, verbose=True)
    scan_mode = status['typical_scan_mode']
    print("")


    # Plot the scan data using matplotlib. Matplotlib is a powerful plotting library for Python, and is widely used in the scientific community. Examples of its use can be found at https://matplotlib.org/stable/gallery/index.html
    # we will be plotting a very simple scatter plot of the scan data
    # we will create a figure and an axis object to plot on
    # subplots() is a function that creates a figure and rows x cols grid of subplots, returning a figure and an array of axes objects (unless rows and cols are both 1, in which case it returns a single axes object)
    figure, ax = plt.subplots(nrows=1, ncols=1, figsize=(10, 10))

    # Unlike the previous example, we extract the plotting code into a separate function
    # This function will take the axis object and the lidar object as arguments
    # And will be called in a loop to update the plot

    # show the plot, this time, we will not block the code execution
    # Instead, we will update the plot in a loop
    plt.show(block=False)
    plt.pause(0.1)
    # We simply animate the plot by repeatedly grabbing a single scan and updating the plot
    # We will use the same plotting code as above, but we will put it in a loop
    # We also surround the loop with a try-except block so that we can exit the loop cleanly using
    # the keyboard interrupt (Ctrl+C)
    try:
        while plt.fignum_exists(figure.number):
            ax.clear()
            inner_plot(ax, lidar, scan_mode)
            plt.pause(0.1) # Pause is needed to let the GUI update/refresh
    except KeyboardInterrupt:
        lidarDisconnect(lidar)
        print("Exiting...")



if __name__ == "__main__":
    plot_single_scan()


