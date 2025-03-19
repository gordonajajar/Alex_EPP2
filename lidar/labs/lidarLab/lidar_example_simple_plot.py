# ensure headless compatibility
import matplotlib
matplotlib.use('TKagg')

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
    scan_mode = 3
    print("")


    print("====== Scanning ======")
    # Start a single scan
    print(f"Starting a single scan on mode {scan_mode}...")
    
    # Get a single scan using the library
    scanResults = performSingleScan(lidar, scan_mode)

    # Disconnect from the LiDAR device
    print("Disconnecting from LiDAR device...")
    lidarDisconnect(lidar)

    # Get the scan angles and distances from the scanResults tuple
    scanAngles = scanResults[0]
    scanDistances = scanResults[1]

    # Prepare the Data for plotting
    # Our plotting function expects X and Y coordinates, but we have polar coordinates (angle, distance)
    # We need to convert the polar coordinates to cartesian coordinates
    Xs, Ys = convert_to_cartesian(scanAngles, scanDistances)


    # Plot the scan data using matplotlib. Matplotlib is a powerful plotting library for Python, and is widely used in the scientific community. Examples of its use can be found at https://matplotlib.org/stable/gallery/index.html
    # we will be plotting a very simple scatter plot of the scan data
    # we will create a figure and an axis object to plot on
    # subplots() is a function that creates a figure and rows x cols grid of subplots, returning a figure and an array of axes objects (unless rows and cols are both 1, in which case it returns a single axes object)
    figure, ax = plt.subplots(nrows=1, ncols=1, figsize=(10, 10))


    title="LiDAR single scan"
    xlabel="X Coordinates (mm)"
    ylabel="Y Coordinates (mm)"
    
    # Scatter plot keyword arguments
    plot_keyword_arguments = {
        's': 1,  # size of the points
        'c': 'blue',  # color of the points
        'alpha': 1  # transparency of the points
    }

    # Plotting the scan data
    # scatter() is a function that creates a scatter plot of x and y data
    # we will plot the scan data as a scatter plot. 
    #  **plot_keyword_arguments is a way to pass a dictionary of keyword arguments to a function. scatter() takes a number of keyword arguments, and we are passing them in this way.
    # this is the same as calling scatter(Xs, Ys, s=1, c='blue', alpha=0.5)
    ax.scatter(Xs, Ys, **plot_keyword_arguments)

    # We add a big red cross at the origin of the plot to signify the origin of the LiDAR scan
    ax.scatter(0, 0, c='red', s=100, marker='x')

    # Now we label the axes
    ax.set_xlabel(xlabel)
    ax.set_ylabel(ylabel)

    # make sure that the aspect ratio of the plot is equal, so that the distances are represented accurately
    ax.set_aspect('equal', 'box')

    # set the title of the plot
    ax.set_title(title)

    # show the plot
    plt.show()

if __name__ == "__main__":
    plot_single_scan()


