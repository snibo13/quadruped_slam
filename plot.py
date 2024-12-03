import csv
import matplotlib.pyplot as plt
import numpy as np
from scipy.spatial.transform import Rotation as R
import sys
from mpl_toolkits.mplot3d import Axes3D


def read_csv(filename):
    """
    Reads the CSV file and extracts time, position, and orientation data.
    """
    time = []
    x = []
    y = []
    z = []
    qx = []
    qy = []
    qz = []
    qw = []

    with open(filename, "r") as file:
        reader = csv.DictReader(file)
        print(reader.fieldnames)
        # # Read the data
        for row in reader:
            time.append(float(row["time(s)"]))
            x.append(float(row["x(m)"]))
            y.append(float(row["y(m)"]))
            z.append(float(row["z(m)"]))
            qx.append(float(row["qx"]))
            qy.append(float(row["qy"]))
            qz.append(float(row["qz"]))
            qw.append(float(row["qw"]))

    return time, x, y, z, qx, qy, qz, qw


def plot_2d_position(x, y, time):
    """
    Plots the 2D position (x, y) with color changing as a function of time.
    """
    # Normalize the time values for mapping to a colormap
    norm_time = (time - np.min(time)) / (np.max(time) - np.min(time))

    # Create a scatter plot with color mapped to time
    plt.figure(figsize=(10, 6))
    scatter = plt.scatter(x, y, c=norm_time, cmap="viridis")
    plt.xlabel("X Position")
    plt.ylabel("Y Position")
    plt.colorbar(scatter, label="Normalized Time")
    plt.grid(True)
    plt.axis("equal")
    plt.show()


def plot_3d_position(x, y, z, time):
    """
    Plots the 3D position (x, y, z) with color changing as a function of time.
    """
    # Normalize time for color mapping
    norm_time = (time - np.min(time)) / (np.max(time) - np.min(time))

    # Create the 3D plot
    fig = plt.figure(figsize=(10, 6))
    ax = fig.add_subplot(111, projection="3d")

    # Use scatter plot for color mapping
    scatter = ax.scatter(x, y, z, c=norm_time, cmap="viridis", marker="o", s=50)

    # Add labels and title
    ax.set_xlabel("X Position")
    ax.set_ylabel("Y Position")
    ax.set_zlabel("Z Position")

    # Add colorbar to indicate time mapping
    colorbar = fig.colorbar(scatter, ax=ax, pad=0.2, shrink=0.6)
    colorbar.set_label("Normalized Time")

    plt.show()


def plot_3d_position_with_orientation(x, y, z, time, qx, qy, qz, qw, skip=10):
    """
    Plots the 3D position (x, y, z) with orientation represented as arrows.
    """

    x = x[::skip]
    y = y[::skip]
    z = z[::skip]
    qx = qx[::skip]
    qy = qy[::skip]
    qz = qz[::skip]
    qw = qw[::skip]
    time = time[::skip]

    # Normalize time for color mapping
    norm_time = (time - np.min(time)) / (np.max(time) - np.min(time))

    # Create the 3D plot
    fig = plt.figure(figsize=(10, 6))
    ax = fig.add_subplot(111, projection="3d")

    # Use scatter plot for color mapping
    scatter = ax.scatter(x, y, z, c=norm_time, cmap="viridis", marker="o", s=5)

    # Convert quaternions to rotation matrices to extract orientation vectors
    rotations = R.from_quat(np.column_stack((qx, qy, qz, qw)))
    orientation_vectors = rotations.apply(
        [1, 0, 0]
    )  # Assume forward direction is [1, 0, 0]

    # Add arrows for orientation
    for i in range(len(x)):
        ax.quiver(
            x[i],
            y[i],
            z[i],  # Starting point
            orientation_vectors[i, 0],
            orientation_vectors[i, 1],
            orientation_vectors[i, 2],  # Direction
            length=100,
            normalize=True,
            color=plt.cm.viridis(norm_time[i]),
        )

    # Add labels and title
    ax.set_xlabel("X Position")
    ax.set_ylabel("Y Position")
    ax.set_zlabel("Z Position")
    ax.set_title("3D Position Plot with Orientation")

    # Add colorbar to indicate time mapping
    colorbar = fig.colorbar(scatter, ax=ax, pad=0.2, shrink=0.6)
    colorbar.set_label("Normalized Time")

    plt.show()


if __name__ == "__main__":
    # Read the first argument as the CSV filename
    print(sys.argv)
    csv_filename = sys.argv[1]

    # Read the data
    time, x, y, z, qx, qy, qz, qw = read_csv(csv_filename)

    # Plot the 2D position
    plot_2d_position(x, y, time)

    # Plot the 3D position
    plot_3d_position(x, y, z, time)

    # Plot the 3D position with orientation
    plot_3d_position_with_orientation(x, y, z, time, qx, qy, qz, qw, 100)
