import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import serial
from matplotlib.widgets import Button
from matplotlib.collections import LineCollection
import sys
from matplotlib.lines import Line2D  # For custom legend handles

# Set up serial communication
ser = serial.Serial(
    port='COM4',
    baudrate=115200,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_TWO,
    bytesize=serial.EIGHTBITS
)
ser.isOpen()

# Global settings
dark_mode = False
xsize = 500

def get_color(temp):
    """Determine color based on temperature"""
    if temp < 35:
        return 'midnightblue'
    elif temp < 47:
        return 'mediumblue'
    elif temp < 60:
        return 'cornflowerblue'
    elif temp < 77:
        return 'lightsteelblue'
    elif temp < 90:
        return 'bisque'
    elif temp < 115:
        return 'lightsalmon'
    elif temp < 120:
        return 'orange'
    elif temp < 140:
        return 'tomato'
    elif temp < 150:
        return 'orangered'
    elif temp < 180:
        return 'red'
    else:
        return 'crimson'

def data_gen():
    """Generator function to read and yield temperature data"""
    if not hasattr(data_gen, "t"):
        data_gen.t = -1

    while True:
        data_gen.t += 1
        line = ser.readline().decode('ascii', errors='ignore').strip()

        if not line or line.startswith('b'):
            continue
        
        try:
            temperature = float(line)
            yield data_gen.t, temperature
        except ValueError:
            continue  # Ignore invalid lines

# Initialize figure and axes
fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 6), sharex=True)

# Set titles and labels
ax1.set_title('Temperature Monitor')
ax1.set_ylabel('Temperature (°C)')
ax1.set_ylim(-10, 300)
ax1.grid(True)

ax2.set_title('Temperature Derivative')
ax2.set_xlabel('Samples')
ax2.set_ylabel('dT/dt')
ax2.set_ylim(-10, 10)
ax2.grid(True)

# Data storage
xdata, ydata, derivative_data = [], [], []

# Lines for temperature and derivative
ax1.set_xlim(0, xsize)
temp_line, = ax1.plot([], [], lw=2, label="Temperature")
deriv_line, = ax2.plot([], [], lw=2, color='black', linestyle='dashed', label="dT/dt")

ax1.legend()
ax2.legend()

def run(data):
    """Update function for animation"""
    t, y = data
    if t > -1 and y is not None:
        xdata.append(t)
        ydata.append(y)

        # Compute and plot derivative if there are enough points
        if len(ydata) > 1:
            dy_dt = (ydata[-1] - ydata[-2]) / (xdata[-1] - xdata[-2])
            derivative_data.append(dy_dt)
        else:
            derivative_data.append(0)  # First value is 0

        # Create a LineCollection for the temperature data with a color gradient
        segments = []
        for i in range(1, len(xdata)):
            segments.append([(xdata[i-1], ydata[i-1]), (xdata[i], ydata[i])])
        
        # Use the LineCollection to apply color gradient
        lc = LineCollection(segments, colors=[get_color(y) for y in ydata[1:]], linewidth=2)
        ax1.add_collection(lc)

        # Update the derivative line
        deriv_line.set_data(xdata, derivative_data)

        # Smooth scrolling window
        ax1.set_xlim(max(0, t - xsize), t)
        ax2.set_xlim(max(0, t - xsize), t)

    return temp_line, deriv_line

def toggle_theme(event):
    """Toggle between light and dark mode"""
    global dark_mode
    dark_mode = not dark_mode
    
    if dark_mode:
        plt.style.use('dark_background')
        ax1.set_facecolor('#121212')
        ax2.set_facecolor('#121212')
    else:
        plt.style.use('default')
        ax1.set_facecolor('white')
        ax2.set_facecolor('white')

    plt.draw()

# Add theme toggle button
ax_button = plt.axes([0.8, 0.005, 0.15, 0.075])
button = Button(ax_button, 'Toggle Theme')
button.on_clicked(toggle_theme)

# Define custom legend handles based on the temperature ranges
legend_labels = [
    ('<35°C', 'midnightblue'),
    ('35°C - 47°C', 'mediumblue'),
    ('47°C - 60°C', 'cornflowerblue'),
    ('60°C - 77°C', 'lightsteelblue'),
    ('77°C - 90°C', 'bisque'),
    ('90°C - 115°C', 'lightsalmon'),
    ('115°C - 120°C', 'orange'),
    ('120°C - 140°C', 'tomato'),
    ('140°C - 150°C', 'orangered'),
    ('150°C - 180°C', 'red'),
    ('>180°C', 'crimson')
]

# Create custom legend handles
custom_legend_handles = [Line2D([0], [0], color=color, lw=6) for label, color in legend_labels]

# Add custom legend for temperature ranges inside the graph
ax1.legend(custom_legend_handles, [label for label, color in legend_labels], loc='upper left', bbox_to_anchor=(0.02, 0.98),
           fontsize=8, handlelength=2, handleheight=1)

# Start animation
ani = animation.FuncAnimation(fig, run, data_gen, blit=False, interval=100, repeat=False)

plt.show()
