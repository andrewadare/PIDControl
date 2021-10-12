#!/usr/bin/env python3

from collections import deque
from time import time, sleep

import numpy as np
import gr

from pid_controller import PIDController


class EmaFilter:
    """Exponential moving average filter"""

    def __init__(self, alpha=0.95):
        self.alpha = alpha  # Filtering factor 0 <= alpha < 1
        self.ma = 0  # Moving average

    def ema(self, x):
        self.ma = (1 - self.alpha) * x + self.alpha * self.ma
        return self.ma


def init_plot_window(xmin, xmax, ymin, ymax):
    gr.clearws()
    gr.setwsviewport(0.0, 0.2, 0.0, 0.2)  # Desktop window extents in meters
    gr.setviewport(0.15, 0.95, 0.15, 0.95)
    gr.setwindow(xmin, xmax, ymin, ymax)


def draw_axes(
    x_tick_spacing,
    y_tick_spacing,
    x_origin,
    y_origin,
    x_major=5,
    y_major=5,
    x_title="x",
    y_title="y",
):
    """
    Parameters
    ----------
    x_tick_spacing, y_tick_spacing : float
        Distance between ticks in data units
    x_origin, y_origin : float
        Location of plot origin in data units
    x_major, y_major : int
        Every x_major-th tick will be a labeled major tick on the x axis. Same for y.
        The ticks in between are unlabeled minor ticks.
    """
    gr.setlinewidth(1)
    gr.axes(x_tick_spacing, y_tick_spacing, x_origin, y_origin, x_major, y_major, -0.01)
    midway = 0.54
    gr.textext(midway, 0.02, "x")
    gr.setcharup(-1, 0)  # Vertical, end-up
    gr.textext(0.05, midway, "y")
    gr.setcharup(0, 1)  # Back to horizontal


def linecolor(r, g, b):
    """
    Parameters
    ----------
    r, g, b: float
        Color intensities in [0.0, 1.0]
    """
    gr.setlinecolorind(gr.inqcolorfromrgb(r, g, b))


def main():
    ymin, ymax = 0.0, 5.0
    timestep = 0.03
    kp, ki, kd = 0.5, 8.0, 0.001

    pid = PIDController(
        kp, ki, kd, (ymax - ymin) / 2, timestep, min_output=0, max_output=1.0
    )

    plant = EmaFilter(alpha=0.7)

    init_plot_window(0, 1, 0, 1)

    queue_size = 100
    t = deque(maxlen=queue_size)
    y1 = deque(maxlen=queue_size)
    y2 = deque(maxlen=queue_size)

    counter = 0
    target = 0.0
    t0 = time()

    while True:
        start = time()
        if counter % 100 == 0:
            target = np.random.randint(low=1, high=5)
            pid.setpoint = target / ymax  # Normalize to lie inside [0, 1]

        # Simulation of measured input
        plant_value = plant.ema(pid.output * (ymax - ymin))

        pid.update(plant_value / (ymax - ymin), time())

        t.append(time() - t0)
        y1.append(target)
        y2.append(pid.output * (ymax - ymin))

        if counter > 0:
            xmin, xmax = t[0], t[-1]
            # ymin, ymax = min(min(y1), min(y2)), max(max(y1), max(y2))
            gr.clearws()
            gr.setwindow(xmin, xmax, ymin, ymax)

            # Target
            gr.setlinewidth(2)
            linecolor(0, 0, 1.0)
            gr.polyline(t, y1)

            # Controller value
            gr.setlinewidth(2)
            linecolor(1.0, 0, 0)
            gr.polyline(t, y2)

            gr.setlinewidth(1)
            linecolor(0, 0, 0)
            draw_axes(1.0, 5.0 / 10, xmin, ymin, x_major=2, y_major=2)
            gr.updatews()
        counter += 1
        sleep(max(timestep - (time() - start), 0.0))


main()
