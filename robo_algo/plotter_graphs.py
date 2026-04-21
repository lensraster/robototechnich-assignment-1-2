import time

import numpy as np
import pygame
from pygame.locals import KEYDOWN, K_ESCAPE, QUIT

from robo_algo.arm import RoboticArmPlotter
from robo_algo.constants import *
import robo_algo.core as core
from robo_algo.core import Color
from robo_algo.drawing_data import get_drawing1, get_drawing2, get_drawing3


if __name__ == "__main__":
    ctx = core.RenderingContext("graph test")
    arm1 = RoboticArmPlotter(
        ctx,
        joint0_position=np.array([8, 8]),
        link_lengths=[2, 3, 4],
        link_angles=[np.deg2rad(0), np.deg2rad(10), np.deg2rad(0)],
        thickness=0.1,
        color=Color(127, 127, 127, 255),
        joint_radius=0.3,
        joint_color=Color(200, 200, 200, 255),
    )
    arm1.start_drawing()

    running = True
    spf_running_mean = 0
    coef = 0
    i_point = 0 
    i_shape = 0
    drawing1 = get_drawing1()
    drawing2 = get_drawing2()
    drawing3 = get_drawing3()
    try:
        while running:
            # Check the event queue
            for event in pygame.event.get():
                if event.type == QUIT or (event.type == KEYDOWN and event.key == K_ESCAPE):
                    # The user closed the window or pressed escape
                    running = False

            ms_start = time.perf_counter_ns() / 1000
            ctx.screen.fill((0, 0, 0, 0))

            drawing = drawing3  # drawing1, drawing2, drawing3
            if i_shape < len(drawing):
                pt = drawing[i_shape][i_point]
                arm1.draw(pt)
                if (i_point + 1) >= len(drawing[i_shape]):
                    i_shape += 1
                    i_point = 0
                    arm1.stop_drawing()
                    arm1.start_drawing()
                else:
                    i_point += 1

            arm1.render()

            # Make Box2D simulate the physics of our world for one step.
            ctx.world.Step(TIME_STEP, 10, 10)
            
            spf = time.perf_counter_ns() / 1000 - ms_start
            spf_running_mean = spf_running_mean * coef + (1 - coef) * spf
            coef = 0.99
            print(f"fps={1 / spf_running_mean * 1000 * 1000:.1f} [{spf_running_mean / 1000:.3f}ms]")

            # Flip the screen and try to keep at the target FPS
            pygame.display.flip()
            ctx.clock.tick(TARGET_FPS)
    except KeyboardInterrupt:
        print("Keyboard interrupt. Terminating...")
    pygame.quit()