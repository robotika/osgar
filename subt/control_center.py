#################################
#
#   SubT Urban - Control Center
#
# inspiration:
#   https://stackoverflow.com/questions/40336960/creating-rect-buttons-with-pygame
#

import pygame
import sys

from osgar.node import Node
from osgar.bus import BusShutdownException


class ControlCenter(Node):
    def __init__(self, config, bus):
        super().__init__(config, bus)
        self.verbose = False

    def run(self):
        pygame.init()
        pygame.font.init() # you have to call this at the start, 
                           # if you want to use this module.
        myfont = pygame.font.SysFont('Comic Sans MS', 30)
        clock = pygame.time.Clock()
        fps = 60
        size = [500, 200]
        bg = [255, 255, 255]

        screen = pygame.display.set_mode(size)

        button = pygame.Rect(100, 100, 250, 50)  # creates a rect object
        # The rect method is similar to a list but with a few added perks
        # for example if you want the position of the button you can simpy type
        # button.x or button.y or if you want size you can type button.width or
        # height. you can also get the top, left, right and bottom of an object
        # with button.right, left, top, and bottom

        textsurface = myfont.render('Robotika Control Center ;-)', False, (0, 0, 0))
        stop_textsurface = myfont.render('STOP Kloubak K3', False, (0, 0, 0))

        while True:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    return False

                if event.type == pygame.MOUSEBUTTONDOWN:
                    mouse_pos = event.pos  # gets mouse position

                    # checks if mouse position is over the button

                    if button.collidepoint(mouse_pos):
                        # prints current location of mouse
                        print('button was pressed at {0}'.format(mouse_pos))
                        self.publish('cmd', b'3:Stop')

            screen.fill(bg)

            pygame.draw.rect(screen, [255, 0, 0], button)  # draw button
            screen.blit(textsurface,(0,0))
            screen.blit(stop_textsurface,(100, 100))

            pygame.display.update()
            clock.tick(fps)

        pygame.quit()


def main():
    n = ControlCenter(config={}, bus=None)
    n.start()
    n.join()


if __name__ == '__main__':
    main()

# vim: expandtab sw=4 ts=4

