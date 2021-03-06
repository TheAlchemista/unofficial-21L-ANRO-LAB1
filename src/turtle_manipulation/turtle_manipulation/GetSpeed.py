# Sets speed of the turtle according to user input.


# Python imports:
import curses

# ROS2 imports:
from geometry_msgs.msg import Twist

# Program imports:
from turtle_manipulation.Constants import Constants


class GetSpeed:

    def __init__(self, params):
        self.speed = Twist()
        self.constants = Constants()
        self.screen = curses.initscr()
        self.setup(params)

    def setup(self, params):
        curses.cbreak()
        curses.noecho()
        self.screen.keypad(1)
        self.screen.addstr(0, 100, "Hit 'q' to quit")
        self.print_steering_info(params)
        self.screen.refresh()

    def filter_and_set_keys(self, params):
        self.reset_speed()
        loop = True
        break_loop = False
        while loop:
            loop = False
            key = self.screen.getch()
            if key == ord(params['up']):
                self.speed.linear.x = self.constants.L_SCALE * 1.0
            elif key == ord(params['down']):
                self.speed.linear.x = self.constants.L_SCALE * -1.0
            elif key == ord(params['left']):
                self.speed.angular.z = self.constants.A_SCALE * 1.0
            elif key == ord(params['right']):
                self.speed.angular.z = self.constants.A_SCALE * -1.0
            elif key == ord('q'):
                break_loop = True
            else:
                loop = True

        return break_loop

    def print_speed(self, speed):
        self.screen.addstr(0, 0, "Speed value:")
        self.screen.addstr(1, 1, "Linear:")
        self.screen.addstr(2, 4, "x = " + str(speed.linear.x))
        self.screen.addstr(3, 4, "y = " + str(speed.linear.y))
        self.screen.addstr(4, 4, "z = " + str(speed.linear.z))
        self.screen.addstr(5, 1, "Angular:")
        self.screen.addstr(6, 4, "x = " + str(speed.angular.x))
        self.screen.addstr(7, 4, "y = " + str(speed.angular.y))
        self.screen.addstr(8, 4, "z = " + str(speed.angular.z))
        self.screen.refresh()

    def print_steering_info(self, params):
        X_LAYOUT = 50
        self.screen.addstr(0, X_LAYOUT, "Steering keys:")
        self.screen.addstr(1, X_LAYOUT, "Up:" + params['up'])
        self.screen.addstr(2, X_LAYOUT, "Down:" + params['down'])
        self.screen.addstr(3, X_LAYOUT, "Left:" + params['left'])
        self.screen.addstr(4, X_LAYOUT, "Right:" + params['right'])
        self.screen.refresh()

    # Closes curses
    def end(self):
        curses.endwin()

    # Resets the speed. Otherwise turtle would never stop
    def reset_speed(self):
        self.speed.linear.x = 0.0
        self.speed.linear.y = 0.0
        self.speed.linear.z = 0.0
        self.speed.angular.x = 0.0
        self.speed.angular.y = 0.0
        self.speed.angular.z = 0.0

    # Getter
    def getSpeed(self):
        return self.speed
