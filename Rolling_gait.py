import math
import numpy as np
from IK import IK


class Rolling:
    """
    str - 0-0.39, dir - 0-3.14
    front(back)-blue, right-yellow, left-green, top-red
    """

    def __init__(self, head_bend: float = 0.13, tail_bend: float = 0.13, l: float = 0.21):
        self.L = l
        self.action = [
            [0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0],
            [0.35, -1.57, 0.35, 0, 0.35, 1.57, 0.35, 1],
            [0.35, -1.57, 0.35, 0, 0.35, 1.57, 0.35, 1],
            [0.35, -1.57, 0.35, 0, 0.35, 1.57, 0.35, 1],
            [0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0]
        ]
        self.neutral = [
            [0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0]
        ]
        self.phi_left = []
        self.phi_right = []
        self.theta_left = []
        self.theta_right = []
        self.phi_head = []
        self.phi_tail = []
        self.theta_head = []
        self.theta_tail = []
        self.x = [0, 0, 0, 0, 0, 0, 0]
        self.y_pos = [0, 0, 0, 0, 0, 0, 0]
        self.y_neg = [0, 0, 0, 0, 0, 0, 0]
        self.head_bend = head_bend
        self.tail_bend = tail_bend

        self.ik_r = IK()
        self.ik_g = IK()
        self.ik_b = IK()
        self.ik_y = IK()
        self.ik_circle = IK()

    def red_top(self, back: str) -> None:

        if back == "blue":
            """
            head: r, tail: b, left: y, right: g
            """

            self.left_right('y', 'g', 0, 0)
            self.head_tail('r', 'b')

            b_phi = self.phi_tail
            b_theta = self.theta_tail
            y_phi = self.phi_left
            y_theta = self.theta_left
            g_phi = self.phi_right
            g_theta = self.theta_right
            r_phi = self.phi_head
            r_theta = self.theta_head

            self.action = [
                [0, 0, 0, 0, 0, 0, 0, 0],
                [b_phi[0], b_theta[0], y_phi[0], y_theta[0], g_phi[0], g_theta[0], r_phi[0], r_theta[0]],
                [b_phi[1], b_theta[1], y_phi[1], y_theta[1], g_phi[1], g_theta[1], r_phi[1], r_theta[1]],
                [b_phi[2], b_theta[2], y_phi[2], y_theta[2], g_phi[2], g_theta[2], r_phi[2], r_theta[2]],
                [b_phi[3], b_theta[3], y_phi[3], y_theta[3], g_phi[3], g_theta[3], r_phi[3], r_theta[3]],
                [b_phi[4], b_theta[4], y_phi[4], y_theta[4], g_phi[4], g_theta[4], r_phi[4], r_theta[4]],
                [b_phi[5], b_theta[5], y_phi[5], y_theta[5], g_phi[5], g_theta[5], r_phi[5], r_theta[5]],
                [b_phi[6], b_theta[6], y_phi[6], y_theta[6], g_phi[6], g_theta[6], r_phi[6], r_theta[6]],
                [0, 0, 0, 0, 0, 0, 0, 0]
            ]

        elif back == "yellow":
            """
            head: r, tail: y, left: g, right: b
            """
            self.left_right('g', 'b', 0, 0)
            self.head_tail('r', 'b', rot_head=-math.pi*2/3)

            y_phi = self.phi_tail
            y_theta = self.theta_tail
            g_phi = self.phi_left
            g_theta = self.theta_left
            b_phi = self.phi_right
            b_theta = self.theta_right
            r_phi = self.phi_head
            r_theta = self.theta_head


            self.action = [
                [0, 0, 0, 0, 0, 0, 0, 0],
                [b_phi[0], b_theta[0], y_phi[0], y_theta[0], g_phi[0], g_theta[0], r_phi[0], r_theta[0]],
                [b_phi[1], b_theta[1], y_phi[1], y_theta[1], g_phi[1], g_theta[1], r_phi[1], r_theta[1]],
                [b_phi[2], b_theta[2], y_phi[2], y_theta[2], g_phi[2], g_theta[2], r_phi[2], r_theta[2]],
                [b_phi[3], b_theta[3], y_phi[3], y_theta[3], g_phi[3], g_theta[3], r_phi[3], r_theta[3]],
                [b_phi[4], b_theta[4], y_phi[4], y_theta[4], g_phi[4], g_theta[4], r_phi[4], r_theta[4]],
                [b_phi[5], b_theta[5], y_phi[5], y_theta[5], g_phi[5], g_theta[5], r_phi[5], r_theta[5]],
                [b_phi[6], b_theta[6], y_phi[6], y_theta[6], g_phi[6], g_theta[6], r_phi[6], r_theta[6]],
                [0, 0, 0, 0, 0, 0, 0, 0]
            ]

        elif back == "green":
            """
            head: r, tail: g, left: b, right: y
            """
            self.left_right('g', 'b', 0, 0)
            self.head_tail('r', 'b', rot_head=math.pi * 2 / 3)

            g_phi = self.phi_tail
            g_theta = self.theta_tail
            b_phi = self.phi_left
            b_theta = self.theta_left
            y_phi = self.phi_right
            y_theta = self.theta_right
            r_phi = self.phi_head
            r_theta = self.theta_head


            self.action = [
                [0, 0, 0, 0, 0, 0, 0, 0],
                [b_phi[0], b_theta[0], y_phi[0], y_theta[0], g_phi[0], g_theta[0], r_phi[0], r_theta[0]],
                [b_phi[1], b_theta[1], y_phi[1], y_theta[1], g_phi[1], g_theta[1], r_phi[1], r_theta[1]],
                [b_phi[2], b_theta[2], y_phi[2], y_theta[2], g_phi[2], g_theta[2], r_phi[2], r_theta[2]],
                [b_phi[3], b_theta[3], y_phi[3], y_theta[3], g_phi[3], g_theta[3], r_phi[3], r_theta[3]],
                [b_phi[4], b_theta[4], y_phi[4], y_theta[4], g_phi[4], g_theta[4], r_phi[4], r_theta[4]],
                [b_phi[5], b_theta[5], y_phi[5], y_theta[5], g_phi[5], g_theta[5], r_phi[5], r_theta[5]],
                [b_phi[6], b_theta[6], y_phi[6], y_theta[6], g_phi[6], g_theta[6], r_phi[6], r_theta[6]],
                [0, 0, 0, 0, 0, 0, 0, 0]
            ]

    def green_top(self, back: str) -> None:

        """
        The rotation angles to transform for standard direction
        red: -math.pi/3  |   green: math.pi (red)   |   yellow: math.pi*2/3   |   blue: -math.pi*2/3
        """

        if back == "blue":

            """
             the initial angle is for rotating the cof to standard direction. Then add the other rotations
            """

            """
            head: g, tail: b, left: r, right: y
            """

            self.left_right('r', 'y', rot_left=-math.pi / 3, rot_right=math.pi * 2 / 3)
            self.head_tail('g', 'b', rot_head=math.pi + math.pi*2/3, rot_tail=-math.pi * 2 / 3)

            b_phi = self.phi_tail
            b_theta = self.theta_tail
            r_phi = self.phi_left
            r_theta = self.theta_left
            y_phi = self.phi_right
            y_theta = self.theta_right
            g_phi = self.phi_head
            g_theta = self.theta_head

            self.action = [
                [0, 0, 0, 0, 0, 0, 0, 0],
                [b_phi[0], b_theta[0], y_phi[0], y_theta[0], g_phi[0], g_theta[0], r_phi[0], r_theta[0]],
                [b_phi[1], b_theta[1], y_phi[1], y_theta[1], g_phi[1], g_theta[1], r_phi[1], r_theta[1]],
                [b_phi[2], b_theta[2], y_phi[2], y_theta[2], g_phi[2], g_theta[2], r_phi[2], r_theta[2]],
                [b_phi[3], b_theta[3], y_phi[3], y_theta[3], g_phi[3], g_theta[3], r_phi[3], r_theta[3]],
                [b_phi[4], b_theta[4], y_phi[4], y_theta[4], g_phi[4], g_theta[4], r_phi[4], r_theta[4]],
                [b_phi[5], b_theta[5], y_phi[5], y_theta[5], g_phi[5], g_theta[5], r_phi[5], r_theta[5]],
                [b_phi[6], b_theta[6], y_phi[6], y_theta[6], g_phi[6], g_theta[6], r_phi[6], r_theta[6]],
                [0, 0, 0, 0, 0, 0, 0, 0]
            ]

        elif back == "yellow":

            """
            head: g, tail: y, left: b, right: r
            """

            self.left_right('b', 'r', rot_left=-math.pi * 2 / 3, rot_right=-math.pi / 3)
            self.head_tail('g', 'y', rot_head=math.pi - math.pi*2/3, rot_tail=math.pi * 2 / 3)

            y_phi = self.phi_tail
            y_theta = self.theta_tail
            b_phi = self.phi_left
            b_theta = self.theta_left
            r_phi = self.phi_right
            r_theta = self.theta_right
            g_phi = self.phi_head
            g_theta = self.theta_head

            self.action = [
                [0, 0, 0, 0, 0, 0, 0, 0],
                [b_phi[0], b_theta[0], y_phi[0], y_theta[0], g_phi[0], g_theta[0], r_phi[0], r_theta[0]],
                [b_phi[1], b_theta[1], y_phi[1], y_theta[1], g_phi[1], g_theta[1], r_phi[1], r_theta[1]],
                [b_phi[2], b_theta[2], y_phi[2], y_theta[2], g_phi[2], g_theta[2], r_phi[2], r_theta[2]],
                [b_phi[3], b_theta[3], y_phi[3], y_theta[3], g_phi[3], g_theta[3], r_phi[3], r_theta[3]],
                [b_phi[4], b_theta[4], y_phi[4], y_theta[4], g_phi[4], g_theta[4], r_phi[4], r_theta[4]],
                [b_phi[5], b_theta[5], y_phi[5], y_theta[5], g_phi[5], g_theta[5], r_phi[5], r_theta[5]],
                [b_phi[6], b_theta[6], y_phi[6], y_theta[6], g_phi[6], g_theta[6], r_phi[6], r_theta[6]],
                [0, 0, 0, 0, 0, 0, 0, 0]
            ]

        elif back == "red":

            """
            head: g, tail: r, left: y, right: b
            """

            self.left_right('y', 'b', rot_left=math.pi * 2 / 3, rot_right=-math.pi * 2 / 3)
            self.head_tail('g', 'r', rot_head=math.pi, rot_tail=-math.pi / 3)

            r_phi = self.phi_tail
            r_theta = self.theta_tail
            y_phi = self.phi_left
            y_theta = self.theta_left
            b_phi = self.phi_right
            b_theta = self.theta_right
            g_phi = self.phi_head
            g_theta = self.theta_head

            self.action = [
                [0, 0, 0, 0, 0, 0, 0, 0],
                [b_phi[0], b_theta[0], y_phi[0], y_theta[0], g_phi[0], g_theta[0], r_phi[0], r_theta[0]],
                [b_phi[1], b_theta[1], y_phi[1], y_theta[1], g_phi[1], g_theta[1], r_phi[1], r_theta[1]],
                [b_phi[2], b_theta[2], y_phi[2], y_theta[2], g_phi[2], g_theta[2], r_phi[2], r_theta[2]],
                [b_phi[3], b_theta[3], y_phi[3], y_theta[3], g_phi[3], g_theta[3], r_phi[3], r_theta[3]],
                [b_phi[4], b_theta[4], y_phi[4], y_theta[4], g_phi[4], g_theta[4], r_phi[4], r_theta[4]],
                [b_phi[5], b_theta[5], y_phi[5], y_theta[5], g_phi[5], g_theta[5], r_phi[5], r_theta[5]],
                [b_phi[6], b_theta[6], y_phi[6], y_theta[6], g_phi[6], g_theta[6], r_phi[6], r_theta[6]],
                [0, 0, 0, 0, 0, 0, 0, 0]
            ]

    def blue_top(self, back: str) -> None:

        """
        The rotation angles to transform for standard direction
        red: math.pi  |   green: math.pi*2/3   |   yellow: -math.pi*2/3   |   blue: math.pi/3 (Green)
        """

        if back == "green":

            """
             the initial angle is for rotating the cof to standard direction. Then add the other rotations
            """

            """
            head: b, tail: g, left: y, right: r
            """

            self.left_right('y', 'r', rot_left=-math.pi * 2 / 3, rot_right=math.pi)
            self.head_tail('b', 'g', rot_head=math.pi / 3, rot_tail=math.pi * 2 / 3)

            g_phi = self.phi_tail
            g_theta = self.theta_tail
            y_phi = self.phi_left
            y_theta = self.theta_left
            r_phi = self.phi_right
            r_theta = self.theta_right
            b_phi = self.phi_head
            b_theta = self.theta_head

            self.action = [
                [0, 0, 0, 0, 0, 0, 0, 0],
                [b_phi[0], b_theta[0], y_phi[0], y_theta[0], g_phi[0], g_theta[0], r_phi[0], r_theta[0]],
                [b_phi[1], b_theta[1], y_phi[1], y_theta[1], g_phi[1], g_theta[1], r_phi[1], r_theta[1]],
                [b_phi[2], b_theta[2], y_phi[2], y_theta[2], g_phi[2], g_theta[2], r_phi[2], r_theta[2]],
                [b_phi[3], b_theta[3], y_phi[3], y_theta[3], g_phi[3], g_theta[3], r_phi[3], r_theta[3]],
                [b_phi[4], b_theta[4], y_phi[4], y_theta[4], g_phi[4], g_theta[4], r_phi[4], r_theta[4]],
                [b_phi[5], b_theta[5], y_phi[5], y_theta[5], g_phi[5], g_theta[5], r_phi[5], r_theta[5]],
                [b_phi[6], b_theta[6], y_phi[6], y_theta[6], g_phi[6], g_theta[6], r_phi[6], r_theta[6]],
                [0, 0, 0, 0, 0, 0, 0, 0]
            ]

        elif back == "yellow":

            """
            head: b, tail: g, left: y, right: r
            """

            self.left_right('r', 'g', rot_left=math.pi, rot_right=math.pi * 2 / 3)
            self.head_tail('b', 'y', rot_head=math.pi / 3 - math.pi*2/3, rot_tail=-math.pi * 2 / 3)

            y_phi = self.phi_tail
            y_theta = self.theta_tail
            r_phi = self.phi_left
            r_theta = self.theta_left
            g_phi = self.phi_right
            g_theta = self.theta_right
            b_phi = self.phi_head
            b_theta = self.theta_head

            self.action = [
                [0, 0, 0, 0, 0, 0, 0, 0],
                [b_phi[0], b_theta[0], y_phi[0], y_theta[0], g_phi[0], g_theta[0], r_phi[0], r_theta[0]],
                [b_phi[1], b_theta[1], y_phi[1], y_theta[1], g_phi[1], g_theta[1], r_phi[1], r_theta[1]],
                [b_phi[2], b_theta[2], y_phi[2], y_theta[2], g_phi[2], g_theta[2], r_phi[2], r_theta[2]],
                [b_phi[3], b_theta[3], y_phi[3], y_theta[3], g_phi[3], g_theta[3], r_phi[3], r_theta[3]],
                [b_phi[4], b_theta[4], y_phi[4], y_theta[4], g_phi[4], g_theta[4], r_phi[4], r_theta[4]],
                [b_phi[5], b_theta[5], y_phi[5], y_theta[5], g_phi[5], g_theta[5], r_phi[5], r_theta[5]],
                [b_phi[6], b_theta[6], y_phi[6], y_theta[6], g_phi[6], g_theta[6], r_phi[6], r_theta[6]],
                [0, 0, 0, 0, 0, 0, 0, 0]
            ]

        elif back == "red":

            """
            head: b, tail: r, left: g, right: y
            """

            self.left_right('g', 'y', rot_left=math.pi * 2 / 3, rot_right=-math.pi * 2 / 3)
            self.head_tail('b', 'r', rot_head=math.pi / 3 + math.pi * 2 / 3, rot_tail=math.pi)

            r_phi = self.phi_tail
            r_theta = self.theta_tail
            g_phi = self.phi_left
            g_theta = self.theta_left
            y_phi = self.phi_right
            y_theta = self.theta_right
            b_phi = self.phi_head
            b_theta = self.theta_head

            self.action = [
                [0, 0, 0, 0, 0, 0, 0, 0],
                [b_phi[0], b_theta[0], y_phi[0], y_theta[0], g_phi[0], g_theta[0], r_phi[0], r_theta[0]],
                [b_phi[1], b_theta[1], y_phi[1], y_theta[1], g_phi[1], g_theta[1], r_phi[1], r_theta[1]],
                [b_phi[2], b_theta[2], y_phi[2], y_theta[2], g_phi[2], g_theta[2], r_phi[2], r_theta[2]],
                [b_phi[3], b_theta[3], y_phi[3], y_theta[3], g_phi[3], g_theta[3], r_phi[3], r_theta[3]],
                [b_phi[4], b_theta[4], y_phi[4], y_theta[4], g_phi[4], g_theta[4], r_phi[4], r_theta[4]],
                [b_phi[5], b_theta[5], y_phi[5], y_theta[5], g_phi[5], g_theta[5], r_phi[5], r_theta[5]],
                [b_phi[6], b_theta[6], y_phi[6], y_theta[6], g_phi[6], g_theta[6], r_phi[6], r_theta[6]],
                [0, 0, 0, 0, 0, 0, 0, 0]
            ]

    def yellow_top(self, back: str) -> None:

        """
        The rotation angles to transform for standard direction

        red: math.pi/3  |   green: -math.pi*2/3   |   yellow: math.pi (red)   |   blue: math.pi*2/3
        """

        if back == "red":

            """
             the initial angle is for rotating the cof to standard direction. Then add the other rotations
            """

            """
            head: y, tail: r, left: b, right: g
            """

            self.left_right('b', 'g', rot_left=math.pi * 2 / 3, rot_right=-math.pi * 2 / 3)
            self.head_tail('y', 'r', rot_head=math.pi, rot_tail=math.pi / 3)

            r_phi = self.phi_tail
            r_theta = self.theta_tail
            b_phi = self.phi_left
            b_theta = self.theta_left
            g_phi = self.phi_right
            g_theta = self.theta_right
            y_phi = self.phi_head
            y_theta = self.theta_head

            self.action = [
                [0, 0, 0, 0, 0, 0, 0, 0],
                [b_phi[0], b_theta[0], y_phi[0], y_theta[0], g_phi[0], g_theta[0], r_phi[0], r_theta[0]],
                [b_phi[1], b_theta[1], y_phi[1], y_theta[1], g_phi[1], g_theta[1], r_phi[1], r_theta[1]],
                [b_phi[2], b_theta[2], y_phi[2], y_theta[2], g_phi[2], g_theta[2], r_phi[2], r_theta[2]],
                [b_phi[3], b_theta[3], y_phi[3], y_theta[3], g_phi[3], g_theta[3], r_phi[3], r_theta[3]],
                [b_phi[4], b_theta[4], y_phi[4], y_theta[4], g_phi[4], g_theta[4], r_phi[4], r_theta[4]],
                [b_phi[5], b_theta[5], y_phi[5], y_theta[5], g_phi[5], g_theta[5], r_phi[5], r_theta[5]],
                [b_phi[6], b_theta[6], y_phi[6], y_theta[6], g_phi[6], g_theta[6], r_phi[6], r_theta[6]],
                [0, 0, 0, 0, 0, 0, 0, 0]
            ]

        elif back == "blue":

            """
             head: y, tail: b, left: g, right: r
            """

            self.left_right('g', 'r', rot_left=-math.pi * 2 / 3, rot_right=math.pi / 3)
            self.head_tail('y', 'b', rot_head=math.pi - math.pi * 2 / 3, rot_tail=math.pi * 2 / 3)

            b_phi = self.phi_tail
            b_theta = self.theta_tail
            g_phi = self.phi_left
            g_theta = self.theta_left
            r_phi = self.phi_right
            r_theta = self.theta_right
            y_phi = self.phi_head
            y_theta = self.theta_head

            self.action = [
                [0, 0, 0, 0, 0, 0, 0, 0],
                [b_phi[0], b_theta[0], y_phi[0], y_theta[0], g_phi[0], g_theta[0], r_phi[0], r_theta[0]],
                [b_phi[1], b_theta[1], y_phi[1], y_theta[1], g_phi[1], g_theta[1], r_phi[1], r_theta[1]],
                [b_phi[2], b_theta[2], y_phi[2], y_theta[2], g_phi[2], g_theta[2], r_phi[2], r_theta[2]],
                [b_phi[3], b_theta[3], y_phi[3], y_theta[3], g_phi[3], g_theta[3], r_phi[3], r_theta[3]],
                [b_phi[4], b_theta[4], y_phi[4], y_theta[4], g_phi[4], g_theta[4], r_phi[4], r_theta[4]],
                [b_phi[5], b_theta[5], y_phi[5], y_theta[5], g_phi[5], g_theta[5], r_phi[5], r_theta[5]],
                [b_phi[6], b_theta[6], y_phi[6], y_theta[6], g_phi[6], g_theta[6], r_phi[6], r_theta[6]],
                [0, 0, 0, 0, 0, 0, 0, 0]
            ]

        elif back == "green":

            """
            head: y, tail: g, left: r, right: b
            """

            self.left_right('r', 'b', rot_left=math.pi / 3, rot_right=math.pi * 2 / 3)
            self.head_tail('y', 'g', rot_head=math.pi + math.pi * 2 / 3, rot_tail=-math.pi * 2 / 3)

            g_phi = self.phi_tail
            g_theta = self.theta_tail
            r_phi = self.phi_left
            r_theta = self.theta_left
            b_phi = self.phi_right
            b_theta = self.theta_right
            y_phi = self.phi_head
            y_theta = self.theta_head

            self.action = [
                [0, 0, 0, 0, 0, 0, 0, 0],
                [b_phi[0], b_theta[0], y_phi[0], y_theta[0], g_phi[0], g_theta[0], r_phi[0], r_theta[0]],
                [b_phi[1], b_theta[1], y_phi[1], y_theta[1], g_phi[1], g_theta[1], r_phi[1], r_theta[1]],
                [b_phi[2], b_theta[2], y_phi[2], y_theta[2], g_phi[2], g_theta[2], r_phi[2], r_theta[2]],
                [b_phi[3], b_theta[3], y_phi[3], y_theta[3], g_phi[3], g_theta[3], r_phi[3], r_theta[3]],
                [b_phi[4], b_theta[4], y_phi[4], y_theta[4], g_phi[4], g_theta[4], r_phi[4], r_theta[4]],
                [b_phi[5], b_theta[5], y_phi[5], y_theta[5], g_phi[5], g_theta[5], r_phi[5], r_theta[5]],
                [b_phi[6], b_theta[6], y_phi[6], y_theta[6], g_phi[6], g_theta[6], r_phi[6], r_theta[6]],
                [0, 0, 0, 0, 0, 0, 0, 0]
            ]

    def half_circle_coordinates(self, radius: float) -> None:
        t, p = self.ik_circle.angles(radius, 0.0)
        # angle_range = np.arange(-np.pi / 2, np.pi / 2 + np.pi / 6, np.pi / 6)
        angle_range = np.arange(0, np.pi + np.pi / 6, np.pi / 6)
        count: int = 0
        for i in angle_range:
            self.x[count] = round((self.L / p) * (math.cos(i) * (1 - math.cos(p))), 4)
            self.y_pos[count] = round((self.L / p) * (math.sin(i) * (1 - math.cos(p))), 4)
            self.y_neg[count] = round((-self.L / p) * (math.sin(i) * (1 - math.cos(p))), 4)
            count = count + 1

    def head_tail(self, head: str, tail: str, rot_head: float = 0.0, rot_tail: float = 0.0) -> None:

        phi_head = [0, 0, 0, 0, 0, 0, 0]
        phi_tail = [0, 0, 0, 0, 0, 0, 0]
        theta_head = [0, 0, 0, 0, 0, 0, 0]
        theta_tail = [0, 0, 0, 0, 0, 0, 0]

        self.phi_head = phi_head
        self.phi_tail = phi_tail
        self.theta_head = theta_head
        self.theta_tail = theta_tail

        hl, hh = 1, 4
        xh = -0.14
        if head == 'g':
            self.ik_g.angles(xh, 0, rot_head)
            self.phi_head[hl:hh] = [self.ik_g.phi/8 for i in range(0, (hh-hl))]
            self.theta_head[hl:hh] = [self.ik_g.theta for i in range(0, (hh-hl))]
        elif head == 'b':
            self.ik_b.angles(xh, 0, rot_head)
            self.phi_head[hl:hh] = [self.ik_b.phi/8 for i in range(0, (hh-hl))]
            self.theta_head[hl:hh] = [self.ik_b.theta for i in range(0, (hh-hl))]
        elif head == 'r':
            self.ik_r.angles(xh, 0, rot_head)
            self.phi_head[hl:hh] = [self.ik_r.phi/8 for i in range(0, (hh-hl))]
            self.theta_head[hl:hh] = [self.ik_r.theta for i in range(0, (hh-hl))]
        elif head == 'y':
            self.ik_y.angles(xh, 0, rot_head)
            self.phi_head[hl:hh] = [self.ik_y.phi/8 for i in range(0, (hh-hl))]
            self.theta_head[hl:hh] = [self.ik_y.theta for i in range(0, (hh-hl))]

        # tail_theta_low
        fl, fh, sl, sh = 3, 6, 6, 7
        xt = 0.1
        push = -0.1
        if tail == 'g':
            self.ik_g.angles(xt, 0, rot_tail)
            self.phi_tail[fl:fh] = [self.ik_g.phi/8 for i in range(0, (fh-fl))]
            self.theta_tail[fl:fh] = [self.ik_g.theta for i in range(0, (fh-fl))]
            self.ik_g.angles(push, 0, rot_tail)
            self.phi_tail[sl:sh] = [self.ik_g.phi/8 for i in range(0, (sh-sl))]
            self.theta_tail[sl:sh] = [self.ik_g.theta for i in range(0, (sh-sl))]
        elif tail == 'b':
            self.ik_b.angles(xt, 0, rot_tail)
            self.phi_tail[fl:fh] = [self.ik_b.phi/8 for i in range(0, (fh-fl))]
            self.theta_tail[fl:fh] = [self.ik_b.theta for i in range(0, (fh-fl))]
            self.ik_b.angles(push, 0, rot_tail)
            self.phi_tail[sl:sh] = [self.ik_b.phi/8 for i in range(0, (sh-sl))]
            self.theta_tail[sl:sh] = [self.ik_b.theta for i in range(0, (sh-sl))]
        elif tail == 'r':
            self.ik_r.angles(xt, 0, rot_tail)
            self.phi_tail[fl:fh] = [self.ik_r.phi/8 for i in range(0, (fh-fl))]
            self.theta_tail[fl:fh] = [self.ik_r.theta for i in range(0, (fh-fl))]
            self.ik_r.angles(push, 0, rot_tail)
            self.phi_tail[sl:sh] = [self.ik_r.phi/8 for i in range(0, (sh-sl))]
            self.theta_tail[sl:sh] = [self.ik_r.theta for i in range(0, (sh-sl))]
        elif tail == 'y':
            self.ik_y.angles(xt, 0, rot_tail)
            self.phi_tail[fl:fh] = [self.ik_y.phi/8 for i in range(0, (fh-fl))]
            self.theta_tail[fl:fh] = [self.ik_y.theta for i in range(0, (fh-fl))]
            self.ik_y.angles(push, 0, rot_tail)
            self.phi_tail[sl:sh] = [self.ik_y.phi/8 for i in range(0, (sh-sl))]
            self.theta_tail[sl:sh] = [self.ik_y.theta for i in range(0, (sh-sl))]

    def left_right(self, left: str, right: str, rot_left: float = 0.0, rot_right: float = 0.0) -> None:

        phi_left = [0, 0, 0, 0, 0, 0, 0]
        phi_right = [0, 0, 0, 0, 0, 0, 0]
        theta_left = [0, 0, 0, 0, 0, 0, 0]
        theta_right = [0, 0, 0, 0, 0, 0, 0]

        self.phi_left = phi_left
        self.phi_right = phi_right
        self.theta_left = theta_left
        self.theta_right = theta_right

        for i in range(0, len(self.x) - 2):
            phi = 0.0
            if left == 'g':
                self.theta_left[i], phi = self.ik_g.angles(self.x[i], self.y_pos[i], rot_left)
                self.phi_left[i] = phi / 8
            elif left == 'b':
                self.theta_left[i], phi = self.ik_b.angles(self.x[i], self.y_pos[i], rot_left)
                self.phi_left[i] = phi / 8
            elif left == 'r':
                self.theta_left[i], phi = self.ik_r.angles(self.x[i], self.y_pos[i], rot_left)
                self.phi_left[i] = phi / 8
            elif left == 'y':
                self.theta_left[i], phi = self.ik_y.angles(self.x[i], self.y_pos[i], rot_left)
                self.phi_left[i] = phi / 8

            if right == 'g':
                self.theta_right[i], phi = self.ik_g.angles(self.x[i], self.y_neg[i], rot_right)
                self.phi_right[i] = phi / 8
            elif right == 'b':
                self.theta_right[i], phi = self.ik_b.angles(self.x[i], self.y_neg[i], rot_right)
                self.phi_right[i] = phi / 8
            elif right == 'r':
                self.theta_right[i], phi = self.ik_r.angles(self.x[i], self.y_neg[i], rot_right)
                self.phi_right[i] = phi / 8
            elif right == 'y':
                self.theta_right[i], phi = self.ik_y.angles(self.x[i], self.y_neg[i], rot_right)
                self.phi_right[i] = phi / 8


# r = Rolling()
# r.half_circle_coordinates(0.1)
