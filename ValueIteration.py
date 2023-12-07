import sys
import copy
import time


class ValueIteration:

    def __init__(self, env: object, gait: object, H: int, start: list, goal: list) -> None:
        """

        :param env: TetraSim class
        :param gait: Rolling class
        :param H: Horizon
        :param start: [X, Y, orientationIndx]
        :param goal: [X, Y, orientationIndx]
        """
        self.heads = env.actions
        self.env = env
        self.gait = gait
        self.actionlist = None
        self.Vpre = {}
        self.Vpost = {}
        self.policy = {}
        self.stepPolicy = {}  # policy for each step.
        self.maxIte = H  # compute the t steps need
        self.start = start
        self.goal = goal
        self.__populateActions()
        self.__nValueFunct()

    def vi(self):
        """
        value iteration algorithm
        :return:
        """

        #  final step value function. set goal value to zero
        self.Vpre[self.goal[0]][self.goal[1]][self.goal[2]] = 0
        iteration = 0
        for ite in range(self.maxIte, 0, -1):
            # print("\r" + str(ite), end='')
            Vpost, policy = self.__newValueFunct()
            for x in self.env.x_states:
                for y in self.env.y_states:
                    count = 0
                    for o in self.env.o_states:
                        print("\r" + str(iteration) + " * " + str(ite) + " -  [" + str(x) + ", " + str(y) + "] - " + str(count), end='')
                        a = None
                        max = float(sys.maxsize)
                        self.env.setEnv(pos=[x, y, 0.5], ori=o)
                        head = self.env.getHead()
                        tails = self.actionlist[head]
                        for tail in tails:
                            self.env.setEnv(pos=[x, y, 0.5], ori=o)
                            while not self.env.is_actionDone():
                                act = [0, 0, 0, 0, 0, 0, 0, 0]
                                t = 0
                                while t < 250:
                                    self.env.step(act)
                                    time.sleep(1. / 24000)
                                    t += 1

                            self.DoAction(head=head, tail=tail)
                            # while not self.env.is_actionDone():
                            #     pass
                            s_next, cost = self.nstate_cost()
                            if not self.isvalid(s_next):
                                print("\r" + "Invalid", end='')
                                continue
                            v_temp = cost + self.Vpre[s_next[0]][s_next[1]][s_next[2]]
                            if max > v_temp:
                                max = v_temp
                                a = tail
                            iteration += 1
                        self.policy[x][y][count] = [head, a]
                        policy[x][y][count] = [head, a]
                        Vpost[x][y][count] = max
                        count += 1
            self.stepPolicy[ite] = copy.deepcopy(policy)
            self.Vpre = copy.deepcopy(Vpost)
        self.env.endSimulation()

    def DoAction(self, head: str, tail: str) -> None:
        """
        Execute the action
        :param head: 'r', 'b', 'g', 'y'
        :param tail: 'r', 'b', 'g', 'y'
        :return: None
        """
        if head == 'r':
            if tail == 'b':
                self.gait.red_top("blue")
            elif tail == 'g':
                self.gait.red_top("green")
            elif tail == 'y':
                self.gait.red_top("yellow")
        elif head == 'b':
            if tail == 'r':
                self.gait.blue_top("red")
            elif tail == 'g':
                self.gait.blue_top("green")
            elif tail == 'y':
                self.gait.blue_top("yellow")
        elif head == 'g':
            if tail == 'r':
                self.gait.green_top("red")
            elif tail == 'b':
                self.gait.green_top("blue")
            elif tail == 'y':
                self.gait.green_top("yellow")
        elif head == 'y':
            if tail == 'r':
                self.gait.yellow_top("red")
            elif tail == 'b':
                self.gait.yellow_top("blue")
            elif tail == 'g':
                self.gait.yellow_top("green")
        action = self.gait.action

        i = 0
        count = 0
        flag_stop = 0
        a = [0, 0, 0, 0, 0, 0, 0, 0]
        while 1:
            if (count > 120) and (count % 36 == 0) and flag_stop == 0:
                a = action[i]
                i += 1

            if i == len(action):
                i = 0
                flag_stop = 1
                break

            self.env.step(a)

            time.sleep(1. / 24000)
            count += 1
        fcount = count
        while count < fcount + 250:
            self.env.step(a)
            time.sleep(1. / 24000)
            count += 1

    def isvalid(self, state: list) -> bool:
        """
        Checking next state validity withing the state space
        :param state: [x, y, o]
        :return: True / False
        """
        x = state[0]
        y = state[1]
        i = state[2]

        flag = 0

        if self.env.xmin > x or x > self.env.xmax:
            flag += 1
        if self.env.ymin > y or y > self.env.ymax:
            flag += 1
        if i < 0:
            flag += 1

        if flag > 0:
            return False
        else:
            return True

    def nstate_cost(self) -> list:
        """
        return next state and the cost associated with.
        In this project there is no cost for movement.
        :return: [state, cost]
        """
        [px, py, pz] = list(self.env.getRobotBasePosition())
        ori = self.env.OrinetationIndex()
        return [[px, py, ori], 0.0]

    def __populateActions(self) -> None:
        """
        generate the available action for each head.
        :return:
        """
        self.actionlist = {'r': ['b', 'g', 'y'],
                           'b': ['r', 'g', 'y'],
                           'g': ['r', 'b', 'y'],
                           'y': ['r', 'b', 'g']}

    def __nValueFunct(self) -> None:
        """
        orientation index;
            0- [pi, 0, -pi / 2]
            1- [-dt, 0.49, -0.87]
            2- [1.18, 0.49, -2.29]
            3- [0, dt, -pi/2]
            4- [0, -dt, pi / 2]
            5- [-1.18, 0.49, 2.29]
            6- [1.18, 0.49, 0.87]
            7- [-pi, 0, pi / 2]

        initialize the dictionaries
        :return:
        """
        for x in self.env.x_states:
            temp_y0 = {}
            temp_yi = {}
            temp_yp = {}
            for y in self.env.y_states:
                temp_o0 = {}
                temp_oi = {}
                temp_op = {}
                count = 0
                for o in self.env.o_states:
                    temp_o0[count] = float("inf")
                    temp_oi[count] = float("inf")
                    temp_op[count] = None
                    count = count + 1

                if y == -0.0:
                    y = 0.0

                temp_y0[y] = temp_o0
                temp_yi[y] = temp_oi
                temp_yp[y] = temp_op
            if x == -0.0:
                x = 0.0

            self.Vpre[x] = temp_y0
            self.Vpost[x] = temp_yi
            self.policy[x] = temp_yp

    def __newValueFunct(self) -> [dict, dict]:
        """
        orientation index;
            0- [pi, 0, -pi / 2]
            1- [-dt, 0.49, -0.87]
            2- [1.18, 0.49, -2.29]
            3- [0, dt, -pi/2]
            4- [0, -dt, pi / 2]
            5- [-1.18, 0.49, 2.29]
            6- [1.18, 0.49, 0.87]
            7- [-pi, 0, pi / 2]

        initialize the dictionaries for post value table and policy
        :return:
        """
        Vpost = {}
        policy = {}
        for x in self.env.x_states:
            temp_yi = {}
            temp_yp = {}
            for y in self.env.y_states:
                temp_oi = {}
                temp_op = {}
                count = 0
                for o in self.env.o_states:
                    temp_oi[count] = float("inf")
                    temp_op[count] = None
                    count = count + 1

                if y == -0.0:
                    y = 0.0

                temp_yi[y] = temp_oi
                temp_yp[y] = temp_op
            if x == -0.0:
                x = 0.0

            Vpost[x] = temp_yi
            policy[x] = temp_yp
        return Vpost, policy
