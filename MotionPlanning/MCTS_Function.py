
import math
import random
import numpy as np

from MCTS_Nodes import Node as Node


class MctsTree:
    """
    A tree in the MCTS tree. Each tree keeps track of its own root node, and the maximum acceleration
    """
    root: Node
    accMax: int
    accSpeed: int
    MaxTimeHorizon: float
    TimeResolution: float

    def __init__(self,ego_car_properties, root_state, accmax, accspeed) -> None:
        self.root = Node(ego_car_properties, root_state, visits=1)
        self.accMax = accmax
        self.accSpeed = accspeed

    def selection(self, start_node: Node) -> Node:
        """
        Select the best child of the current node
        :param start_node:
        :return:
        """
        next_node = start_node.children[-1]
        for child in start_node.children:
            if child.UCB >= next_node.UCB:
                next_node = child
        return next_node

    def expand(self, node: Node, time_resolution: float) -> None:
        """
        Expand the current node
        """
        for i in range(0, self.accMax + 2, 2):
            slow_down_s, slow_down_l = self.get_next_position(node, -i, 0)
            node.add_child(Node(("slow_down", np.array([node.state[1][0] - i * self.TimeResolution,
                                                        i, 0, 0, slow_down_s, slow_down_l])),
                                node.curr_time_state + time_resolution, node))
            speed_up_s, speed_up_l = self.get_next_position(node, i, 0)
            node.add_child(Node(("speed_up", np.array([node.state[1][0] + i * self.TimeResolution,
                                                       - i, 0, 0, speed_up_s, speed_up_l])),
                                node.curr_time_state + time_resolution, node))
            keep_speed_s, keep_speed_l = self.get_next_position(node, 0, 0)
            node.add_child(Node(("keep_speed", np.array([node.state[1][0], i, 0, 0, keep_speed_s, keep_speed_l])),
                                node.curr_time_state + time_resolution, node))

    def roll_out(self, node: Node, MaxTimeHorizon: float, TimeResolution: float) -> float:
        """
        Randomly select a child of the current node until reach terminal node
        and compute the score based on the terminal state
        :param node:
        :return:
        """
        start_node = node.copy_leaf()
        curr_node = start_node
        total_reward = 0.0
        while self.is_terminal(curr_node, MaxTimeHorizon) is False and self.check_collision(curr_node) is False and \
                curr_node.curr_time_state < MaxTimeHorizon:
            random_number = random.random()
            random_acc = random.randint(0, self.accMax)
            if random_number <= 0.20:
                slow_down_s, slow_down_l = self.get_next_position(node, -random_acc, 0)
                new_node = Node(state=("slow_down", np.array([curr_node.state[1][0] - random_acc * self.TimeResolution,
                                                        random_acc, 0, 0, slow_down_s, slow_down_l]))
                                , curr_time_state=curr_node.curr_time_state + TimeResolution, parent=curr_node)
                total_reward += self.compute_reward(new_node)
                curr_node = new_node
            elif random_number >= 0.80:
                speed_up_s, speed_up_l = self.get_next_position(node, random_acc, 0)
                new_node = Node(estate=("speed_up", np.array([curr_node.state[1][0] + random_acc * self.TimeResolution,
                                                        random_acc, 0, 0, speed_up_s, speed_up_l])),
                                curr_time_state=curr_node.curr_time_state + TimeResolution, parent=curr_node)
                total_reward += self.compute_reward(new_node)
                curr_node = new_node
            else:
                keep_speed_s, keep_speed_l = self.get_next_position(node, 0, 0)
                new_node = Node(("keep_speed", np.array([curr_node.state[1][0], random_acc, 0, 0, keep_speed_s, keep_speed_l])),
                                curr_node.curr_time_state + TimeResolution, curr_node)
                total_reward += self.compute_reward(new_node)
                curr_node = new_node
        return total_reward

    def back_propagation(self, node: Node, score: float) -> None:
        """
        Update the score of the current node
        :param node:
        :return:
        """
        while node is not None:
            node.update(score)
            node = node.parent
        self.update_ucb()
        return None

    def is_terminal(self, node: Node, MaxTimeHorizon: float) -> bool:
        """
        Check if the current node is a terminal node
        :param node:
        :return:
        """
        if node.curr_time_state >= MaxTimeHorizon:
            # If the time horizon has been reached, then it's a terminal state
            return True
        else:
            return False

    def check_collision(self, node: Node) -> bool:
        """
        Check if the rollout has resulted in a collision
        :param node:
        :return:
        """
        if node.state[5] + node.ego_car_properties[0]

        self.update_ucb()
        pass

    def compute_reward(self, node: Node) -> float:
        """
        Compute the reward for reaching a node based on the actions taken
        :param node:
        :return:
        """
        # assign reward based on the actions
        if node.state[0] == "keep_speed":
            reward = 1.0  # high score for keeping speed
        elif node.state[0] == "slow_down":
            reward = -0.5  # some penalty for slowing down
        else:
            reward = -0.5  # some penalty for speeding up

        if self.check_collision(node):
            reward = -10.0  # heavy penalty for collision
        elif self.check_sudden_stop(node):
            reward = -2.0  # lesser penalty for sudden stop

        return reward

    def update_ucb(self) -> None:
        """
        Update the UCB for all the nodes in the tree
        :param:
        :return:
        """
        queue = [self.root]
        while queue:
            node = queue.pop(0)
            if node.visits == 0:
                node.UCB = float('inf')
            else:
                node.UCB = node.score / node.visits + 2 * math.sqrt(math.log(self.root.visits) / node.visits)
            if node.children:
                for child in node.children:
                    queue.append(child)
        return None

    def mcts(self, max_iter: int) -> Node:
        """
        The main function of the MCTS
        """
        root = self.root
        queue = [root]
        maxtimehorizon = self.MaxTimeHorizon
        timeresolution = self.TimeResolution
        while self.root.visits < max_iter:
            curr_node = queue.pop(0)
            if not self.is_terminal(curr_node, maxtimehorizon) and not self.check_collision(curr_node):
                if not curr_node.children:
                    if curr_node.visits == 0:
                        reward = self.roll_out(curr_node, maxtimehorizon, timeresolution)
                        self.back_propagation(curr_node, reward)
                        queue = [root]
                    else:
                        self.expand(curr_node, timeresolution)
                        queue = [root]
                else:
                    queue.append(self.selection(curr_node))
            else:
                if self.is_terminal(curr_node, maxtimehorizon):
                    reward = self.roll_out(curr_node, maxtimehorizon, timeresolution)
                    self.back_propagation(curr_node, reward)
                queue = [root]
        root.children.sort(key=lambda x: x.UCB, reverse=False)
        return root.children[-1]

    def check_sudden_stop(self, node: Node) -> bool:
        """
        Check if the rollout has resulted in a sudden stop
        :param node:
        """
        if node.state[1][0] <= 1 and node.state[1][1] < 0:
            return True

    def get_next_position(self, node: Node, acceleration_S: float, acceleration_L: float) -> tuple[float, float]:
        """
        Get the next position of the ego vehicle
        """
        delta_t = self.TimeResolution
        next_position_s = node.state[1][0] * delta_t + 0.5 * acceleration_S * delta_t ** 2
        next_position_l = node.state[1][2] * delta_t + 0.5 * acceleration_L * delta_t ** 2
        return next_position_s, next_position_l
