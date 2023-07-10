
from typing import Optional
import numpy as np


class Node:
    """
    A node in the MCTS tree. Each node keeps track of its own state, the parent
    that generated it, the children that it has generated, the number of times
    the node has been visited, and the score of all the visits combined.

    === Attributes ===
    state: ("slow_down", ds, dds, dl, ddl, kappa, theta, s, l) or ("speed_up", ds, dds, dl, ddl, s, l)
     or ("keep_speed", ds, dds, dl, ddl, s, l)
    """
    state: tuple[str, np.ndarray]
    children: list[Node]
    parent: Optional[Node]
    score: float
    visits: int
    UCB: float
    curr_time_state: float
    ego_car_properties: list[float]

    def __init__(self, ego_car_properties: list, state: tuple, curr_time_state=0.0, parent=None, visits=0) -> None:
        self.ego_car_properties = ego_car_properties
        self.state = state
        self.parent = parent
        self.children = []
        self.visits = visits
        self.score = 0.0
        self.UCB = 0.0
        self.curr_time_state = curr_time_state

    def add_child(self, child: Node) -> None:
        """
        Add a child to the node
        :param child:
        """
        self.children.append(child)

    def update(self, score: float) -> None:
        """
        Update the score of the node
        :param score:
        """
        self.score += score
        self.visits += 1

    def copy_leaf(self) -> Node:
        """
        Return a copy of the leaf node
        :return:
        """
        node = Node(self.state, curr_time_state=self.curr_time_state)
        node.score = self.score
        return node

    def __repr__(self):
        return f"Node({self.state}, {self.visits}, {self.score})"

    def __str__(self):
        return f"Node({self.state}, {self.visits}, {self.score})"
