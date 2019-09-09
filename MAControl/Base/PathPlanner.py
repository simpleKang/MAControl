from abc import ABC, abstractmethod


class PathPlanner(ABC):

    def __init__(self):
        pass

    @abstractmethod
    def planpath(self):
        pass
