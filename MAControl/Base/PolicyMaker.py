from abc import ABC, abstractmethod


class PolicyMaker(ABC):

    def __init__(self):
        pass

    @abstractmethod
    def makepolicy(self):
        pass
