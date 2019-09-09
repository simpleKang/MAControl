from abc import ABC, abstractmethod


class InnerController(ABC):

    def __init__(self):
        pass

    @abstractmethod
    def controlinner(self):
        pass
