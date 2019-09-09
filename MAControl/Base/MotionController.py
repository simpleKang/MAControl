from abc import ABC, abstractmethod


class MotionController(ABC):

    def __init__(self):
        pass

    @abstractmethod
    def controlmotion(self):
        pass

