from MAControl.Base import InnerController


class InnerController_PID(InnerController.InnerController):

    def __init__(self):
        super(InnerController_PID, self).__init__()
        pass

    def controlinner(self):
        print('This is a innercontroller.')
        pass

