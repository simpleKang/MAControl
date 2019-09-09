from MAControl.Base import MotionController


class MotionController_L1_TECS(MotionController.MotionController):

    def __init__(self):
        super(MotionController_L1_TECS, self).__init__()
        pass

    def controlmotion(self):
        print('This is a motioncontroller.')
        pass

