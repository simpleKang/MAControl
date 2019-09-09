from MAControl.Base import PolicyMaker


class PolicyMaker_Auciton(PolicyMaker.PolicyMaker):

    def __init__(self):
        super(PolicyMaker_Auciton, self).__init__()
        pass

    def makepolicy(self):
        print('This is a policymaker.')

