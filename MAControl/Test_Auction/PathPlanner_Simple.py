from MAControl.Base import PathPlanner


class PathPlanner_Simple(PathPlanner.PathPlanner):

    def __init__(self):
        super(PathPlanner_Simple, self).__init__()
        pass

    def planpath(self):
        print('This is a pathplanner.')
        pass

