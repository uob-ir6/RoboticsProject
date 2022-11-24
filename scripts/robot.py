class Robot(object):
    def __init__(self, robotId, pose, state, location, assignmentPoint):
        self.id = robotId
        self.pose = pose
        self.state = state
        self.location = location
        self.assignmentPoint = assignmentPoint
        