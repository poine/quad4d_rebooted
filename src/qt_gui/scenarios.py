#import traj_factory # not needed?


class Scenario:
    pass

class Scenario1:
    ids= [4]
    trajs= ["circle_with_intro1"]

class Scenario2:
    ids= [4, 5]
    trajs= ["circle_with_intro1", "circle_with_intro2"]

class Scenario3:
    ids= [4, 5, 6]
    trajs= ["circle_with_intro1", "circle_with_intro2", "circle_with_intro3"]

class Scenario4:
    ids= [4, 5, 6, 7]
    trajs= ["circle_with_intro1", "circle_with_intro2", "circle_with_intro3", "circle_with_intro4"]

class Scenario5:
    ids= [4,5]
    trajs= ["smooth_back_and_forth1", "smooth_back_and_forth2"]

class Scenario6:
    ids = [4]
    trajs = ["space indexed gate race1"]
    arena = "data/arena_3.yaml"


scenarios = [Scenario1, Scenario2, Scenario3, Scenario4, Scenario5, Scenario6]

