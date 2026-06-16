#import traj_factory # not needed?


class Scenario:
    pass

class Scenario1:
    ids= [112]
    trajs= ["circle_with_intro1"]

class Scenario2:
    ids= [112, 113]
    trajs= ["circle_with_intro1", "circle_with_intro2"]

class Scenario3:
    ids= [112, 113, 114]
    trajs= ["circle_with_intro1", "circle_with_intro2", "circle_with_intro3"]

class Scenario4:
    ids= [112, 113, 114, 115]
    trajs= ["circle_with_intro1", "circle_with_intro2", "circle_with_intro3", "circle_with_intro4"]

class Scenario5:
    ids= [112, 113]
    trajs= ["smooth_back_and_forth1", "smooth_back_and_forth2"]

class Scenario6:
    ids = [112]
    trajs = ["space indexed gate race1"]
    arena = "data/arena_3.yaml"

class Scenario7:
    ids = [4]
    trajs = ["scara race"]
    arena = "data/arena_4.yaml"

class Scenario8:
    ids= [112]
    trajs= ["cercle_back_and_forth"]

class Scenario9:
    ids= [112, 113, 114]
    trajs= ["smooth_back_and_forth1", "space indexed gate race1", "circle_with_intro1"]

class Scenario10:
    ids= [112, 113, 114]
    trajs= ["space indexed oval", "space indexed figure of height2", "space indexed gate race1"]


class Scenario11:
    ids= [112, 113, 114]
    trajs= ["queue leu leu 1", "queue leu leu 2", "queue leu leu 3"]

scenarios = [Scenario1, Scenario2, Scenario3, Scenario4, Scenario5, Scenario6, Scenario7, Scenario8, Scenario9, Scenario10, Scenario11]

