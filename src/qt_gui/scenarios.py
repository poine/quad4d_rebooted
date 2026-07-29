#import traj_factory # not needed?


class Scenario:
    pass

class Scenario1:
    desc  = 'single circle with intro'
    ids= [112]
    trajs= ["circle_with_intro1"]

class Scenario2:
    desc  = 'two circles with intro'
    ids= [112, 113]
    trajs= ["circle_with_intro1", "circle_with_intro2"]

class Scenario3:
    desc  = 'three circles with intro'
    ids= [112, 113, 114]
    trajs= ["circle_with_intro1", "circle_with_intro2", "circle_with_intro3"]

class Scenario4:
    desc  = 'four circles with intro'
    ids= [112, 113, 114, 115]
    trajs= ["circle_with_intro1", "circle_with_intro2", "circle_with_intro3", "circle_with_intro112"]

class Scenario5:
    desc  = 'two back-and-forth'
    ids= [112, 113]
    trajs= ["smooth_back_and_forth1", "smooth_back_and_forth2"]

class Scenario6:
    desc  = 'gate race, solo'
    ids = [112]
    trajs = ["space indexed gate race1"]
    arena = "data/arena_3.yaml"

class Scenario7:
    desc  = 'scara race, solo'
    ids = [112]
    trajs = ["scara race"]
    arena = "data/arena_112.yaml"

class Scenario8:
    desc  = 'circle then back-and-forth'
    ids= [112]
    trajs= ["cercle_back_and_forth"]

class Scenario9:
    desc  = 'mixed: back-and-forth, gate race, circle'
    ids= [112, 113, 114]
    trajs= ["smooth_back_and_forth1", "space indexed gate race1", "circle_with_intro1"]

class Scenario10:
    desc  = 'mixed: oval, figure-of-eight, gate race'
    ids= [112, 113, 114]
    trajs= ["space indexed oval", "space indexed figure of height2", "space indexed gate race1"]


class Scenario11:
    desc  = 'follow-the-leader, 3 drones'
    ids= [112, 113, 114]
    trajs= ["queue leu leu 1", "queue leu leu 2", "queue leu leu 3"]

class Scenario12:
    desc  = 'race track and slalom'
    ids= [112, 113]
    trajs= ["space indexed race track 1", "space indexed slalon"]

class Scenario13:
    desc  = 'follow-the-leader, 2 drones'
    ids= [112, 113]
    trajs= ["queue leu leu 1", "queue leu leu 2"]

class Scenario14:
    desc  = 'two figure-of-eight'
    ids   = [112, 113]
    trajs = ['space indexed figure of height3 flat', 'space indexed figure of height']

class Scenario15:
    desc  = 'two concentric safe circles'
    ids = [112, 113]
    trajs = ["cercle safe 1", "cercle safe 2"]

class Scenario16:
    desc  = 'three concentric safe circles'
    ids = [112, 113, 114]
    trajs = ["cercle safe 1", "cercle safe 2", "cercle safe 3"]



class Scenario17:   # rotating triangle
    desc  = 'rotating triangle'
    ids   = [112, 113, 114]
    trajs = ['show rosette a', 'show rosette b', 'show rosette c']

class Scenario18:   # swirling tower
    desc  = 'swirling tower'
    ids   = [112, 113, 114]
    trajs = ['show tornado inner', 'show tornado mid', 'show tornado outer']

class Scenario19:   # counter-rotating rings
    desc  = 'counter-rotating rings'
    ids   = [112, 113]
    trajs = ['show twin ring low', 'show twin ring high']

class Scenario20:  # pulsing ring
    desc  = 'pulsing ring'
    ids   = [112, 113, 114]
    trajs = ['show pulse a', 'show pulse b', 'show pulse c']

class Scenario21:  # stacked ovals
    desc  = 'stacked ovals'
    ids   = [112, 113]
    trajs = ['show oval low', 'show oval high']

class Scenario22:  # lissajous solo
    desc  = 'lissajous solo'
    ids   = [112]
    trajs = ['show lissajous']

class Scenario23:  # star solo
    desc  = 'star solo'
    ids   = [112]
    trajs = ['show star']


class Scenario24:  # convergence a 3
    desc  = 'three-way convergence'
    ids   = [112, 113, 114]
    trajs = ['conflit tri a', 'conflit tri b', 'conflit tri c']


class Scenario25:   # spirale montante a 3 drones
    desc  = 'ascending spiral, 3 drones'
    ids   = [112, 113, 114]
    trajs = ['spirale a', 'spirale b', 'spirale c']

class Scenario26:   # spirale a 2 drones
    desc  = 'spiral, 2 drones'
    ids   = [112, 113]
    trajs = ['spirale a', 'spirale c']

class Scenario27:   # vraie spirale montante a 3 drones
    desc  = 'true ascending spiral, 3 drones'
    ids   = [112, 113, 114]
    trajs = ['spirale montante a', 'spirale montante b', 'spirale montante c']

class Scenario28:   # safe: deux cercles cote a cote (separation spatiale)
    desc  = 'two circles side by side'
    ids   = [112, 113]
    trajs = ['circle left', 'circle right']

class Scenario29:   # safe: formes differentes a hauteurs differentes
    desc  = 'lissajous & ring, split heights'
    ids   = [112, 113]
    trajs = ['show lissajous low', 'ring high']
    
scenarios = [
    Scenario1, 
    Scenario2, 
    Scenario3, 
    Scenario4, 
    Scenario5, 
    Scenario6, 
    Scenario7, 
    Scenario8, 
    Scenario9, 
    Scenario10, 
    Scenario11, 
    Scenario12, 
    Scenario13, 
    Scenario14, 
    Scenario15, 
    Scenario16, 
    Scenario17, 
    Scenario18, 
    Scenario19, 
    Scenario20, 
    Scenario21, 
    Scenario22, 
    Scenario23,
    Scenario24,
    Scenario25,
    Scenario26,
    Scenario27,
    Scenario28,
    Scenario29
    ]


# Split the predefined scenarios into two groups for the picker: those
# designed conflict-free (solo, concentric, height/radius-separated,
# follow-the-leader) and those with inter-drone conflicts (crossing or
# converging paths -- the deconfliction testbeds). To move a scenario to
# the other group, just move its class name between the two lists below;
# anything left out defaults to no-conflict.
_WITH_CONFLICT = [
    Scenario5,    # two back-and-forth (head-on)
    Scenario9,    # mixed: back-and-forth, gate race, circle
    Scenario10,   # mixed: oval, figure-of-eight, gate race
    Scenario14,   # two figure-of-eight, same height (cross at centre)
    Scenario24,   # three-way convergence
]

for _c in scenarios:
    _c.conflict = _c in _WITH_CONFLICT


FLEET_IDS = [110, 112, 111]
_ID_REMAP = {112: FLEET_IDS[0], 113: FLEET_IDS[1], 114: FLEET_IDS[2]}
for _c in scenarios:
    _c.ids = [_ID_REMAP.get(_id, _id) for _id in _c.ids]
