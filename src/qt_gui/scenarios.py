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
    desc  = 'two back-and-forth'
    ids= [112, 113]
    trajs= ["smooth_back_and_forth1", "smooth_back_and_forth2"]

class Scenario5:
    desc  = 'gate race, solo'
    ids = [112]
    trajs = ["space indexed gate race1"]
    arena = "data/arena_3.yaml"

class Scenario6:
    desc  = 'scara race, solo'
    ids = [112]
    trajs = ["scara race"]
    arena = "data/arena_112.yaml"

class Scenario7:
    desc  = 'circle then back-and-forth'
    ids= [112]
    trajs= ["cercle_back_and_forth"]

class Scenario8:
    desc  = 'two figure-of-eight'
    ids   = [112, 113]
    trajs = ['space indexed figure of height3 flat', 'space indexed figure of height']

class Scenario9:
    desc  = 'two concentric safe circles'
    ids = [112, 113]
    trajs = ["cercle safe 1", "cercle safe 2"]

class Scenario10:
    desc  = 'three concentric safe circles'
    ids = [112, 113, 114]
    trajs = ["cercle safe 1", "cercle safe 2", "cercle safe 3"]



class Scenario11:  
    desc  = 'rotating triangle'
    ids   = [112, 113, 114]
    trajs = ['show rosette a', 'show rosette b', 'show rosette c']

class Scenario12:   
    desc  = 'swirling tower, 3 drones'
    ids   = [112, 113, 114]
    trajs = ['show tornado inner', 'show tornado mid', 'show tornado outer']

class Scenario13:   
    desc  = 'swirling tower, 2 drones'
    ids   = [112, 113]
    trajs = ['show tornado inner', 'show tornado outer']

class Scenario14:  
    desc  = 'counter-rotating rings'
    ids   = [112, 113]
    trajs = ['show twin ring low', 'show twin ring high']

class Scenario15:  
    desc  = 'pulsing ring'
    ids   = [112, 113, 114]
    trajs = ['show pulse a', 'show pulse b', 'show pulse c']

class Scenario16:  
    desc  = 'stacked ovals'
    ids   = [112, 113]
    trajs = ['show oval low', 'show oval high']

class Scenario17:  
    desc  = 'lissajous solo'
    ids   = [112]
    trajs = ['show lissajous']

class Scenario18:  
    desc  = 'three-way convergence'
    ids   = [112, 113, 114]
    trajs = ['conflit tri a', 'conflit tri b', 'conflit tri c']

class Scenario19:  
    desc  = 'spiral, 3 drones'
    ids   = [112, 113, 114]
    trajs = ['spirale a', 'spirale b', 'spirale c']

class Scenario20:
    desc  = 'spiral, 2 drones'
    ids   = [112, 113]
    trajs = ['spirale a', 'spirale c']

class Scenario21:   
    desc  = 'true ascending spiral, 3 drones'
    ids   = [112, 113, 114]
    trajs = ['spirale montante a', 'spirale montante b', 'spirale montante c']

class Scenario22:   
    desc  = 'true ascending spiral, 2 drones'
    ids   = [112, 113]
    trajs = ['spirale montante a', 'spirale montante c']

class Scenario23:  
    desc  = 'blooming flower, 3 drones'
    ids   = [112, 113, 114]
    trajs = ['flower a', 'flower b', 'flower c']
 
class Scenario24:  
    desc  = 'double helix (DNA), 2 drones'
    ids   = [112, 113]
    trajs = ['dna strand a', 'dna strand b']
 
class Scenario25:   
    desc  = 'cascade staircase, 3 drones'
    ids   = [112, 113, 114]
    trajs = ['cascade low', 'cascade mid', 'cascade high']
 
class Scenario26:   
    desc  = 'spirograph rosette, solo'
    ids   = [112]
    trajs = ['show spirograph']
 
class Scenario27:  
    desc  = '3D torus knot, solo'
    ids   = [112]
    trajs = ['show knot']
 
class Scenario28:  
    desc  = 'spirograph tower, 2 drones'
    ids   = [112, 113]
    trajs = ['show spirograph low', 'show spirograph high']
 
class Scenario29:  
    desc  = 'fountain bloom, 3 drones'
    ids   = [112, 113, 114]
    trajs = ['fountain a', 'fountain b', 'fountain c']
 
class Scenario30:   
    desc  = 'fountain bloom, 2 drones'
    ids   = [112, 113]
    trajs = ['fountain a', 'fountain opp']
    
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
    Scenario29,
    Scenario30,
    ]


# Split the predefined scenarios into two groups for the picker
_WITH_CONFLICT = [
    Scenario2,    
    Scenario3,    
    Scenario4,    # two back-and-forth (head-on)
    Scenario8,    # two figure-of-eight, same height (cross at centre)
    Scenario18,   # three-way convergence
]

for _c in scenarios:
    _c.conflict = _c in _WITH_CONFLICT


FLEET_IDS = [110, 112, 111]
_ID_REMAP = {112: FLEET_IDS[0], 113: FLEET_IDS[1], 114: FLEET_IDS[2]}
for _c in scenarios:
    _c.ids = [_ID_REMAP.get(_id, _id) for _id in _c.ids]
