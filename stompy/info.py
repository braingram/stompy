#!/usr/bin/env python

# order matches ip addresses
legs = ('fl', 'ml', 'rl', 'rr', 'mr', 'fr')

left_legs = ('fl', 'ml', 'rl')
right_legs = ('fr', 'mr', 'rr')
front_legs = ('fl', 'fr')
middle_legs = ('ml', 'mr')
rear_legs = ('rl', 'rr')


foot_centers = {
    'fr': (3.19594053089, -1.95627967418),
    'mr': (0.0, -2.3608),
    'rr': (-3.19594053089, -1.95627967418),
    'fl': (3.19594053089, 1.95627967418),
    'ml': (0.0, 2.3608),
    'rl': (-3.19594053089, 1.95627967418),

#    'fr': (1.95627967418, 3.19594053089),
#    'mr': (2.3608, 0.0),
#    'rr': (1.95627967418, -3.19594053089),
#    'fl': (-1.95627967418, 3.19594053089),
#    'ml': (-2.3608, 0.0),
#    'rl': (-1.95627967418, -3.19594053089),
}

leg_neighbors = {
    'fr': ('fl', 'mr'),
    'mr': ('fr', 'rr'),
    'rr': ('mr', 'rl'),
    'fl': ('fr', 'ml'),
    'ml': ('fl', 'rl'),
    'rl': ('ml', 'rr'),
}

leg_all_neighbors = {
    'fr': [l for l in legs if l != 'fr'],
    'mr': [l for l in legs if l != 'mr'],
    'rr': [l for l in legs if l != 'rr'],
    'fl': [l for l in legs if l != 'fl'],
    'ml': [l for l in legs if l != 'ml'],
    'rl': [l for l in legs if l != 'rl'],
}
