#!/usr/bin/env python

import os


env_var = 'STOMPY_BULLET'

# if not explicit, try to enable
if env_var not in os.environ:
    try:
        import pybullet
        os.environ[env_var] = '1'
    except ImportError:
        print("Failed to import pybullet, disabling bullet sim")
        os.environ[env_var] = '0'

if os.environ.get(env_var, '1') == '1':
    from . import bullet

    get = bullet.get
else:
    from . import nosim

    get = nosim.get
