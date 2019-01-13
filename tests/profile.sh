#!/bin/bash

# generate profile
python -m cProfile -o program.prof ui.py

# visualize profile
snakeviz program.prof
