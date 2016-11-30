#!/usr/bin/env python
"""
Crawl gait (moving 1 leg at a time)

Steps
- shift body in direction by distance
- move each leg in that direction by distance

When legs are on the ground, they all must be moving in unison
In the air, legs can move independently

Starting with the leg on the ground
- Send trajectory that is opposite of body target direction
  - if changed during movement, send new trajectory
- When all feet are down, pick foot that is most restricted
- lift that foot
  - if changed during movement, send new trajectory
- move foot to forward foot fall location
- put foot down (while following trajectory)
  - if changed during movement, send new trajectory
"""
