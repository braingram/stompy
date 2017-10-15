#!/usr/bin/env python
"""
Calf joint is a 4 bar linkage with sides:
    1.75" A: sensor arm length
    4.75" B: sensor arm to calf link
    1.75" C: calf link mounting hole distance from pivot
    5" D: calf link pivot to sensor pivot

The sensor outputs a voltage proportional to angle DA
and we are interested in measuring angle CD

Treat this as 2 triangles
1) AD + diagonal (E)
2) BC + diagonal (E)

1) compute diagonal E (using A, D + angle DA)
E = sqrt(A * A + D * D - 2 * A * D * cos(DA))

2) compute angle ED of triangle 1 (using A, D, E)
E = b
D = c
A = a
ED = alpha

alpha = acos((b * b + c * c - a * a) / (2 * b * c))
ED = acos((E * E + D * D - A * A) / (2 * E * D))

3) compute angle of CE of triangle 2 (using B, C, E)
C = c
E = b
B = a
CE = alpha
CE = acos((E * E + C * C - B * B) / (2 * E * C))

4) add angles #2 and #3
CE + ED = CD

5) compute excursion of spring using angle #4
"""

import numpy
import pylab


radians = numpy.radians
sin = numpy.sin
cos = numpy.cos


class CalfLinkage(object):
    def __init__(
            self, a=1.75, b=4.75, c=1.75, d=5.0,
            da=112, ab=70, bc=125, cd=55):
        self.a = a
        self.b = b
        self.c = c
        self.d = d
        self.da = da
        self.ab = ab
        self.bc = bc
        self.cd = cd

    def draw(self, x=0, y=0):
        pylab.gca().set_aspect(1.0)
        # x,y is position of a
        # assuming d is vertical
        # draw a
        x1 = x + sin(radians(self.ab))
        y1 = y + cos(radians(self.ab))
        pylab.plot([x, x1], [y, y1])
        # d
        x2 = x
        y2 = y - self.d
        pylab.plot([x, x2], [y, y2])

        # c
        x3 = x2 + sin(radians(180 - self.bc))
        y3 = y2 + cos(radians(180 - self.bc))
        pylab.plot([x2, x3], [y2, y3])

        # b
        pylab.plot([x1, x3], [y1, y3])
        #dx = x3 - x1
        #dy = y3 - y1
        #print(numpy.sqrt(dx * dx + dy * dy) - self.b)

    def compute_cd(self):
        # 1) compute diagonal E (using A, D + angle DA)
        # E = sqrt(A * A + D * D - 2 * A * D * cos(DA))
        e = numpy.sqrt(
            self.a * self.a +
            self.d * self.d -
            2 * self.a * self.d * cos(radians(self.da)))

        # 2) compute angle ED of triangle 1 (using A, D, E)
        # ED = acos((E * E + D * D - A * A) / (2 * E * D))
        ed = numpy.arccos(
            (e * e + self.d * self.d - self.a * self.a) /
            (2 * e * self.d))

        # 3) compute angle of CE of triangle 2 (using B, C, E)
        # CE = acos((E * E + C * C - B * B) / (2 * E * C))
        ce = numpy.arccos(
            (e * e + self.c * self.c - self.b * self.b) /
            (2 * e * self.c))

        # 4) add angles #2 and #3
        # CD = CE + ED
        cd = numpy.degrees(ce + ed)
        e = cd - self.cd
        print(e)

        # 5) compute excursion of spring using angle #4
        return cd


if __name__ == '__main__':
    c = CalfLinkage()
    c.draw()
    c.compute_cd()
    pylab.show()
