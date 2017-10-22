#!/usr/bin/env python
"""
Calf joint is a 4 bar linkage with sides:
    1.75" A: sensor arm length
    4.75" B: sensor arm to calf link
    1.75" C: calf link mounting hole distance from pivot
    5" D: calf link pivot to sensor pivot

    8" length of calf link
    18" from upper calf link mount to spring top
    ~18" spring length when unloaded
    600 lb / inch

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
F = 18"
G = 8"
sqrt(F * F + G * G - 2 * F * G * cos(CD))
"""

import numpy
import pylab


radians = numpy.radians
sin = numpy.sin
cos = numpy.cos


class CalfLinkage(object):
    def __init__(
            self, a=1.75, b=4.75, c=1.75, d=5.0,
            da=112, ab=70, bc=125, cd=55,
            f=18., g=8., h=18., lbin=600.):
        self.a = a
        self.b = b
        self.c = c
        self.d = d
        self.da = da
        self.ab = ab
        self.bc = bc
        self.cd = cd
        self.f = f  # upper triangle link
        self.g = g  # calf link
        self.h = h  # base calf spring length
        self.lbin = lbin  # lbs per inch spring compression

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
        # DA is the measured sensor value
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
        cd = ce + ed  # radians
        #e = numpy.degrees(cd) - self.cd
        #print("calf link angle error: %s" % e)

        # 5) compute excursion of spring using angle #4
        return cd

    def compute_spring_length(self, cd=None):
        if cd is None:
            cd = self.compute_cd()
        return numpy.sqrt(
            self.f * self.f + self.g * self.g -
            2 * self.f * self.g * numpy.cos(cd))

    def compute_calf_load(self, sl=None):
        if sl is None:
            sl = self.compute_spring_length()
        dl = self.h - sl
        return dl * self.lbin


def draw_calf():
    c = CalfLinkage()
    c.draw()
    cd = c.compute_cd()
    print("sensor angle error: %s" % (numpy.degrees(cd) - c.cd))
    sl = c.compute_spring_length(cd)
    print("spring length: %s" % sl)
    load = c.compute_calf_load(sl)
    print("Calf load: %s" % load)
    pylab.show()


def compress_calf():
    c = CalfLinkage()
    sda = c.da
    angles = numpy.arange(30) + sda - 17
    sls = []
    lbs = []
    for a in angles:
        c.da = a
        sl = c.compute_spring_length()
        lb = c.compute_calf_load(sl)
        sls.append(sl)
        lbs.append(lb)
    sls = numpy.array(sls)
    lbs = numpy.array(lbs)
    pylab.subplot(311)
    #pylab.plot(angles)
    pylab.plot(sls)
    #pylab.plot(lbs)
    # how far is lbs from a straight line?
    flbs = numpy.linspace(lbs.min(), lbs.max(), lbs.size)
    r = lbs - flbs
    pylab.subplot(312)
    pylab.plot(angles, r)
    pylab.title('calf linear model error')
    pylab.ylabel('load error (lbs)')
    pylab.subplot(313)
    pylab.plot(angles, (r * 100.) / lbs)
    pylab.ylabel('load error (%)')
    pylab.xlabel('sensor angle (degrees)')
    #pylab.plot((angles - angles.min()) / angles.ptp())
    #pylab.plot((sls - sls.min()) / sls.ptp())
    #pylab.plot((lbs - lbs.min()) / lbs.ptp())
    pylab.show()


if __name__ == '__main__':
    compress_calf()
