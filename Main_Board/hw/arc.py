from math import *

X = 129.9
Y = 119.55
# R 40/47.5/50

for r1 in [49.4, 49.4-3.5, 42, 42-3.5]:
  # print('(start %g %g) (mid %g %g) (end %g %g)' % (X-r1, Y, X+r1, Y, X-r1, Y))
  pass

for r1, a1, a2, net in [
  (49.4, -160, 148.3, 16),
  (49.4-3.5, -157, 151.3, 15),
  (42, 172.8-360, 121.4, 16),
  (42-3.5, 176.4-360, 125, 15),
]:
  a1 = a1 / 180 * pi
  a2 = a2 / 180 * pi
  print('(arc (start %g %g) (mid %g %g) (end %g %g) (layer "B.Cu") (net %d) (tstamp 6a036b40-33f9-415a-bacc-3fdd6c988bac))' %
    (X + cos(a1)*r1, Y - sin(a1)*r1,
     X + cos((a1+a2)/2)*r1, Y - sin((a1+a2)/2)*r1,
     X + cos(a2)*r1, Y - sin(a2)*r1,
     net))
