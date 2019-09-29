import math


def atan3(ychange, xchange):
    atan = math.atan2(ychange, xchange)
    if atan > 0:
        return atan
    else:
        return 2 * math.pi + atan
