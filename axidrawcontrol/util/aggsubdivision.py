"""port of the adaptive bezier subdivision algorithm from the Anti-Grain Geometry library.
see http://www.antigrain.com/research/adaptive_bezier/index.html for reference
"""

from collections import namedtuple
from math import atan2, fabs


AdaptiveSubdivisionParams = namedtuple('AdaptiveSubdivision', [
    'distance_tolerance', 'angle_tolerance',
    'curve_recursion_limit', 'cusp_limit',
    'curve_collinearity_epsilon', 'curve_angle_tolerance_epsilon'
])


def bezier(xy1, xy2, xy3, xy4,
            distance_tolerance=1.,
            angle_tolerance=0,
            recursion_limit=32,
            cusp_limit=.1,
            collinearity_epsilon=1e-12,
            angle_tolerance_epsilon=1e-12
    ):
    params = AdaptiveSubdivisionParams(
        distance_tolerance**2,
        angle_tolerance,
        recursion_limit,
        cusp_limit,
        collinearity_epsilon,
        angle_tolerance_epsilon,
    )
    yield xy1
    yield from recursive_bezier(params, *xy1, *xy2, *xy3, *xy4)
    yield xy4


def recursive_bezier(params, x1,y1, x2,y2, x3,y3, x4,y4, level=0):
    if level > params.curve_recursion_limit:
        return

    # Calculate all the mid-points of the line segments
    x12   = (x1   + x2  ) / 2.
    y12   = (y1   + y2  ) / 2.
    x23   = (x2   + x3  ) / 2.
    y23   = (y2   + y3  ) / 2.
    x34   = (x3   + x4  ) / 2.
    y34   = (y3   + y4  ) / 2.
    x123  = (x12  + x23 ) / 2.
    y123  = (y12  + y23 ) / 2.
    x234  = (x23  + x34 ) / 2.
    y234  = (y23  + y34 ) / 2.
    x1234 = (x123 + x234) / 2.
    y1234 = (y123 + y234) / 2.

    if level > 0: # Enforce subdivision first time
        # Try to approximate the full cubic curve by a single straight line
        dx = x4-x1
        dy = y4-y1

        d2 = abs(((x2 - x4) * dy - (y2 - y4) * dx))
        d3 = abs(((x3 - x4) * dy - (y3 - y4) * dx))

        if d2 > params.curve_collinearity_epsilon and d3 > params.curve_collinearity_epsilon:
            # Regular care
            if (d2 + d3)*(d2 + d3) <= params.distance_tolerance * (dx*dx + dy*dy):
                # If the curvature doesn't exceed the distance_tolerance value
                # we tend to finish subdivisions.
                if params.angle_tolerance < params.curve_angle_tolerance_epsilon:
                    yield (x1234, y1234)
                    return

                # Angle & Cusp Condition
                a23 = atan2(y3 - y2, x3 - x2)
                da1 = fabs(a23 - atan2(y2 - y1, x2 - x1))
                da2 = fabs(atan2(y4 - y3, x4 - x3) - a23)
                if da1 >= pi: da1 = 2*pi - da1
                if da2 >= pi: da2 = 2*pi - da2

                if da1 + da2 < params.angle_tolerance:
                    # Finally we can stop the recursion
                    yield (x1234, y1234)
                    return

                if params.cusp_limit != 0.0:
                    if da1 > params.cusp_limit:
                        yield (x2, y2)
                        return

                    if da2 > params.cusp_limit:
                        yield (x3, y3)
                        return

        else:
            if d2 > params.curve_collinearity_epsilon:
                # p1,p3,p4 are collinear, p2 is considerable
                if d2 * d2 <= params.distance_tolerance * (dx*dx + dy*dy):
                    if params.angle_tolerance < params.curve_angle_tolerance_epsilon:
                        yield (x1234, y1234)
                        return

                    # Angle Condition
                    da1 = fabs(atan2(y3 - y2, x3 - x2) - atan2(y2 - y1, x2 - x1))
                    if da1 >= pi: da1 = 2*pi - da1

                    if da1 < params.angle_tolerance:
                        yield (x2, y2)
                        yield (x3, y3)
                        return

                    if params.cusp_limit != 0.0:
                        if da1 > params.cusp_limit:
                            yield (x2, y2)
                            return

            elif d3 > params.curve_collinearity_epsilon:
                # p1,p2,p4 are collinear, p3 is considerable
                if d3 * d3 <= params.distance_tolerance * (dx*dx + dy*dy):
                    if params.angle_tolerance < params.curve_angle_tolerance_epsilon:
                        yield (x1234, y1234)
                        return

                    # Angle Condition
                    da1 = fabs(atan2(y4 - y3, x4 - x3) - atan2(y3 - y2, x3 - x2))
                    if da1 >= pi: da1 = 2*pi - da1

                    if da1 < params.angle_tolerance:
                        yield (x2, y2)
                        yield (x3, y3)
                        return

                    if params.cusp_limit != 0.0:
                        if da1 > params.cusp_limit:
                            yield (x3, y3)
                            return

            else:
                # Collinear case
                dx = x1234 - (x1 + x4) / 2
                dy = y1234 - (y1 + y4) / 2
                if dx*dx + dy*dy <= params.distance_tolerance:
                    yield (x1234, y1234)
                    return

    # Continue subdivision
    yield from recursive_bezier(params, x1,y1, x12,y12, x123,y123, x1234,y1234, level+1)
    yield from recursive_bezier(params, x1234,y1234, x234,y234, x34,y34, x4,y4, level+1)
