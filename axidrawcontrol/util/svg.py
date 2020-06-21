import math
import re
from itertools import islice
import xml.etree.ElementTree as ET

import svgelements
import pint
unitRegistry = pint.UnitRegistry()
unitRegistry.define('pt = point')
Quantity = unitRegistry.Quantity

from . import aggsubdivision

import logging

def path_to_polylines(path_or_svgd, start=0j, tolerance=0.1):

    def subdivide_cubicBezier(cubic):
        for x,y in islice(aggsubdivision.bezier(
            (cubic.start   .real, cubic.start   .imag),
            (cubic.control1.real, cubic.control1.imag),
            (cubic.control2.real, cubic.control2.imag),
            (cubic.end     .real, cubic.end     .imag),
            distance_tolerance=tolerance
        ), 1, None):
            yield complex(x,y)


    def subpath_to_polyline(subpath):
        for seg in subpath:
            if isinstance(seg, svgelements.Move):
                yield complex(*seg.end)
            elif isinstance(seg, svgelements.Line):
                yield complex(*seg.end)
            elif isinstance(seg, svgelements.CubicBezier):
                yield from subdivide_cubicBezier(seg)
            elif isinstance(seg, svgelements.QuadraticBezier):
                cubic = svgelements.CubicBezier(
                        seg.start,
                    1/3*seg.start + 2/3*seg.control,
                                    2/3*seg.control + 1/3*seg.end,
                                                          seg.end
                )
                yield from subdivide_cubicBezier(cubic)
            elif isinstance(seg, svgelements.Arc):
                for cubic in seg.as_cubic_curves():
                    yield from subdivide_cubicBezier(cubic)
            elif isinstance(seg, svgelements.Close):
                yield complex(*seg.end)
            else:
                logging.warn('unimplemented segement type %s', type(seg))


    if isinstance(path_or_svgd, svgelements.Path):
        path = path_or_svgd
    else:
        path = svgelements.Path()
        path.append(svgelements.Move(start))
        path.parse(path_or_svgd)
        path.pop(0)
    for subpath in path.as_subpaths():
        if subpath:
            yield list(subpath_to_polyline(subpath))


def find_paths(svgfn, scale=1):
    svg = ET.parse(svgfn).getroot()

    try:
        x0,y0,w,h = map(float, svg.attrib['viewBox'].split(' '))
        sx = Quantity(svg.attrib['width' ]).m_as('mm') / w
        sy = Quantity(svg.attrib['height']).m_as('mm') / h
        xform0 = 'scale(%s %s)' %(sx*scale, sy*scale)
    except KeyError:
        logging.warn('could not determine SVG units, assuming mm')
        xform0 = 'scale(%s %s)' %(scale, scale)

    for e,ancestors in walk_svg(svg):
        tag = simple_tag(e)
        if tag == 'path':
            path = svgelements.Path(e.get('d',''))
            transform_str = ' '.join(e2.get('transform','') for e2 in ancestors)
            transform = svgelements.Matrix(xform0 + transform_str)
            transformed_path = path * transform
            transformed_path.reify()
            yield transformed_path




STRIP_NS_RE = re.compile(r'^\{.+\}')
def simple_tag(e):
    return STRIP_NS_RE.sub('', e.tag)

def walk_svg(svg):
    q = [(e, []) for e in svg]
    while q:
        e,ancestors = q.pop(0)
        tag = simple_tag(e)
        if tag == 'defs':
            pass
        elif tag == 'use':
            href = e.get('{http://www.w3.org/1999/xlink}href')
            if href:
                found = svg.findall('.//*[@id="%s"]' % href.lstrip('#'))
                if found:
                    q = [(e2, ancestors+[e]) for e2 in found[0]] + q
                else:
                    logging.warn('could not find href element for <use>')
            else:
                logging.warn('could not get href attr for <use>')
        else:
            if tag != 'g':
                yield e,ancestors
            q = [(e2, ancestors+[e]) for e2 in e] + q



