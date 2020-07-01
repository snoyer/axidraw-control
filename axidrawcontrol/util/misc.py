import threading


def roundf(x):
    xi = round(x)
    return x - xi, xi


def croundf(c):
    xf,xi = roundf(c.real)
    yf,yi = roundf(c.imag)
    return complex(xf,yf), complex(xi,yi)


def clamp(x, low, high):
    return low  if x <= low  \
      else high if x >= high \
      else x


def lerp(t, t0, t1, v0, v1):
    tv = (t - t0) / (t1 - t0)
    return v0 + (v1 - v0) * tv


def lerp_clamp(t, t0, t1, v0, v1):
    return lerp(clamp(t, t0,t1), t0,t1, v0,v1)




def uniq(xs):
    prev = None
    for x in xs:
        if x != prev:
            yield x
            prev = x


def pairwise(iterable):
    try:
        iterable = iter(iterable)
        prev = next(iterable)
        while iterable:
            try:
                item = next(iterable)
                yield prev,item
                prev = item
            except StopIteration:
                break
    except StopIteration:
        pass


def split_iterable(iterable, after=None, before=None, between=None, streaming=False):
    stop = threading.Event()
    try:
        iterable = iter(iterable)
        peeked = []

        def iter_until():
            prev = None
            while peeked:
                item = peeked.pop()
                yield item
                prev = item

            while not stop.is_set():
                try:
                    item = next(iterable)

                    if (before and before(item)) \
                    or (between and between(prev, item)):
                        peeked.append(item)
                        break
                    
                    yield item
                    prev = item

                    if after and after(item):
                        break

                except StopIteration:
                    stop.set()

        while not stop.is_set():
            if not peeked:
                peeked.append(next(iterable))
            group = iter_until()
            yield group if streaming else list(group)

    except StopIteration:
        stop.set()




def pretty_time_delta(seconds):
    seconds = int(seconds)
    hours, seconds = divmod(seconds, 3600)
    minutes, seconds = divmod(seconds, 60)
    # if hours > 0:
    #     return '%dh%dm%ds' % (hours, minutes, seconds)
    # elif minutes > 0:
    #     return '%dm%ds' % (minutes, seconds)
    # else:
    #     return '%ds' % (seconds,)

    if hours > 0:
        return '%02d:%02d:%02d' % (hours, minutes, seconds)
    else:
        return '%02d:%02d' % (minutes, seconds)



from pint import UnitRegistry, DimensionalityError
unitRegistry = UnitRegistry()
unitRegistry.define('pt = point')
Quantity = unitRegistry.Quantity

def as_unit(v,u):
    if u:
        q = Quantity(v)
        if q.dimensionless:
            return float(v)
        else:
            return q.m_as(u)
    else:
        return float(v)

def parse_value(v, default, minmax, unit=None):
    if v == None:
        v = default
    try:
        if isinstance(v, str) and v.endswith('%'):
            t = float(v.replace('%', ''))/100
            return lerp_clamp(t, 0,1, as_unit(minmax[0], unit), as_unit(minmax[1], unit))
        else:
            return as_unit(v, unit)
    except DimensionalityError as e:
        raise ValueError('`%s` not convertible to %s' % (v,unit))
