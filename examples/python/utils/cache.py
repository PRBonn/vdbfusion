# MIT License
#
# # Copyright (c) 2022 Ignacio Vizzo, Cyrill Stachniss, University of Bonn
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

import functools as ft

from diskcache import Cache
from diskcache.core import ENOVAL, args_to_key, full_name


def get_cache(directory):
    return Cache(directory, timeout=1, size_limit=3e11)


def memoize(name=None, typed=False, expire=None, tag=None):
    """Same as DiskCache.memoize but ignoring the first argument(self) for the keys."""
    # Caution: Nearly identical code exists in DjangoCache.memoize
    if callable(name):
        raise TypeError(f"name {name} cannot be callable")

    def decorator(func):
        """Decorator created by memoize() for callable `func`."""
        base = (full_name(func),) if name is None else (name,)

        @ft.wraps(func)
        def wrapper(*args, **kwargs):
            cls = args[0]
            if not cls.use_cache:
                return func(*args, **kwargs)
            key = wrapper.__cache_key__(*args, **kwargs)
            result = cls.cache.get(key, default=ENOVAL, retry=True)

            if result is ENOVAL:
                result = func(*args, **kwargs)
                if expire is None or expire > 0:
                    cls.cache.set(key, result, expire, tag=tag, retry=True)

            return result

        def __cache_key__(*args, **kwargs):
            """Make key for cache given function arguments."""
            return args_to_key(base, args, kwargs, typed, ignore={0, "self"})

        wrapper.__cache_key__ = __cache_key__
        return wrapper

    return decorator
