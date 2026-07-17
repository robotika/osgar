
try:
    from importlib.metadata import version
except ImportError:
    try:
        from importlib_metadata import version
    except ImportError:
        version = lambda name: "0.0.0"

try:
    __version__ = version("osgar")
except Exception:
    __version__ = "0.0.0"

from osgar.lib import create_load_tests
load_tests = create_load_tests(__file__)
