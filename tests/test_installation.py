import unittest


class ImportTest(unittest.TestCase):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

    def test_import(self):
        try:
            import vdbfusion
        except ImportError:
            self.fail("vdbfusion not properly installed, please run `make`")

    def test_not_empty(self):
        """Checks that the generated python bindings are not empty."""
        import vdbfusion

        vdir = [x for x in dir(vdbfusion) if not x.startswith("__")]
        self.assertGreater(len(vdir), 1)

    def test_import_pybind(self):
        try:
            import vdbfusion.pybind
        except ImportError:
            self.fail("vdbfusion not properly installed, please run `make`")
