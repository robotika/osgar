#!/usr/bin/env python

if __name__ == "__main__":
    import unittest
    import unittest.loader
    testLoader = unittest.loader.TestLoader()
    from pathlib import Path
    top_level_dir = Path(__file__).parent
    testLoader._top_level_dir = top_level_dir
    unittest.main(module=None, testLoader=testLoader)
