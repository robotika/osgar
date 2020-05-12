import subprocess
import unittest

class Test(unittest.TestCase):

    def test(self):
        ret = subprocess.run(['git', 'lfs', 'env'], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        self.assertEqual(ret.returncode, 0)
        self.assertEqual(ret.stderr, b"")
        config = ret.stdout.splitlines()[-3:]
        self.assertEqual(config[0], b'git config filter.lfs.process = "git-lfs filter-process"')
        self.assertEqual(config[1], b'git config filter.lfs.smudge = "git-lfs smudge -- %f"')
        self.assertEqual(config[2], b'git config filter.lfs.clean = "git-lfs clean -- %f"')
