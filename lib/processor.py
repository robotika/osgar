#!/usr/bin/python
"""
  Parallel processor/worker/filter for collected sensor data
"""
from multiprocessing import Pool


class Processor:
    def __init__(self, process_fn):
        self.pool = Pool(processes=1)
        self.process_fn = process_fn
        self.processing = None

    def get_result(self):
        if self.processing is None or not self.processing.ready():
            return None
        ret = self.processing.get()
        self.processing = None
        return ret

    def start(self):
        pass  # only legacy function

    def requestStop(self):
        self.pool.close()

    def push_back(self, data):
        print "Processor", data
        if self.processing is None:
            self.processing = self.pool.apply_async(self.process_fn, (data,))
        else:
            print "Skipped", data

# vim: expandtab sw=4 ts=4 

