"""
  Report detected artifacts
"""
from osgar.node import Node
from subt.trace import Trace, distance3D


class ArtifactReporter(Node):
    def __init__(self, config, bus):
        super().__init__(config, bus)
        bus.register("artf_cmd", "artf_all")
        self.repeat_report_sec = config.get('repeat_report_sec')
        self.artf_xyz = []
        self.artf_xyz_accumulated = []  # items are [type, position, source, flag]
                                        # position = [x, y, z], flag(scored) = True/False/None

    def publish_artf(self, artf_xyz):
        count = 0
        for artf_type, pos, src, scored in artf_xyz:
            if scored is None:
                if count == 0:
                    print(self.time, "DETECTED:")
                count += 1
                ix, iy, iz = pos
                print(" ", artf_type, ix/1000.0, iy/1000.0, iz/1000.0)
                s = '%s %.2f %.2f %.2f\n' % (artf_type, ix/1000.0, iy/1000.0, iz/1000.0)
                self.publish('artf_cmd', bytes('artf ' + s, encoding='ascii'))
        if count > 0:
            print('report completed')

    def on_sim_time_sec(self, data):
        if self.repeat_report_sec is None or self.sim_time_sec % self.repeat_report_sec != 0:
            return
        if len(self.artf_xyz) == 0:
            return
        # publish all artifacts only on regular update (to avoid LoRA cross-talk)
        self.publish_artf(self.artf_xyz_accumulated)
        self.publish('artf_all', self.artf_xyz_accumulated)

    def on_artf_xyz(self, data):
        for item in data:
            if item[:2] not in [arr[:2] for arr in self.artf_xyz_accumulated]:
                # new entry with unique (type, position)
                self.artf_xyz_accumulated.append(item)
            elif item not in self.artf_xyz_accumulated:
                # existing entry but with different flag (or source)
                i = [arr[:2] for arr in self.artf_xyz_accumulated].index(item[:2])
                if item[-1] is True or item[-1] is False:
                    assert self.artf_xyz_accumulated[i][-1] is None, self.artf_xyz_accumulated[i]
                    self.artf_xyz_accumulated[i] = item
                else:
                    pass  # item.score is None, but we already know the answer -> ignore
        self.publish_artf(data)

    def on_base_station(self, data):
        p = data['artifact_position']
        dist = [distance3D(p, [x/1000.0 for x in artf[1]]) for artf in self.artf_xyz_accumulated]
        if len(dist) > 0 and min(dist) < 0.1:
            # i.e. it is our report and we have it in the list ... I would almost assert it
            min_i = dist.index(min(dist))
            self.artf_xyz_accumulated[min_i][-1] = (data['score_change'] > 0)
            # TODO? check type

    def update(self):
        channel = super().update()  # define self.time
        handler = getattr(self, "on_" + channel, None)
        if handler is not None:
            handler(getattr(self, channel))
        return channel


# vim: expandtab sw=4 ts=4
