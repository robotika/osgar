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
        for item in self.artf_xyz:
            if item not in self.artf_xyz_accumulated:
                self.artf_xyz_accumulated.append(item)
        self.publish_artf(self.artf_xyz)

    def on_base_station(self, data):
        p = data['artifact_position']
        for i, artf in enumerate(self.artf_xyz_accumulated):
            if distance3D(p, [x/1000.0 for x in artf[1]]) < 0.1:
                self.artf_xyz_accumulated[i][-1] = (data['score_change'] > 0)

    def update(self):
        channel = super().update()  # define self.time
        handler = getattr(self, "on_" + channel, None)
        if handler is not None:
            handler(getattr(self, channel))
        return channel


# vim: expandtab sw=4 ts=4
