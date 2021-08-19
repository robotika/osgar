"""
  Report and filter shared detected artifacts

The artifacts are reported to "DARPA base station" as [type, position]. The "DARPA base station" answers with
message, where only "score_changed" has value 1 if it was correctly localized artifact within 5 meters radius (3D).

The implemented solution is using shared array of items [type, position, source, scored]. Type and 3D position
is the same as for DARPA report. "source" is identification of robot which detected artifact (for example "A1300L").
"scored" has states: True/False/None. This corresponds to base station answer "score_changed"=1/"score_changed"=0/
"unknown=no answer".

If ArtifactReporter receives message with scored=None then first verifies if identical type and position is already
in the pool. If yes, then received message is ignored. There still can be another report already confirmed as
scored=True so it goes through identical types and if position is within RADIUS then it is rejected (it would need
4th type - detected but ignored). Note, that in the rules "SubT_Challenge_Cave_Rules.pdf" is nothing about proximity
of placed artifacts, so there can be two backpacks side by side. RADIUS is only "probabilistic", that it is rather
duplicity than two artifacts placed side by side.

Artifacts received with scored=True/False replace only scored=None in the pool. Not matching values should assert.

There is a race condition between reporting artifact and spreading this info to other robots. It has to be done
immediately otherwise other robots may use this time to report independently new found artifact. The only solution
would be teambase, but then it would be the single point of failure.
"""
from osgar.node import Node
from subt.trace import Trace, distance3D


RADIUS = 4.0  # there are probably not two artifacts within sphere of this radius
RADIUS_FALSE = 2.0  # reported artifact was not correct so do not repeat it within this radius


class ArtifactReporter(Node):
    def __init__(self, config, bus):
        super().__init__(config, bus)
        bus.register("artf_cmd", "artf_all")
        self.repeat_report_sec = config.get('repeat_report_sec')
        self.artf_xyz = []
        self.artf_xyz_accumulated = []  # items are [type, position, source, flag]
                                        # position = [x, y, z], flag(scored) = True/False/None
        self.verbose = False

    def publish_artf(self, artf_xyz):
        count = 0
        for artf_type, pos, src, scored in self.select_artf_for_report(artf_xyz):
            if count == 0 and self.verbose:
                print(self.time, "DETECTED:")
            count += 1
            ix, iy, iz = pos
            if self.verbose:
                print(" ", artf_type, ix/1000.0, iy/1000.0, iz/1000.0)
            s = '%s %.2f %.2f %.2f\n' % (artf_type, ix/1000.0, iy/1000.0, iz/1000.0)
            self.publish('artf_cmd', bytes('artf ' + s, encoding='ascii'))
        if count > 0 and self.verbose:
            print('report completed')

    def select_artf_for_report(self, artf_xyz):
        """
        There are cases when two or more robots discover artefact independently and they do not
        have the base answer yet. Never-the-less only one representative (team identical) should
        be selected and reported.
        """
        # first sort all known artifacts into three groups
        scored_true, scored_false, scored_unknown = [], [], []
        for item in artf_xyz:
            artf_type, pos, src, scored = item
            if scored is True:
                scored_true.append(item)
            elif scored is False:
                scored_false.append(item)
            else:
                assert scored is None, scored
                scored_unknown.append(item)

        # remove all unknown too close to already successfully scored artifacts
        tmp = []
        for item in scored_unknown:
            _, pos, _, _ = item
            for _, pos2, _, _ in scored_true:
                # we want to filter out also other artifacts of different type (i.e. probably wrongly recognized?)
                if distance3D(pos, pos2)/1000.0 < RADIUS:
                    break
            else:
                tmp.append(item)
        scored_unknown = tmp

        # now remove all already reported close to false reports
        tmp = []
        for item in scored_unknown:
            artf_type, pos, _, _ = item
            for artf_type2, pos2, _, _ in scored_false:
                if artf_type == artf_type2 and distance3D(pos, pos2)/1000.0 < RADIUS_FALSE:
                    # close to wrongly reported artifact with the same type
                    break
            else:
                tmp.append(item)
        scored_unknown = tmp

        # finally pick only one representative (any, just ordered)
        # ... and handle other reports once this is confirmed
        # TODO confirmation from base can be lost several times ... is it OK?
        if len(scored_unknown) > 0:
            return [min(scored_unknown)]
        else:
            return []

    def nearest_scored_artifact(self, artf_type, position):
        # TODO remove 1000x scaling to millimeters - source of headache
        dist = [distance3D(position, artf[1]) for artf in self.artf_xyz_accumulated
                if artf[0] == artf_type and artf[-1] is True]
        if len(dist) == 0:
            return None  # no nearest for given filter
        return min(dist)/1000.0  # due to mm scaling

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
            atype, pos, src, scored = item
            if [atype, pos] not in [arr[:2] for arr in self.artf_xyz_accumulated]:
                # new entry with unique (type, position)
                nearest = self.nearest_scored_artifact(atype, pos)
                if nearest is None or nearest > RADIUS:
                    self.artf_xyz_accumulated.append(item)
            elif item not in self.artf_xyz_accumulated:
                # existing entry but with different flag (or source)
                i = [arr[:2] for arr in self.artf_xyz_accumulated].index(item[:2])
                if scored is True or scored is False:
                    assert self.artf_xyz_accumulated[i][-1] is None, self.artf_xyz_accumulated[i]
                    self.artf_xyz_accumulated[i] = item
                else:
                    pass  # item.score is None, but we already know the answer -> ignore
        self.publish_artf(self.artf_xyz_accumulated)

    def on_base_station(self, data):
        p = data['artifact_position']
        dist = [distance3D(p, [x/1000.0 for x in artf[1]]) for artf in self.artf_xyz_accumulated]
        if len(dist) > 0 and min(dist) < 0.1:
            # i.e. it is our report and we have it in the list ... I would almost assert it
            min_i = dist.index(min(dist))
            was_unknown = self.artf_xyz_accumulated[min_i][-1] is None
            self.artf_xyz_accumulated[min_i][-1] = (data['score_change'] > 0)
            # TODO? check type - we decided to ignore it for Cave Circuit
            if was_unknown:
                self.publish('artf_all', self.artf_xyz_accumulated)  # broadcast new update
                # trigger sending next artifact if there is any
                self.publish_artf(self.artf_xyz_accumulated)

    def update(self):
        channel = super().update()  # define self.time
        handler = getattr(self, "on_" + channel, None)
        if handler is not None:
            handler(getattr(self, channel))
        return channel


# vim: expandtab sw=4 ts=4
