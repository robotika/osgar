FROM osrf/subt-virtual-testbed:subt_solution_latest

SHELL ["/bin/bash", "-c"]

RUN /usr/bin/python3 -m pip install --user --no-cache-dir \
  "msgpack==1.0.0" \
  "numpy==1.18.2" \
  "opencv-python==4.2.0.32" \
  "pyzmq==19.0.0"

COPY --chown=developer:developer . ./osgar/

RUN source /opt/ros/melodic/setup.bash && ./osgar/subt/script/sync.sh

ENTRYPOINT ["./osgar/subt/docker/robotika/entrypoint.bash"]

CMD ["./osgar/subt/docker/robotika/run_solution.bash"]
