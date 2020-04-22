FROM robotika/subt-base:2020-04-21

ENTRYPOINT ["./src/subt/docker/robotika/entrypoint.bash"]

CMD ["./src/subt/docker/robotika/run_solution.bash"]

# support `docker exec -it <container> bash
RUN echo "source /osgar-ws/devel/setup.sh" >> ~/.bashrc

COPY --chown=root:root . ./src/

RUN /osgar-ws/env/bin/pip install -e src/

RUN source /opt/subt/setup.bash && catkin_make -DCMAKE_BUILD_TYPE=Release


