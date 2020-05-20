FROM robotika/subt-base:2020-05-18

ENTRYPOINT ["./src/osgar/subt/docker/robotika/entrypoint.bash"]

CMD ["./src/osgar/subt/docker/robotika/run_solution.bash"]

# support `docker exec -it <container> bash
RUN echo "source /osgar-ws/devel/setup.sh" >> ~/.bashrc

# copy whole build context
COPY . ./src/osgar/

RUN /osgar-ws/env/bin/pip install -e src/osgar/

RUN source /opt/subt/setup.bash && catkin_make -DCMAKE_BUILD_TYPE=Release
