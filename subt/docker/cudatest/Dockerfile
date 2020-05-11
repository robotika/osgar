FROM robotika/subt-base:2020-05-06

ADD subt/docker/cudatest/cudatest_entrypoint.bash src/
ADD subt/docker/cudatest/cudatest3.py src/
ADD subt/docker/cudatest/cloudsim_run.py src/

ENTRYPOINT [ "./src/cudatest_entrypoint.bash" ]

CMD [ "./src/cloudsim_run.py", "/osgar-ws/env/bin/python", "src/cudatest3.py" ]
