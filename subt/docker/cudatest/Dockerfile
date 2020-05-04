FROM robotika/subt-base:2020-05-04

ADD subt/docker/cudatest/cudatest_entrypoint.bash src/
ADD subt/docker/cudatest/cudatest3.py src/
ADD subt/docker/cudatest/rospywrapper.py src/

ENTRYPOINT [ "./src/cudatest_entrypoint.bash" ]

CMD [ "./src/rospywrapper.py" ]
