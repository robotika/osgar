FROM ubuntu:20.04

RUN apt-get update \
    && DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
        python3-rospy \
        python3-std-srvs \
    && apt-get clean

ADD subt/docker/rospy/cmd.py .

ENTRYPOINT []

CMD [ "/usr/bin/python3" , "cmd.py" ]
#CMD [ "/bin/bash" ]
