FROM osrf/subt-virtual-testbed:subt_solution_latest

RUN sudo apt-get update && sudo apt-get install -y \
    python-bcrypt \
    python-pip \
    python-pip-whl \
    python-virtualenv \
 && sudo rm -rf /var/lib/apt/lists/*

COPY subt/docker/loadtest/main.py subt/docker/loadtest/loadtest_entrypoint.bash ./

ENTRYPOINT ["./loadtest_entrypoint.bash"]

CMD ["python", "main.py", "100"]
