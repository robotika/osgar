FROM osrf/subt-virtual-testbed:subt_solution_latest

SHELL ["/bin/bash", "-c"]

RUN /usr/bin/python3 -m pip install --user --no-cache-dir \
  "msgpack==1.0.0" \
  "numpy==1.18.2" \
  "opencv-python==4.2.0.32" \
  "pyzmq==19.0.0" \
  "torch==1.4.0"

RUN sudo wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu1804/x86_64/7fa2af80.pub -q -O /etc/apt/trusted.gpg.d/nvidia_cuda.asc && \
    sudo bash -c 'echo "deb https://developer.download.nvidia.com/compute/cuda/repos/ubuntu1804/x86_64 /" > /etc/apt/sources.list.d/cuda.list' && \
    sudo bash -c 'echo "deb https://developer.download.nvidia.com/compute/machine-learning/repos/ubuntu1804/x86_64 /" > /etc/apt/sources.list.d/nvidia-ml.list' 

# For libraries in the cuda-compat-* package: https://docs.nvidia.com/cuda/eula/index.html#attachment-a
RUN sudo apt-get update && sudo apt-get install -y --no-install-recommends cuda-cudart-10-1=10.1.243-1 && \
    sudo ln -s cuda-10.1 /usr/local/cuda && \
    sudo rm -rf /var/lib/apt/lists/*

# Required for nvidia-docker v1
RUN sudo bash -c 'echo "/usr/local/nvidia/lib" >> /etc/ld.so.conf.d/nvidia.conf' && \
    sudo  bash -c 'echo "/usr/local/nvidia/lib64" >> /etc/ld.so.conf.d/nvidia.conf'

ENV PATH /usr/local/nvidia/bin:/usr/local/cuda/bin:${PATH}
ENV LD_LIBRARY_PATH /usr/local/nvidia/lib:/usr/local/nvidia/lib64:${LD_LIBRARY_PATH}

# nvidia-container-runtime
ENV NVIDIA_VISIBLE_DEVICES all
ENV NVIDIA_DRIVER_CAPABILITIES compute,utility
ENV NVIDIA_REQUIRE_CUDA "cuda>=10.1 brand=tesla,driver>=384,driver<385 brand=tesla,driver>=396,driver<397 brand=tesla,driver>=410,driver<411"

RUN cd /home/developer/ && head -n -2 .bashrc > .bashrc.new && mv .bashrc.new .bashrc

COPY --chown=developer:developer . ./osgar/

RUN source /opt/ros/melodic/setup.bash && ./osgar/subt/script/sync.sh

ENTRYPOINT ["./osgar/subt/docker/robotika/entrypoint.bash"]

CMD ["./osgar/subt/docker/robotika/run_solution.bash"]
