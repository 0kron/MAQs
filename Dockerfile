FROM althack/ros2:humble-full

ENV TZ=America/Mexico_City
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone
USER root

# Shell y herramientas
RUN apt update && apt install -y zsh git wget curl \
    openssh-client iputils-ping net-tools iproute2 \
    && rm -rf /var/lib/apt/lists/*

# Oh-My-Zsh
RUN wget https://raw.githubusercontent.com/ohmyzsh/ohmyzsh/master/tools/install.sh && \
    chmod +x install.sh && ./install.sh

# ROS2 DDS
RUN apt update && apt install -y \
    ros-humble-rmw-cyclonedds-cpp \
    ros-humble-rosidl-generator-dds-idl \
    && rm -rf /var/lib/apt/lists/*

# Clang
RUN wget https://apt.llvm.org/llvm.sh && chmod +x llvm.sh && ./llvm.sh 18 all

# Configuración SSH
RUN mkdir -p /root/.ssh && chmod 700 /root/.ssh

# Workspace
WORKDIR /workspace
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.zshrc
RUN echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> ~/.zshrc

CMD [ "zsh" ]
