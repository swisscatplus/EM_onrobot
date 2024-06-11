FROM yanniscod/vision:latest

WORKDIR /home

# clone using id and token, token may be not reusable and need to be updated
RUN git clone https://Yanniscod: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@github.com/swisscatplus/SwissCat-on_robot.git

RUN pip install pyserial

WORKDIR /home/SwissCat-on_robot/

# Install dependencies and build the workspace
RUN colcon build
