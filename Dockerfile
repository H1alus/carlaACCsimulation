FROM python:3.8-bullseye
ENV PYTHONUNBUFFERED 1

WORKDIR /code

# Copying the requirements, this is needed because at this point the volume isn't mounted yet
COPY requirements.txt /code/

# Installing requirements, if you don't use this, you should.
# More info: https://pip.pypa.io/en/stable/user_guide/
RUN pip install -r requirements.txt

RUN apt-get update && apt-get install -y --no-install-recommends \
      bzip2 \
      g++ \
      git \
      graphviz \
      libgl1-mesa-glx \
      libhdf5-dev \
      openmpi-bin \
      wget \
      python3-tk && \
    rm -rf /var/lib/apt/lists/*
RUN (apt-get autoremove -y; \
     apt-get autoclean -y)
     
WORKDIR /app
COPY . /app

ENV QT_X11_NO_MITSHM=1
CMD ["python", "main.py"]