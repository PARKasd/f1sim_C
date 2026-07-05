.. raw:: html

   <style>
   .rst-content .section>img {
       width: 30px;
       margin-bottom: 0;
       margin-top: 0;
       margin-right: 15px;
       margin-left: 15px;
       float: left;
   }
   </style>

Installation
=================
``f1tenth_gym`` is a Python package with a C++ simulation core, built automatically on install (a C++17 compiler is required). We provide two ways to set up the environment.

.. image:: assets/docker_logo.png

Using docker
----------------

A Dockerfile and a ``docker-compose.yml`` are provided; the image includes ROS 2 and a browser-accessible GUI (noVNC). Note that ``sudo`` might be needed depending on how you've set up your Docker engine.

.. code:: bash

    $ git clone https://github.com/PARKasd/f1sim_C.git
    $ cd f1sim_C
    $ docker compose up -d
    $ docker exec -it f1tenth_gym-sim-1 /bin/bash

.. image:: assets/pip_logo.svg

Using pip
---------------

The environment is a Python package, and only depends on ``numpy``, ``numba``, ``Pillow``, ``gym``, ``pyyaml``, ``pyglet``, and ``pyopengl``. Clone the repository and run the install script (it also builds the optional ROS 2 bridge when ROS 2 is installed):

.. code:: bash

    $ git clone https://github.com/PARKasd/f1sim_C.git
    $ cd f1sim_C
    $ ./install.sh
