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

A Dockerfile is provided. A container can be created by running the following commands. Note that ``sudo`` might be needed depending on how you've set up your Docker engine.

.. code:: bash

    $ git clone https://github.com/f1tenth/f1tenth_gym.git
    $ cd f1tenth_gym
    $ git checkout exp_py
    $ docker build -t f1tenth_gym -f Dockerfile .
    $ docker run -it --name=f1tenth_gym_container --rm f1tenth_gym

.. image:: assets/pip_logo.svg

Using pip
---------------

The environment is a Python package, and only depends on ``numpy``, ``numba``, ``Pillow``, ``gym``, ``pyyaml``, ``pyglet``, and ``pyopengl``. You can install the package via pip:

.. code:: bash

    $ pip3 install git+https://github.com/f1tenth/f1tenth_gym.git
