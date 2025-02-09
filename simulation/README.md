Simulation Utilities
====================

* dh.html - Interactive Visualization of Denavit-Hartenberg Parameters & Coordinates
  See [live version](https://mathcs.holycross.edu/~kwalsh/dh.html)
  or simply open this file in a browser.
  This simulates 5 revolute (rotating) joints, using the "classic" DH
  parameters.
  - Example: [xArm with resting position outstretched vertically](https://mathcs.holycross.edu/~kwalsh/dh.html?dh=%5B%7B%22theta_offset%22%3A0%2C%22alpha%22%3A90%2C%22a%22%3A0%2C%22d%22%3A2%2C%22min%22%3A-180%2C%22max%22%3A180%7D%2C%7B%22theta_offset%22%3A90%2C%22alpha%22%3A0%2C%22a%22%3A2%2C%22d%22%3A0%2C%22min%22%3A-90%2C%22max%22%3A90%7D%2C%7B%22theta_offset%22%3A0%2C%22alpha%22%3A0%2C%22a%22%3A2%2C%22d%22%3A0%2C%22min%22%3A-90%2C%22max%22%3A90%7D%2C%7B%22theta_offset%22%3A90%2C%22alpha%22%3A90%2C%22a%22%3A0%2C%22d%22%3A0%2C%22min%22%3A-90%2C%22max%22%3A90%7D%2C%7B%22theta_offset%22%3A0%2C%22alpha%22%3A0%2C%22a%22%3A0%2C%22d%22%3A2%2C%22min%22%3A-90%2C%22max%22%3A90%7D%5D)
  - Example: [xArm with resting position outstretched horizontally](https://mathcs.holycross.edu/~kwalsh/dh.html?dh=%5B%7B%22theta_offset%22%3A0%2C%22alpha%22%3A90%2C%22a%22%3A0%2C%22d%22%3A2%2C%22min%22%3A-180%2C%22max%22%3A180%7D%2C%7B%22theta_offset%22%3A0%2C%22alpha%22%3A0%2C%22a%22%3A2%2C%22d%22%3A0%2C%22min%22%3A0%2C%22max%22%3A180%7D%2C%7B%22theta_offset%22%3A0%2C%22alpha%22%3A0%2C%22a%22%3A2%2C%22d%22%3A0%2C%22min%22%3A-90%2C%22max%22%3A90%7D%2C%7B%22theta_offset%22%3A90%2C%22alpha%22%3A90%2C%22a%22%3A0%2C%22d%22%3A0%2C%22min%22%3A-90%2C%22max%22%3A90%7D%2C%7B%22theta_offset%22%3A0%2C%22alpha%22%3A0%2C%22a%22%3A0%2C%22d%22%3A2%2C%22min%22%3A-90%2C%22max%22%3A90%7D%5D)

* server.py, arm.{html,css,js} - Interacive Simulation of xArm 6dof Robotic Arm
  See below for instructions on installing, running, and using the server.

* client.py - A command-line interface for interacting with server.py
  This is mostly only useful for debugging.

* simulation.py - Allows python kinematics code to interact with server.py
  The python code in ../movement/ was meant for the actual arm, but using this
  file can allow it to work with the simulated arm instead. See below for
  instructions.

## Using server.py and simulation.py

1. Create and activate a python venv, and install python dependencies, e.g.

    python3 -m venv ~/xarm-python-venv         # create python virtual environment
    source ~/xarm-python-venv/bin/activate    # activate it
    pip install -r requirements.txt           # install dependencies

2. Run the server: `./server.py`
3. Open the interactive page at the URL it prints, e.g.
   [http://localhost:8000](http://localhost:8000/arm.html)
4. Either use the interactive page to move the arm. Or, in a separate command
   line window, run the kinematics controller (after having changed it to
   connect to the simulated arm using simulation.py), e.g.
   `python3 ../movement/controller.py goto 0.150 0.050 0.940`

In order for the kinematics controller code to connect to the simulated arm, one
small change must be made to the controller code. Where the kinematics code
connects to the real arm using something like:

        from connection import Connection
        ...
        connection = Connection()
        ...

It should be changed to instead to connect to the simulated arm like so:

        from simulation import Simulation
        ...
        connection = Simulation()
        ...

This requires that simulation.py be copied or linked to the ../movement/ folder.
