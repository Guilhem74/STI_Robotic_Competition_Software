# STI_Robotic_Competition_Software
[EPFL] Semester Project

Github repository for the Software of the STI INTERDISCIPLINARY ROBOT COMPETITION 2019 at EPFL.

Team 5 : Green Eye, Winner of 2019 with 250 points.

Developped by AZZANO Guilhem,ERBACHER Pierre and PIQUET Anthony

Software is mostly splitted in 3 parts:  
  - Bottle Detection
  - Beacon Localization
  - Robot control, high level.

The other materials can be found at :
  - Low-Level control : https://github.com/Guilhem74/STI_Robotic_Competition_Electronics 
  - CAD Design :https://github.com/Guilhem74/STI_Robotic_Competition_Mechanics

A jupyter notebook(Architecture/main.ipynb) represents the list of commands used for the last match of the competition and shouldn't be reused like that.

The class robot (in robot.py) represents most of the architecture and behaviour of the robot, even if it has been designed quickly, and in an not so much object oriented way, it is easily reusable.

The beacon localisation can be found in beacon.py.

Bottle detection can be found in the Jupyter Notebook BottleDetection/main.ipynb

