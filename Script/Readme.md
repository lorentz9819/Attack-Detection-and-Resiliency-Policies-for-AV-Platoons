SCRIPT

Here we report the script used for platoon attack simulation.
Each folder have 3 scripts, cloud.py, platooning.py and <scenario_name_>final.py.
The one that has to be executed is the <scenario_name_>final.py, while the other are classes used to construct and manage the platoon.

In <scenario_name_>final.py, we define the platoon spawn position, the sensors' spawn and all the graphical details of the simulation.

In cloud.py, we define the functions used to communicate data between platoon members.

In platooning.py, we define all the functions used to make the platoon able to move, to follow the data received and to elaborate them.

At the end of this script, we define the attacks tested. They focused on the data communication between leader and followers, by modifying some passed parameters.
