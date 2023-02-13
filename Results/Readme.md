Here, we analyze the collected data, and plot some comparation graphs.
We retrieve all the data in the .xlsx files from the simulations. We started the simulation scenario, then, for each tick of CARLA, we collect different tuples of data:
-X and Y coordinates of leader's vehicle
-X and Y coordinates of follower1's vehicle
-X and Y coordinates of follower2's vehicle
-Speed of leader's vehicle
-Speed of follower1's vehicle
-Speed of follower2's vehicle.
We collect all this data 3 times for each scenario: normal, under attack with rules, under attack w/o rules.
In this way, we can plot comparative graphs with three dataset, in order to highlight the differences and present the rules'results.

Then, we also plot the CDF graph for each one of the attack scenarios. We calculate the point to point distance (from the normal scenario) for every simulation tick.

