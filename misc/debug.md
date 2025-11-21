### debug line for the action servers and the action handler in context of non twice callability of the cation servers

debug decision after trying various things from various llms, and Persons:\

raise a action server and handler from ground zero and add things until unwanted behaiviour returns.

this decision was made, because it already worked woith another action server and i really like to process the logical things in a timer callback to losen from any other callback to especially unforce the logic to any other behaivior than a deterministic time.

first i wrote a new package adapted all imports also these ones for the future and added all files. I wrote a simple count to ten acrtion server and a client which calls it forever. I used a flag and all corresponding function chains i used in the non wirking programm but i especially used variable naming from the Programm that worked because the function chains where the same but the variable naming was different.

so far this is the only point where i changed anythign from the non working server.
-> if i can rebuild the completly same action server ass the none working one but with the var-names of the functioning server this would be the issue.

to start at rock bottom i only started the action server and called the action goal manually, also twice or more often and everything worked fine

by starting the action handler also everything worked fine and the goal is called repeatedly

at this state i added all other listeners and publishers. this code then si executed on the robot to (hopefullyt) prove there is no race condition betwene node spining and listening and so on...



