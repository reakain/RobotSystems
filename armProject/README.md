# Final Project - Shoulder Parrot

## How to set up

 1. git clone repo to home directory
 2. Run setup shell script
    `cd ~/RobotSystems/armProject/setup.sh`
 3. Call the parrot service to start
    `rosservice call /parrot/enter "{}"`
 4. Set Running
    `rosservice call /parrot/set_running "data: true"`
 5. Set Target
    `rosservice call /parrot/set_target "color:
- 'red'
- 'green'
- 'blue'"`



## Order of Operations

 - Look for face
 - Clacky-clack the grabbers at the face
 - Look for cube
 - Take cube (or clacky clack again at time out?, extra work)
 - Pull cube to self
 - Drop cube!
 - Repeat from start



