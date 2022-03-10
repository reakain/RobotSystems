# Final Project - Shoulder Parrot

## How to set up

 1. git clone repo to home directory
 2. Copy parrot folder to armpi_fpv src folder
    `cp -R ~/RobotSystems/armProject/parrot ~/armpi_fpv/src/`
 3. From armpi_fpv (ros workspace) build the new code setup
    `cd ~/armpi_fpv && catkin_make`
 4. Source the new info to your path
    `source ~/armpi_fpv/devel/setup.bash`
 5. Call the parrot service to start
    `rosservice call /parrot/enter "{}"`
    

## Order of Operations

 - Look for face
 - Clacky-clack the grabbers at the face
 - Look for cube
 - Take cube (or clacky clack again at time out?, extra work)
 - Pull cube to self
 - Drop cube!
 - Repeat from start


 
