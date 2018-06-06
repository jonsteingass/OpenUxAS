# GPS Denied Example

This is an example of running UxAS service that communicates to the Amase simulation in order test the vision based navigation service.
## Files:

* `cfg_GpsDenied_LeftTurn.xml` - This is a UxAS config file where a vehicle turns left in the GPS denied zone.
* `cfg_GpsDenied_Straight.xml` - This is a UxAS config file where a vehicle maintains a straight course through a gps denied zone.
* `runAMASE_GpsDenied.sh` - This script executes AMASE with its Gps Denied config file.
* `runUxAS_GpsDenied_Straight.sh` - This script executes UxAS with the cfg_GpsDenied_Straight.xml config file.
* `runUxAS_GpsDenied_LeftTurn.sh` - This is a script that will execute UxAS with the cfg_GpsDeniedLeftTurn.xml config file.
* `Scenario_GpsDenied.xml` - This is the AMASE configuration file for the GPS-denied scenario.
* `MessagesToSend/` - This is where the messages sent by the SendMessagesService are located.
* `MessagesToSend/GpsDeniedZone.xml` - This is the GpsDeniedZone message sent from the UxAS SendMessagesService.
* `MessagesToSend/InitialStatesAndConfiguration/` - This folder which stores the initial states and configurations of the vehicle used in this simulation.
* `MessagesToSend/InitialStatesAndConfigurations/AirVehicleConfiguration_V400.xml` - This is the AirVehicleConfiguration message sent from the SendMessagesService. This is identical to the AirVehicleConfiguration in the AMASE configuration file "Scenario_GpsDenied.xml".
* `MessagesToSend/InitialStatesAndConfigurations/AirVehicleState_V400.xml` - This is the initial AirVehicleState message sent from the SendMessagesService. This is identical to the AirVehicleState in the AMASE configuration file "Scenario_GpsDenied.xml".
* `MessagesToSend/Tasks/` - This folder stores the tasks used in this simulation
* `LoiterMissionCommand.xml` - This is a mission command that commands the vehicle to loiter over the first point in the simulation.
* `GPSDenied_LeftTurn_MissionCommand.xml` - This mission command commands the vehicle to a waypoint inside the GPS denied zone, then to another waypoint outside of the GPS denied zone. The location of the waypoints makes the vehicle turn left.
* `GPSDenied_Straight_MissionCommand.xml` - This mission command commands the vehicle to a waypoint inside the GPS denied zone, then to another waypoint outside of the GPS denied zone. The location of the waypoints makes the vehicle approximate a straight heading. 

## Running the Example:
1. open a terminal window in the directory: "examples/07_GpsDenied/"
2. enter the command: `./runAMASE_GpsDenied.sh`
3. open another terminal window in the directory: "examples/07_GpsDenied/"
4. enter the command: `./runUxAS_GpsDenied_TurnLeft.sh` or `./runUxAS_GpsDenied_Straight.sh`
5. start the Amase simulation (i.e. push the play button)

### What Happens?
* When the Amase simulation starts, a UAV will be initialized and begin flying south at latitude 39.3446361111, longitude -86.03.
* .3 seconds after UxAS starts, the GPS Denied zone will appear as a magenta square north of the UAV
* 1 second after UxAS starts, the vehicle will navigate and begin to lotier around a point located at latitude 39.344629275, longitude -86.038240601.
* 3 minutes and 50 second in, the vehicle will be command to a waypoint within the gps denied zone at latitude 39.344627567, longitude -86.060963844, followed by another point outside of the GPS denied zone.
* Once the vehicle exits the GPS denied zone, it will be send a request to loiter around the first waypoint outside of the GPS denied zone from the VisionBasedNavService
