// ===============================================================================
// Authors: AFRL/RQQA
// Organization: Air Force Research Laboratory, Aerospace Systems Directorate, Power and Control Division
// 
// Copyright (c) 2017 Government of the United State of America, as represented by
// the Secretary of the Air Force.  No copyright is claimed in the United States under
// Title 17, U.S. Code.  All Other Rights Reserved.
// ===============================================================================

/* 
 * File:   VisionBasedNavService.h
 * Authors: Jon & Caine
 *
 * Created on February 12, 2018, 8:00 AM
 */

#ifndef VISON_BASED_NAV_SERVICE_H
#define VISION_BASED_NAV_SERVICE_H



#include "ServiceBase.h"
#include <string>
#include <vector>
#include "CallbackTimer.h"
#include "afrl/cmasi/EntityState.h"
#include "uxas/messages/uxnative/SafeHeadingAction.h"
#include "afrl/cmasi/AutomationRequest.h"
#include "TypeDefs/UxAS_TypeDefs_Timer.h"
#include "afrl/cmasi/CMASI.h"
#include "../../src/VisilibityLib/visilibity.h"
#include "afrl/cmasi/MissionCommand.h"
#include "afrl/cmasi/Waypoint.h"

namespace uxas {
    namespace service {

        /*! \class VisionBasedNavService
            \brief This is a service that will be used to perform vision based navigation commands.
         *   This class will implement the vision based algorithm and output SafeHeadingActions.

         *  
         * 
         * Configuration String: <Service Type="VisionBasedNavService" VehicleID="0" MinWaypointDistance="150" LoiterRadius="350"/>
         *
         * Parameters
         *  - VehicleID: when non-zero only vision based navigation will be performed on that vehicle when a condition is evaluated
         *  - MinWaypointDistance: The minimum distance between the vehicle in meters and the waypoint before switching targeting the next waypoint
         *  - LoiterRadius: The loiter radius for the first point outside of the GPS denied zone (will loiter there until new task starts up)
         * 
         * Subscribed Messages:
         *
         * - uxas::projects::teas::GpsDeniedZone
         *		This message is received when a GPS denied zone is recognized by UxAS
         *		Contains:
         *				-ZoneID:		   A globally unique reference number for this zone (int64)[None]
         *
         *				-MinAltitude:	   Lower altitude bound for the zone (real32)[meter]
         *
         *				-MinAltitudeType:  Altitude type for min altitude (AltitudeType)[None]
         *
         *				-MaxAltitude:	   Maximum altitude for operations (real32)[meter]
         *
         *				-MaxAltitudeType:  Altitude type for max altitude (AltitudeType)[None]
         *
         *				-AffectedAircraft: A list of aircraft IDs that this zone applies to. 
         *								   If the list is empty, then it is assumed that the boundary applies to all aircraft (int64[])[None]
         *
         *				-StartTime:		   Time at which this zone becomes active. Time datum is defined by the application,
         *								   but unless otherwise specified is milliseconds singe 1 Jan 1970 (int64)[milliseconds]
         *
         *				-EndTime:		   Time at which this zone becomes inactive. Time datum is defined by the application, 
         *								   but unless otherwise specified is milliseconds since 1 Jan 1970 (int64)[milliseconds]
         *
         *				-Padding:		   Buffer to add/subtract around the border of the zone (real32)[meters]
         *
         *				-Label:			   Optional label for this zone (string)[None]
         *
         *				-Boundary:		   Geometry object describing the boundary. This boundary is 2-dimensional. The zone boundary is defined 
         *								   as an extrusion of this boundary from MinAltitude to MaxAltitude. A valid zone must define a boundary (AbstractGeometry)[None]
         *
         *				-GPSSignalType:	   The type of GPS signal. (GPSSignalType)[None]
         *
         *  - afrl::cmasi::EntityState
         *		This message contains the current state of the vehicle. For our purposes this will be cast to an AirVehicleState.
         *		Refer to the details of AirVehicleState and EntityState for more information.
         *		Details found in the following path in the OpenUxAS directory: doc/LMCP/index.html
         *		Contains:
         *				-ID:			   A unique ID for this entity. IDs should be greater than zero (int64)[None]
         *
         *				-u:				   Velocity in the body x-direction (positive out nose) (real32)[meter/sec]
         *				
         *				-v:				   Velocity in the body y-direction (positive out right wing) (real32)[meter/sec]
         *				
         *				-w:				   Velocity in the body z-direction (positive downward) (real32)[meter/sec]
         *				
         *				-udot:			   Accelaration in the body x-direction (positive out nose) (real32)[meter/sec/sec]
         *				
         *				-vdot:			   Acceleration in the body y-direction (positive out right wing) (real32)[meter/sec/sec]
         *				
         *				-wdot:			   Acceleration in the body z-direction (positive downward) (real32)[meter/sec/sec]
         *				
         *				-Heading:		   Angle between true North and the projection of the body x-axis in the North-East plane (real32)[degree]
         *				
         *				-Pitch:			   Pitch of vehicle around body y-axis (positive upwards) (real32)[degree]
         *				
         *				-Roll:			   Roll angle of the vehicle around body x-axis (positive right wing down) (real32)[degree]
         *				
         *				-p:				   roll-rate of vehicle (angular velocity around body x-axis). Positive right-wing down (real32)[degree/sec]
         *				
         *				-q:				   pitch rate of the vehicle (angular velocity around body y-axis). Positive nose-up. (real32)[degree/sec]
         *				
         *				-r:				   yaw rate of the vehicle (angular velocity around body z-axis). Positive nose right. (real32)[degree/sec]
         *				
         *				-Course:		   Course/Groundtrack angle of the entity referenced to true North (real32)[degrees]
         *				
         *				-GroundSpeed:	   Currenty entity ground speed (real32)[meter/sec]
         *				
         *				-Location:		    The perceived entity location. A valid EntityState must include location (Location3D)[None]
         *				
         *				-EnergyAvailable:   The available energy remaining, expressed in terms of the percentage of maximum capacity (real32)[%]
         *				
         *				-ActualEnergyRate:  The consumption rate of available energy, expressed in terms of the percentage of maximum capacity used per second (real32)[%/sec]
         *				
         *				-PayloadStateList:  A list of states for any onboard payloads (PayloadState[])[None]
         *				
         *				-CurrentWaypoint:   The ID of the current waypoint. Only valid if the vehicle is in waypoint following mode. (int64)[None]
         *				
         *				-Mode:				The current mode for this vehicle. (NavigationMode)[None]
         *				
         *				-AssociatedTasks:	Tasks that this entity is currently executing. The task number should coincide with the task number in the task request. (int64)[None]
         *				
         *				-Time:				Timestamp of this data. Time datum is defined by the application, but unless otherwise specified is milliseconds since Jan 1st 1970. (int64)[millisecond]
         *				
         *				-Info:				A list that maps keys to values for the inclusion of extra, custom information about this entity (KeyValuePair[])[None]
         * 
         *  - afrl::cmasi::MissionCommand
         *              This message contains the waypoints a vehicle is directed to. The waypoints within the GPS denied zone, and the first waypoint outside of the GPS denied zone will be navigated to
         *              Contains:
         *                              - WaypointList:         A list of waypoints associated with this mission task. Each waypoint contains a waypoint number. This is a unique identifyer for each waypoint in the mission command.
         *                                                      The waypoint's next number is used to navigate the waypoints in order. (Waypoint[])[None]
         *                              - FirstWaypoint:        The waypoint number of the first waypoint in the waypoint list.
         * 
         * Sent Messages:
         *  - uxas::messages::uxnative::SafeHeadingAction
         *		This message is sent when the desired heading is calculated.
         *		This will be received by the LoiterLeashService, which will perform the safe heading action
         *		Contains: 
         *				-VehicleID:			 The ID of the vehicle which will be commanded (int64)[None]
         *				
         *				-OperatingRegion:	 the operating region ID indication the respected airspace constraint (int64)[None]
         *				
         *				-LeadAheadDistance:  Lead-ahead distance for the waypoint (real32)[meters]
         *				
         *				-LoiterRadius:		 Loiter radius on lead-ahead waypoint. If zero or negative, uses calculated minumum turn radius from vehicle configuration (real32)[meters]
         *				
         *				-DesiredHeading:	 Desired heading that vehicle should attempt to reach in degrees from true north (real32)[degrees]
         *				
         *				-DesiredHeadingRate: Desired heading rate for the vehicle (real32)[degrees/sec]
         *				
         *				-UseHeadingRate:	 Flag indicating selecting between heading (false) or heading rate (true) commands
         * 
         */

        class VisionBasedNavService : public ServiceBase {
        public:

            /** \brief This string is used to identify this service in XML configuration
             * files, i.e. Service Type="VisionBasedNavService". It is also entered into
             * service registry and used to create new instances of this service. */
            static const std::string&
            s_typeName() {
                static std::string s_string("VisionBasedNavService");
                return (s_string);
            };

            static const std::vector<std::string>
            s_registryServiceTypeNames() {
                std::vector<std::string> registryServiceTypeNames = {s_typeName()};
                return (registryServiceTypeNames);
            };

            /** \brief If this string is not empty, it is used to create a data 
             * directory to be used by the service. The path to this directory is
             * accessed through the ServiceBase variable m_workDirectoryPath. */
            static const std::string&
            s_directoryName() {
                static std::string s_string("");
                return (s_string);
            };

            static ServiceBase*
            create() {
                return new VisionBasedNavService;
            };

            VisionBasedNavService();

            virtual
            ~VisionBasedNavService();

        private:

            static
            ServiceBase::CreationRegistrar<VisionBasedNavService> s_registrar;

            /** \brief Copy construction not permitted */
            VisionBasedNavService(VisionBasedNavService const&) = delete;

            /** \brief Copy assignment operation not permitted */
            void operator=(VisionBasedNavService const&) = delete;

            bool
            configure(const pugi::xml_node& serviceXmlNode) override;

            bool
            initialize() override;

            bool
            start() override;

            bool
            terminate() override;

            bool
            processReceivedLmcpMessage(std::unique_ptr<uxas::communications::data::LmcpMessage> receivedLmcpMessage) override;
            
            /* \brief Checks if a point is in the GPS denied zone or not
             */
            bool
            isInGpsDeniedZone(double latitude, double longitude);

            /* \brief Creates a polygon from an abstract geometry
             */
            VisiLibity::Polygon
            fromAbstractGeometry(afrl::cmasi::AbstractGeometry *geom);

            /* \brief Converts abstract geometries to polygons 
            */
            bool
            linearizeBoundary(afrl::cmasi::AbstractGeometry * boundary, VisiLibity::Polygon &poly);

            /* \brief Adds waypoints of interest to the m_waypointList
             */
            void addWaypointsFromMissionCommandToWaypointList(afrl::cmasi::MissionCommand missionCommand);
            
            /* \brief A method that gets the next waypoint in the passed waypoint list*/
            afrl::cmasi::Waypoint* getWaypointInWaypointVector(int waypointNumber, std::vector<afrl::cmasi::Waypoint*> waypointList);

            /* \brief A helper method to get the next waypoint using the waypoint number.
             Doesnt use the currentwaypoint becuase we start with the waypoint number from the mission command*/
            afrl::cmasi::Waypoint getNextWaypointWithWaypointNumber(int waypointNumber);
            
            /* \brief Sends a loiter action at the desired waypoint...*/
            void sendLoiterAction(afrl::cmasi::Waypoint waypoint);

            /* \brief Gets the distance between the vehicle and the current waypoint. (takes vehicles latitude, longitude)*/
            double distanceToNextWaypoint(double vehicleLatitude, double vehicleLongitude);

            /* \brief Gets the heading angle to the next waypoint (with respect to true north) */
            double angleToNextWaypoint(double vehicleLatitude, double vehicleLongitude);
        private:

            /* \brief Vehicle ID for which vision based nav is enabled*/
            int64_t m_vehicleId{ 0 };
            
            /* \brief The loiter radius in meters for the last waypoint [m]*/
            float m_loiterRadius{304.8};
            
            /* \brief Flag to determine if in GPS denied zone or not. 
             * Set to true using entity states.
             * Set to false using VisionBasedPosition*/
            bool m_isInGpsDeniedZone{false};
            
            /* \brief The minimum distance from the waypoint before switching to the next waypoint in the waypoint list [m]*/
            double m_minWaypointDistance{20.0};
            
            /* \brief The target waypoint for the loiter leash*/
            afrl::cmasi::Waypoint m_currentWaypoint;

            /* \brief The list of waypoint to follow*/
            std::vector<afrl::cmasi::Waypoint*> m_waypointList;

            /* \brief A GPS denied zone which. 
            NOTE: Currently implementing single GPS denied zone*/
            VisiLibity::Polygon m_GpsDeniedPolygon = VisiLibity::Polygon();

            
        };

    } //namespace service
} //namespace uxas

#endif /* VISON_BASED_NAV_SERVICE_H */

