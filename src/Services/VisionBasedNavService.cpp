// ===============================================================================
// Authors: AFRL/RQQA
// Organization: Air Force Research Laboratory, Aerospace Systems Directorate, Power and Control Division
// 
// Copyright (c) 2017 Government of the United State of America, as represented by
// the Secretary of the Air Force.  No copyright is claimed in the United States under
// Title 17, U.S. Code.  All Other Rights Reserved.
// ===============================================================================

/* 
 * File:   VisionBasedNavService.cpp
 * Author: Jon & Caine
 *
 * Created on March 17, 2017, 5:55 PM
 *
 * <Service Type="VisionBasedNavService" VehicleID="0" MinWaypointDistance="150" LoiterRadius="350"/>
 * 
 */
//this service
#include "VisionBasedNavService.h"
//subscribed messages
#include "afrl/cmasi/EntityState.h"
#include "uxas/projects/teas/VisionBasedNavSwitch.h" 
#include "uxas/projects/teas/GPSDeniedZone.h"
#include "afrl/cmasi/MissionCommand.h"
#include "uxas/projects/teas/VisionBasedPosition.h"
//sent messages
#include "uxas/messages/uxnative/SafeHeadingAction.h"
#include "afrl/cmasi/AutomationRequest.h"
//other
#include "afrl/cmasi/KeyValuePair.h"
#include "UnitConversions.h"
#include "visilibity.h"
#include <map>
#include <iostream>     
#include <cmath>


// namespace definitions
namespace uxas // uxas::
{
    namespace service // uxas::service::
    {

        // this entry registers the service in the service creation registry
        VisionBasedNavService::ServiceBase::CreationRegistrar<VisionBasedNavService>
        VisionBasedNavService::s_registrar(VisionBasedNavService::s_registryServiceTypeNames());

        // service constructor
        VisionBasedNavService::VisionBasedNavService()
        : ServiceBase(VisionBasedNavService::s_typeName(), VisionBasedNavService::s_directoryName()) {
        };

        // service destructor
        VisionBasedNavService::~VisionBasedNavService() {
        };

        bool VisionBasedNavService::configure(const pugi::xml_node& ndComponent) {

            // configure based on attributes:
            if (!ndComponent.attribute("VehicleID").empty()) {
                m_vehicleId = ndComponent.attribute("VehicleID").as_int64();
            }
            if(!ndComponent.attribute("MinWaypointDistance").empty())
            {
                m_minWaypointDistance = ndComponent.attribute("MinWaypointDistance").as_double();
            }
            if(!ndComponent.attribute("LoiterRadius").empty())
            {
                m_loiterRadius = ndComponent.attribute("LoiterRadius").as_double();
            }

            // subscribe to messages:
            // ENTITY STATES
            addSubscriptionAddress(afrl::cmasi::EntityState::Subscription);
            std::vector< std::string > childstates = afrl::cmasi::EntityStateDescendants();
            for (auto child : childstates)
                addSubscriptionAddress(child);

            // GPS DENIED ZONE
            addSubscriptionAddress(uxas::projects::teas::GPSDeniedZone::Subscription);

            // MISSION COMMAND
            addSubscriptionAddress(afrl::cmasi::MissionCommand::Subscription);
            
            // VISION BASED POSITION
            addSubscriptionAddress(uxas::projects::teas::VisionBasedPosition::Subscription);
            return (true);
        }

        bool VisionBasedNavService::initialize() {
            // perform any required initialization before the service is started
            //std::cout << "*** INITIALIZING:: Service[" << s_typeName() << "] Service Id[" << m_serviceId << "] with working directory [" << m_workDirectoryName << "] *** " << std::endl;

            return (true);
        }

        bool VisionBasedNavService::start() {
            // perform any actions required at the time the service starts
            //std::cout << "*** STARTING:: Service[" << s_typeName() << "] Service Id[" << m_serviceId << "] with working directory [" << m_workDirectoryName << "] *** " << std::endl;

            return (true);
        };

        bool VisionBasedNavService::terminate() {
            // perform any action required during service termination, before destructor is called.
            //std::cout << "*** TERMINATING:: Service[" << s_typeName() << "] Service Id[" << m_serviceId << "] with working directory [" << m_workDirectoryName << "] *** " << std::endl;

            return (true);
        }

        //called every time a message is received
        bool VisionBasedNavService::processReceivedLmcpMessage(std::unique_ptr<uxas::communications::data::LmcpMessage> receivedLmcpMessage) {
            std::shared_ptr<avtas::lmcp::Object> messageObject = receivedLmcpMessage->m_object;

            bool isReadyToSend = false; //set to true when ready to send a safe heading action
            auto entityState = std::dynamic_pointer_cast<afrl::cmasi::EntityState>(messageObject);
            auto visionBasedPosition = std::dynamic_pointer_cast<uxas::projects::teas::VisionBasedPosition>(messageObject);
            auto gpsDeniedZone = std::dynamic_pointer_cast<uxas::projects::teas::GPSDeniedZone>(messageObject);
            auto missionCommand = std::dynamic_pointer_cast<afrl::cmasi::MissionCommand>(messageObject);
            float desiredHeading;

            if (entityState && entityState->getID() == m_vehicleId) //when an entity state is received, check if within GPS Denied Zone, if it is then set stuff for vehicle safe heading action and send message
            {
                //check if vehicle is in GPS denied zone//

                //first get lat long
                auto entityLocation = entityState->getLocation();
                double latitude = entityLocation->getLatitude();
                double longitude = entityLocation->getLongitude();

                // check if entity state's vehicle ID matches m_vehicleId and vehicle is within the Gps denied zone
               if(isInGpsDeniedZone(latitude,longitude)) {
                    m_isInGpsDeniedZone = true;
               }
               else{
                   if(m_isInGpsDeniedZone)
                   {
                       //just exited gps denied zone//
                       //std::cout << "Vehicle [" << m_vehicleId << "] exited the GPS denied zone" << std::endl;
                        m_isInGpsDeniedZone = false;
                        
                        //send loiter action at the current waypoint when vehicle exits gps denied zone
                        sendLoiterAction(m_currentWaypoint);
                   }
               }

            }
            
            else if (gpsDeniedZone) //when a gps denied zone is received, convert to polygon and save as member
            {
                //make polygon from gpsDeniedZone//
                //std::cout << "Vision based nav service received a gps denied zone" << std::endl;
                afrl::cmasi::AbstractGeometry *gpsGeo = gpsDeniedZone->getBoundary();
                m_GpsDeniedPolygon = fromAbstractGeometry(gpsGeo); //get polygon from boundary of the GpsDeniedZone
            }
            
            else if (missionCommand) {
                //assign waypoints from mission command//
                //std::cout << "NEW MISSION COMMAND RECEIVED" << std::endl;
                addWaypointsFromMissionCommandToWaypointList(*missionCommand);
                
                
            } else if(visionBasedPosition && m_isInGpsDeniedZone && visionBasedPosition->getID() == m_vehicleId)
            {
                //navigate with the vision based position message if true location (entity state) showed the vehicle is within gps denied zone//
                
                double latitude = visionBasedPosition->getLatitude();
                double longitude = visionBasedPosition->getLongitude();

                //set current waypoint as next waypoint if close to current waypoint.
                if (!m_waypointList.empty())
                {
                    auto distance = distanceToNextWaypoint(latitude, longitude);

                    //std::cout << "The current distance is: " << distance << "\nThe min distance is: " << m_minWaypointDistance << std::endl;
                    if (distance <= m_minWaypointDistance) {
                        m_currentWaypoint = getNextWaypointWithWaypointNumber(m_currentWaypoint.getNextWaypoint());
                    }
                    if(isInGpsDeniedZone(latitude, longitude))
                    {
                        desiredHeading = angleToNextWaypoint(latitude, longitude);
                        //std::cout << "The desired heading angle is: " << desiredHeading << std::endl;
                        isReadyToSend = true;
                    }
                }
            }

            //send a safe heading action when ready
            if (isReadyToSend) {
                //std::cout << "Sending a safe heading action for vehicle [" << m_vehicleId << "] at heading [" << desiredHeading << "]" << std::endl;

                // send out a safe heading action that will be handled by the LoiterLeash service
                auto safeHeadingAction = std::shared_ptr<uxas::messages::uxnative::SafeHeadingAction>(new uxas::messages::uxnative::SafeHeadingAction());
                safeHeadingAction->setDesiredHeading(desiredHeading); 
                safeHeadingAction->setLoiterRadius(m_loiterRadius);
                safeHeadingAction->setVehicleID(m_vehicleId); //set the vehicle commanded
                sendSharedLmcpObjectBroadcastMessage(safeHeadingAction); //send the message
            }

            return false;
        }
        
        void VisionBasedNavService::sendLoiterAction(afrl::cmasi::Waypoint waypoint)
        {
            afrl::cmasi::LoiterAction* pLoiterAction = new afrl::cmasi::LoiterAction();
            afrl::cmasi::Location3D* loiterLoc = new afrl::cmasi::Location3D(waypoint);
            pLoiterAction->setLocation(loiterLoc);
            loiterLoc = 0;
            pLoiterAction->setRadius(m_loiterRadius);
            pLoiterAction->setAxis(0.0);
            pLoiterAction->setDirection(afrl::cmasi::LoiterDirection::VehicleDefault);
            pLoiterAction->setDuration(-1.0);
            pLoiterAction->setLength(0.0);
            pLoiterAction->setLoiterType(afrl::cmasi::LoiterType::VehicleDefault);
            
            auto vehicleActionCommand = std::make_shared<afrl::cmasi::VehicleActionCommand>();
            vehicleActionCommand->setVehicleID(m_vehicleId);
            vehicleActionCommand->getVehicleActionList().push_back(pLoiterAction);
            pLoiterAction = 0;
            sendSharedLmcpObjectBroadcastMessage(vehicleActionCommand);            
        }

        bool VisionBasedNavService::isInGpsDeniedZone(double latitude, double longitude) {
            common::utilities::CUnitConversions unitConversions;

            // if GPS denied zone has more than 1 vertex, check if vehicle exists in the zone
            if (m_GpsDeniedPolygon.n() > 0) {
                //first convert the entity state's lat/long to flat earth coordinate
                double north, east;
                unitConversions.ConvertLatLong_degToNorthEast_m(latitude, longitude, north, east);
                //create point from flat earth coordinates
                VisiLibity::Point vehiclePoint = VisiLibity::Point(east, north);
                //check if the point is in the gps denied zone

                if (vehiclePoint.in(m_GpsDeniedPolygon)) {
                    return true;
                }
            }
            return false;
        }

        VisiLibity::Polygon VisionBasedNavService::fromAbstractGeometry(afrl::cmasi::AbstractGeometry *geom) {
            auto poly = VisiLibity::Polygon();
            linearizeBoundary(geom, poly);
            poly.eliminate_redundant_vertices(1.0);

            if (poly.area() < 0) {
                poly.reverse();
            }

            return poly;

        }

        bool VisionBasedNavService::linearizeBoundary(afrl::cmasi::AbstractGeometry *boundary, VisiLibity::Polygon &poly) {
            uxas::common::utilities::CUnitConversions flatEarth;
            bool isValid = false;
            poly.clear();

            if (afrl::cmasi::isPolygon(boundary)) {
                afrl::cmasi::Polygon* boundaryPolygon = (afrl::cmasi::Polygon*) boundary;
                for (unsigned int k = 0; k < boundaryPolygon->getBoundaryPoints().size(); k++) {
                    VisiLibity::Point pt;
                    double north, east;
                    flatEarth.ConvertLatLong_degToNorthEast_m(boundaryPolygon->getBoundaryPoints()[k]->getLatitude(), boundaryPolygon->getBoundaryPoints()[k]->getLongitude(), north, east);
                    pt.set_x(east);
                    pt.set_y(north);
                    poly.push_back(pt);
                }
                isValid = true;
            } else if (afrl::cmasi::isRectangle(boundary)) {
                afrl::cmasi::Rectangle* rectangle = (afrl::cmasi::Rectangle*) boundary;
                VisiLibity::Point c;
                double north, east;
                flatEarth.ConvertLatLong_degToNorthEast_m(rectangle->getCenterPoint()->getLatitude(), rectangle->getCenterPoint()->getLongitude(), north, east);
                c.set_x(east);
                c.set_y(north);

                VisiLibity::Point pt;
                // note: north-aligned positive rotation is opposite direction of x-aligned positive rotation
                double a = -rectangle->getRotation() * n_Const::c_Convert::dDegreesToRadians();
                double w = rectangle->getWidth() / 2;
                double h = rectangle->getHeight() / 2;

                poly.push_back(VisiLibity::Point::rotate(VisiLibity::Point(east + w, north + h) - c, a) + c);
                poly.push_back(VisiLibity::Point::rotate(VisiLibity::Point(east - w, north + h) - c, a) + c);
                poly.push_back(VisiLibity::Point::rotate(VisiLibity::Point(east - w, north - h) - c, a) + c);
                poly.push_back(VisiLibity::Point::rotate(VisiLibity::Point(east + w, north - h) - c, a) + c);

                isValid = true;
            } else if (afrl::cmasi::isCircle(boundary)) {
                afrl::cmasi::Circle* circle = (afrl::cmasi::Circle*) boundary;
                VisiLibity::Point c;
                double north, east;
                flatEarth.ConvertLatLong_degToNorthEast_m(circle->getCenterPoint()->getLatitude(), circle->getCenterPoint()->getLongitude(), north, east);
                c.set_x(east);
                c.set_y(north);
                double r = circle->getRadius() / cos(10.0 * n_Const::c_Convert::dDegreesToRadians());

                for (uint32_t k = 0; k < 18; k++) {
                    VisiLibity::Point pt;
                    pt.set_x(c.x() + r * cos(n_Const::c_Convert::dTwoPi() * k / 18.0));
                    pt.set_y(c.y() + r * sin(n_Const::c_Convert::dTwoPi() * k / 18.0));
                    poly.push_back(pt);
                }

                isValid = true;
            }

            return isValid;
        }

        double VisionBasedNavService::distanceToNextWaypoint(double vehicleLatitude, double vehicleLongitude) {
            uxas::common::utilities::CUnitConversions flatEarth;

            double north, east;
            flatEarth.ConvertLatLong_degToNorthEast_m(vehicleLatitude, vehicleLongitude, north, east);
            VisiLibity::Point vehicleLocation = VisiLibity::Point(east, north);
            //std::cout << "The first points coordinates are:\n\tEast: " << east << "\n\tNorth: " << north << std::endl;

            flatEarth.ConvertLatLong_degToNorthEast_m(m_currentWaypoint.getLatitude(), m_currentWaypoint.getLongitude(), north, east);
            VisiLibity::Point waypointLocation = VisiLibity::Point(east, north);
            //std::cout << "The second points coordinates are:\n\tEast " << east << "\n\tNorth: " << north << std::endl;
            
            return sqrt(pow(vehicleLocation.x() - waypointLocation.x(), 2) + pow(vehicleLocation.y() - waypointLocation.y(), 2));
        }

        double VisionBasedNavService::angleToNextWaypoint(double vehicleLatitude, double vehicleLongitude) {
            uxas::common::utilities::CUnitConversions flatEarth;
            double north, east;

            //create vehicle coordinate
            flatEarth.ConvertLatLong_degToNorthEast_m(vehicleLatitude, vehicleLongitude, north, east);
            VisiLibity::Point vehicleLocation = VisiLibity::Point(east, north);

            //try using the vehicle's longitude and increase its north for a north reference point
            north = north + 5000;
            VisiLibity::Point northLocation = VisiLibity::Point(east, north);

            double waypointLat = m_currentWaypoint.getLatitude();
            double waypointLong = m_currentWaypoint.getLongitude();

            //std::cout << "Moving to waypoint at: \n\tLat: " << waypointLat << "\n\tLong: " << waypointLong << std::endl;

            //create next waypoint coordinate
            flatEarth.ConvertLatLong_degToNorthEast_m(m_currentWaypoint.getLatitude(), m_currentWaypoint.getLongitude(), north, east);
            VisiLibity::Point waypointLocation = VisiLibity::Point(east, north);
            
            //get the angle between the points
            VisiLibity::Angle angleOfSeparation = VisiLibity::Point::angle_separation(northLocation, vehicleLocation, waypointLocation);

            /* ccw is positive for the angle of separation. subtract this from 360 to get the
             * bearing angle with respect to true north where cw is positive (desired heading) */
            double desiredHeading = 360 - (n_Const::c_Convert::toDegrees(angleOfSeparation.get()));
            /*std::cout << "=================================================" << std::endl;
            std::cout << "-------------------------------------------------" << std::endl;
            std::cout << "The angle is: " << desiredHeading << std::endl;
            std::cout << "-------------------------------------------------" << std::endl;
            std::cout << "=================================================" << std::endl;*/

            return desiredHeading;

        }

        afrl::cmasi::Waypoint VisionBasedNavService::getNextWaypointWithWaypointNumber(int waypointNumber) {

            std::vector<afrl::cmasi::Waypoint*> tmpVector(m_waypointList);
            afrl::cmasi::Waypoint* nextWaypoint = getWaypointInWaypointVector(waypointNumber, tmpVector);
            
            if (!nextWaypoint) //if nullptr then use the current waypoint
            {
                nextWaypoint = &m_currentWaypoint;
            }
            /*std::cout << "====================================================================="<< std::endl;
            std::cout << "====================================================================="<< std::endl;
            std::cout << "The next waypoint's location is:\n\tLatitude: " << nextWaypoint->getLatitude() << "\n\tLongitude: " << nextWaypoint->getLongitude() << std::endl;
            std::cout << "====================================================================="<< std::endl;
            std::cout << "====================================================================="<< std::endl;*/

            return *nextWaypoint;
        }
        
        afrl::cmasi::Waypoint* VisionBasedNavService::getWaypointInWaypointVector(int waypointNumber, std::vector<afrl::cmasi::Waypoint*> waypointList )
        {
            afrl::cmasi::Waypoint* nextWaypoint = nullptr;
            
            for(afrl::cmasi::Waypoint* waypoint : waypointList)
            {
                if (waypoint->getNumber() == waypointNumber)
                {
                    nextWaypoint = waypoint;
                    break;
                }
            }
            return nextWaypoint;
        }


        void VisionBasedNavService::addWaypointsFromMissionCommandToWaypointList(afrl::cmasi::MissionCommand missionCommand)
        {
            std::vector<afrl::cmasi::Waypoint*> waypointList(missionCommand.getWaypointList());
            afrl::cmasi::Waypoint* currentWaypoint = getWaypointInWaypointVector(missionCommand.getFirstWaypoint(), waypointList);
            //loop through waypoints
            bool previousWaypointInGpsDeniedZone = false;
            while(currentWaypoint)
            {
                //check if point in GPS denied zone
                if (isInGpsDeniedZone(currentWaypoint->getLatitude(), currentWaypoint->getLongitude()))
                {
                    if(!previousWaypointInGpsDeniedZone)
                    {
                        m_currentWaypoint = *currentWaypoint;
                    }
                    //if in gps denied zone then add to list of waypoints
                    m_waypointList.push_back(currentWaypoint->clone());
                    previousWaypointInGpsDeniedZone = true;
                }
                else
                {
                    //otherwise, if previous point was in GPS denied zone, then add to waypoint list
                    if(previousWaypointInGpsDeniedZone)
                    {
                        m_waypointList.push_back(currentWaypoint->clone());
                        //print the last waypoint's data
                        //std::cout << "The last waypoints data: \n\tLatitude: " << currentWaypoint->getLatitude() << "\n\tLongitude: " << currentWaypoint->getLongitude() << std::endl;
                    }
                }
                
                if(currentWaypoint->getNextWaypoint() == currentWaypoint->getNumber())
                {
                    currentWaypoint = nullptr;
                } else {
                currentWaypoint = getWaypointInWaypointVector(currentWaypoint->getNextWaypoint(), waypointList);
                }
            }
            //print the length of a vector:
            /*std::cout << "------------------------------------------" << std::endl;
            std::cout << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << std::endl;
            std::cout << "Waypoints added from mission command: \n\t" << m_waypointList.size() << std::endl;
            std::cout << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << std::endl;            
            std::cout << "------------------------------------------" << std::endl;*/
        }


    }; //namespace service
}; //namespace uxas
