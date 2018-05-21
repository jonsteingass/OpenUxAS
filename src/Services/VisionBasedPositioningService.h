// ===============================================================================
// Authors: AFRL/RQQA
// Organization: Air Force Research Laboratory, Aerospace Systems Directorate, Power and Control Division
// 
// Copyright (c) 2017 Government of the United State of America, as represented by
// the Secretary of the Air Force.  No copyright is claimed in the United States under
// Title 17, U.S. Code.  All Other Rights Reserved.
// ===============================================================================

/* 
 * File:   VisionBasedPositioningService.h
 * Author: Jon
 *
 * Created on March 17, 2017, 5:55 PM
 */

#ifndef UXAS_VISIONBASEDPOSITIONINGSERVICE_H
#define UXAS_VISIONBASEDPOSITIONINGSERVICE_H



#include "ServiceBase.h"
#include "CallbackTimer.h"
#include "TypeDefs/UxAS_TypeDefs_Timer.h"

namespace uxas {
    namespace service {

        /*! \class VisionBasedPositioningService
            \brief This service sends vision based position messages, which are processed by the vision based nav service
         * 
         * Configuration String: <Service Type="VisionBasedPositionService" VehicleId="0" IsTestMode="false" />
         * 
         * Options:
         *  - VehicleId - The Id of the vehicle to be sent in the VisionBasedPosition message. Used to only use entity state of vehicle navigating
         * 
         * Subscribed Messages:
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
         * Sent Messages:
         *  - uxas::projects::teas::VisionBasedNavPosition
         *          This message will be sent to communicate to other services the position of the vehicle as calculated by the vision based algorithm.
         * 
         * 
         */

        class VisionBasedPositioningService : public ServiceBase {
        public:

            /** \brief This string is used to identify this service in XML configuration
             * files, i.e. Service Type="VisionBasedPositionService". It is also entered into
             * service registry and used to create new instances of this service. */
            static const std::string&
            s_typeName() {
                static std::string s_string("VisionBasedPositioningService");
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
                return new VisionBasedPositioningService;
            };

            VisionBasedPositioningService();

            virtual
            ~VisionBasedPositioningService();

        private:

            static
            ServiceBase::CreationRegistrar<VisionBasedPositioningService> s_registrar;

            /** brief Copy construction not permitted */
            VisionBasedPositioningService(VisionBasedPositioningService const&) = delete;

            /** brief Copy assignment operation not permitted */
            void operator=(VisionBasedPositioningService const&) = delete;

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


        private:
          int m_vehicleId{0};  
          bool m_isTestMode{false};
        };

    }; //namespace service
}; //namespace uxas

#endif /* UXAS_VISIONBASEDPOSITIONINGSERVICE_H */

