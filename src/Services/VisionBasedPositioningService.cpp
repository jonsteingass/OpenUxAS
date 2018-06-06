// ===============================================================================
// Authors: AFRL/RQQA
// Organization: Air Force Research Laboratory, Aerospace Systems Directorate, Power and Control Division
// 
// Copyright (c) 2017 Government of the United State of America, as represented by
// the Secretary of the Air Force.  No copyright is claimed in the United States under
// Title 17, U.S. Code.  All Other Rights Reserved.
// ===============================================================================

/* 
 * File:   VisionBasedPositionService.cpp
 * Authors: Jon
 *
 * Created on March 17, 2017, 5:55 PM
 *
 * <Service Type="VisionBasedPositionService" VehicleId="0" IsTestMode="false"/>
 * 
 */

// include header for this service
#include "VisionBasedPositioningService.h"

#include "afrl/cmasi/EntityState.h"
#include "uxas/projects/teas/VisionBasedPosition.h"

#include <iostream>     // std::cout, cerr, etc

// convenience definitions for the option strings
#define VehicleId "VehicleID"
#define TestMode "IsTestMode"

// namespace definitions
namespace uxas  // uxas::
{
namespace service   // uxas::service::
{

// this entry registers the service in the service creation registry
VisionBasedPositioningService::ServiceBase::CreationRegistrar<VisionBasedPositioningService>
VisionBasedPositioningService::s_registrar(VisionBasedPositioningService::s_registryServiceTypeNames());

// service constructor
VisionBasedPositioningService::VisionBasedPositioningService()
: ServiceBase(VisionBasedPositioningService::s_typeName(), VisionBasedPositioningService::s_directoryName()) { };

// service destructor
VisionBasedPositioningService::~VisionBasedPositioningService() { };


//set up service based on xml attributes and subscribe to messages here
bool VisionBasedPositioningService::configure(const pugi::xml_node& ndComponent)
{
    bool isSuccess(true);

    if (!ndComponent.attribute(VehicleId).empty())
    {
        m_vehicleId = ndComponent.attribute(VehicleId).as_int();
    }
    if(!ndComponent.attribute(TestMode).empty())
    {
        m_isTestMode = ndComponent.attribute(TestMode).as_bool();
    }

    // subscribe to messages::
    // ENTITY STATES
    addSubscriptionAddress(afrl::cmasi::EntityState::Subscription);
    std::vector< std::string > childstates = afrl::cmasi::EntityStateDescendants();
    for (auto child : childstates)
        addSubscriptionAddress(child);
    return (isSuccess);
}

//send message when intiialized
bool VisionBasedPositioningService::initialize()
{
    // perform any required initialization before the service is started
    //std::cout << "*** INITIALIZING:: Service[" << s_typeName() << "] Service Id[" << m_serviceId << "] with working directory [" << m_workDirectoryName << "] *** " << std::endl;
    
    return (true);
}

//send message when the service is starting
bool VisionBasedPositioningService::start()
{
    // perform any actions required at the time the service starts
    //std::cout << "*** STARTING:: Service[" << s_typeName() << "] Service Id[" << m_serviceId << "] with working directory [" << m_workDirectoryName << "] *** " << std::endl;
    
    return (true);
};

//print statement when service is terminating. Do some clean up here
bool VisionBasedPositioningService::terminate()
{
    // perform any action required during service termination, before destructor is called.
    //std::cout << "*** TERMINATING:: Service[" << s_typeName() << "] Service Id[" << m_serviceId << "] with working directory [" << m_workDirectoryName << "] *** " << std::endl;
    
    return (true);
}

//The meat of a service. This is where the messages are digested and other stuff is handled
bool VisionBasedPositioningService::processReceivedLmcpMessage(std::unique_ptr<uxas::communications::data::LmcpMessage> receivedLmcpMessage)
{
    std::shared_ptr<avtas::lmcp::Object> messageObject = receivedLmcpMessage->m_object;
    
    auto entityState = std::dynamic_pointer_cast<afrl::cmasi::EntityState>(messageObject);
    bool isReadyToSend = false;
    auto positionMessage = std::shared_ptr<uxas::projects::teas::VisionBasedPosition>(new uxas::projects::teas::VisionBasedPosition());
    
    if(entityState && entityState->getID() == m_vehicleId)
    {
        auto entityLocation = entityState->getLocation();
        double latitude = entityLocation->getLatitude();
        double longitude = entityLocation->getLongitude();
        //get IMU data for vision-based navigation algorithm?
        //Check out what the entity state contains in the MDMs (OpenUxAS/doc/LMCP/index.html and look for entitystate)
        if(m_isTestMode)
        {
            //use the entity state location as the sent position message
            positionMessage->setLatitude(latitude);
            positionMessage->setLongitude(longitude);
            positionMessage->setVarianceX(0.0);
            positionMessage->setVarianceY(0.0);
            positionMessage->setVarianceZ(0.0);
            isReadyToSend = true;
        }
    }
    
    if(isReadyToSend)
    {
        //positionMessage->setLatitude(someLatVal);
        //positionMessage->setLongitude(someLonVal);
        
        //can set variance of direction if in covariance matrix (diagonal vals?). Otherwise its not the end of the world
        //positionMessage->setVarianceX(0.0);
        //positionMessage->setVarianceY(0.0);
        //positionMessage->setVarianceZ(0.0);
        positionMessage->setID(m_vehicleId);
        
        //Send the message
        sendSharedLmcpObjectBroadcastMessage(positionMessage);
    }
    
    
    return false;
}

}; //namespace service
}; //namespace uxas
