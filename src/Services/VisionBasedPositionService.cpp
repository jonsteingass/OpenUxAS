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
 * <Service Type="VisionBasedPositionService" VehicleId="0"/>
 * 
 */

// include header for this service
#include "VisionBasedPositionService.h"

#include "afrl/cmasi/EntityState.h"
#include "uxas/projects/teas/VisionBasedPosition.h"

#include <iostream>     // std::cout, cerr, etc

// convenience definitions for the option strings
#define VehicleId "VehicleId"

// namespace definitions
namespace uxas  // uxas::
{
namespace service   // uxas::service::
{

// this entry registers the service in the service creation registry
VisionBasedPositionService::ServiceBase::CreationRegistrar<VisionBasedPositionService>
VisionBasedPositionService::s_registrar(VisionBasedPositionService::s_registryServiceTypeNames());

// service constructor
VisionBasedPositionService::VisionBasedPositionService()
: ServiceBase(VisionBasedPositionService::s_typeName(), VisionBasedPositionService::s_directoryName()) { };

// service destructor
VisionBasedPositionService::~VisionBasedPositionService() { };


//set up service based on xml attributes and subscribe to messages here
bool VisionBasedPositionService::configure(const pugi::xml_node& ndComponent)
{
    bool isSuccess(true);

    if (!ndComponent.attribute(VehicleId).empty())
    {
        m_vehicleId = ndComponent.attribute(VehicleId).as_int();
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
bool VisionBasedPositionService::initialize()
{
    // perform any required initialization before the service is started
    std::cout << "*** INITIALIZING:: Service[" << s_typeName() << "] Service Id[" << m_serviceId << "] with working directory [" << m_workDirectoryName << "] *** " << std::endl;
    
    return (true);
}

//send message when the service is starting
bool VisionBasedPositionService::start()
{
    // perform any actions required at the time the service starts
    std::cout << "*** STARTING:: Service[" << s_typeName() << "] Service Id[" << m_serviceId << "] with working directory [" << m_workDirectoryName << "] *** " << std::endl;
    
    return (true);
};

//print statement when service is terminating. Do some clean up here
bool VisionBasedPositionService::terminate()
{
    // perform any action required during service termination, before destructor is called.
    std::cout << "*** TERMINATING:: Service[" << s_typeName() << "] Service Id[" << m_serviceId << "] with working directory [" << m_workDirectoryName << "] *** " << std::endl;
    
    return (true);
}

//The meat of a service. This is where the messages are digested and other stuff is handled
bool VisionBasedPositionService::processReceivedLmcpMessage(std::unique_ptr<uxas::communications::data::LmcpMessage> receivedLmcpMessage)
{
    std::shared_ptr<avtas::lmcp::Object> messageObject = receivedLmcpMessage->m_object;
    
    auto entityState = std::dynamic_pointer_cast<afrl::cmasi::EntityState>(messageObject);
    bool isReadyToSend = false;
    auto positionMessage = std::shared_ptr<uxas::projects::teas::VisionBasedPosition>(new uxas::projects::teas::VisionBasedPosition());
    
    if(entityState && entityState->getID() == m_vehicleId)
    {
        //get IMU data for vision-based navigation algorithm?
        //Check out what the entity state contains in the MDMs (OpenUxAS/doc/LMCP/index.html and look for entitystate)
    }
    
    if(isReadyToSend)
    {
        //positionMessage->setLatitude(someLatVal);
        //positionMessage->setLongitude(someLonVal);
        
        //can set variance of direction if in covariance matrix (diagonal vals?). Otherwise its not the end of the world
        //positionMessage->setVarianceX(0.0);
        //positionMessage->setVarianceY(0.0);
        //positionMessage->setVarianceZ(0.0);
        
        //Send the message
        //sendSharedLmcpObjectBroadcastMessage(positionMessage);
    }
    
    
    return false;
}

}; //namespace service
}; //namespace uxas
