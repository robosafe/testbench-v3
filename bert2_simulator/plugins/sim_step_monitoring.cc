/* 
sim_step_monitoring.cc
This code implements assertion monitoring at each step of a gazebo simulation.
In particular, we want to execute the following on every time step of the
simulation:
- save parameter values as 'signals'.
- execute checks on assertions.

Created by David Western, June 2015.
Following http://gazebosim.org/tutorials?tut=system_plugin&cat=write_plugin
*/

#include <gazebo/physics/physics.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/transport/transport.hh>
#include "std_msgs/String.h"
#include "gazebo/msgs/msgs.hh"
#include <common/common.hh>
#include <iostream>



namespace gazebo
{

  typedef const boost::shared_ptr<const msgs::WorldStatistics> WorldStatsPtr;

  class MyPlugin : public SystemPlugin
  {
    transport::PublisherPtr pub;
    transport::SubscriberPtr sub;
    common::Time t;
    msgs::Time tMsg;
    gazebo::msgs::WorldStatistics wMsg;
    physics::WorldPtr world;

    /////////////////////////////////////////////
    /// \brief Destructor
    public: virtual ~MyPlugin()
    { 
      this->connections.clear();
    }

    /////////////////////////////////////////////
    /// \brief Called after the plugin has been constructed.
    public: void Load(int argc, char ** argv)
    {
      printf("********** Hello from Load **************\n");

      this->connections.push_back(
          event::Events::ConnectWorldCreated(
            boost::bind(&MyPlugin::myWorldCreated, this)));
      this->connections.push_back(
          event::Events::ConnectWorldUpdateBegin(
            boost::bind(&MyPlugin::myWorldUpdateBegin, this)));
      this->connections.push_back(
          event::Events::ConnectWorldUpdateEnd(
            boost::bind(&MyPlugin::myWorldUpdateEnd, this)));
      /* See http://osrf-distributions.s3.amazonaws.com/gazebo/api/dev/classgazebo_1_1event_1_1Events.html#details
         for other events to use as triggers.
      */
    }

    /////////////////////////////////////////////
    // \brief Called once after Load
    private: void Init()
    {
      printf("********** Hello from Init **************\n");

      // Initialise protobuf message:
      tMsg.set_sec(0);
      tMsg.set_nsec(0);
      gazebo::msgs::Time* tp = wMsg.mutable_sim_time();
      tp->CopyFrom(tMsg);
      tp = wMsg.mutable_pause_time();
      tp->CopyFrom(tMsg);
      tp = wMsg.mutable_real_time();
      tp->CopyFrom(tMsg);
      wMsg.set_paused(false);
      wMsg.set_iterations(0);

    }
    
    //\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\
    /////////////////////////////////////////////
    /// \brief Called every WorldCreated event. See the Load function.
    private: void myWorldCreated()
    {

      printf("\n\n\n********** WorldCreated **************\n\n\n\n");

      // Get pointer to world, so we can pause it as needed.
      world = physics::get_world("default");

      // Get handle for the gazebo node:
      transport::NodePtr gzN(new transport::Node());

      // Set up a publisher for that node:
      pub = gzN->Advertise<gazebo::msgs::WorldStatistics>("~/AMM_trigger");
      /*std::cout << "\n\n\n\n\n\nWaiting for connection.\n\n\n\n";
      pub->WaitForConnection();
      std::cout << "\n\n\n\n\n\nConnection made\n\n\n\n";*/

      // Set up a subscriber to handle responses to AMM_trigger:
      this->sub = gzN->Subscribe("~/AMM_response", &MyPlugin::AMM_CB, this);

    }

    /* PROBLEM: This callback never gets invoked at the moment.
                See issue raised on Gazebo answers:
                http://answers.gazebosim.org/question/8667/problem-in-system-plugin-subscriber-callback-is/
    */
    public: void AMM_CB(WorldStatsPtr &response) {
      if (response->iterations() == wMsg.iterations()) {
        // Release the simulation:
        world->SetPaused(0);
        updateEnded = 1;
        //std::cout << "\n\n\nUnpaused\n\n\n";
        std::cout << "\n\n\nACCESSED AMM_CB CALLBACK in sim_step_monitoring.cc\n\n\n";
      } else {
        std::cout << "Lost sync between AMM trigger and response.\n"
                  << "       Now you're stuck on pause :p\n";
      }
    }
    
    //\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\
    /////////////////////////////////////////////
    /// \brief Called every WorldUpdateBegin event. See the Load function.
    private: void myWorldUpdateBegin()
    {

      //printf("********** WorldUpdateBegin **************\n");
      /*
      if (!updateEnded) {
          //physics::World::SetPaused(1); 	
          printf("\n\n\n\n\n* Overlap?? *\n\n\n\n\n");
      }
      updateEnded = 0;
      */
    }
    
    /////////////////////////////////////////////
    /// \brief Called every WorldUpdateBegin event. See the Load function.
    private: void myWorldUpdateEnd()
    {

      //printf("********** WorldUpdateEnd **************\n");

      // Don't let the new update start until the assertion monitors (AMs)
      // have done their thing.
      //world->SetPaused(1); 
      // The world will be unpaused when the AMM responds the trigger below.
      // See AMM_CB.
      /* PROBLEM: AMM_CB never gets invoked at the moment.
                  See issue raised on Gazebo answers:
                  http://answers.gazebosim.org/question/8667/problem-in-system-plugin-subscriber-callback-is/
      */
      /* WORKAROUND: The world is now unpaused by the AMM itself. */

      // Trigger AMM (AM Manager):
      wMsg.set_iterations(wMsg.iterations()+1);
      //wMsg.set_iterations(world->GetIterations()); // Only works in later versions of Gazebo.
      pub->Publish(wMsg);

      /*
      if (pub->HasConnections()) {
        std::cout << "Connected.\n";
      }

      unsigned int cbId = this->sub->GetCallbackId();
      std::string str = this->sub->GetTopic();
      std::cout << "cb ID = " << cbId << "\n" 
                << "topic = " << str << "\n";

      std::cout << "Worlds running = " << gazebo::physics::worlds_running() << "\n";*/
    }


    private: bool updateEnded;

    /// All the event connections.
    private: std::vector<event::ConnectionPtr> connections;
   
  };

  // Register this plugin with the simulator
  GZ_REGISTER_SYSTEM_PLUGIN(MyPlugin)
}
