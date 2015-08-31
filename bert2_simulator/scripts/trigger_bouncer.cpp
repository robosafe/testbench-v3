/* 
Trigger bouncer
This program is used in triggering assertion monitors on each Gazebo time step.
A gazebo system plugin (sim_step_monitoring.so) sends out trigger messages on a
gazebo topic to initiate assertion checks; trigger_bouncer relays these messages
to a ROS topic so that the assertion monitor manager (a ROS node) can handle them.

We should be able to skip this program and have the system plugin trigger the 
assertion monitor manager directly via its ROS topic, but I haven't made that work
yet (runtime errors caused by publishing to ROS topic; hard to debug).

Created by David Western, June 2015.
*/

#include "ros/ros.h"
#include <iostream>
#include "std_msgs/String.h"
#include "std_msgs/UInt64.h"
#include <gazebo/gazebo.hh>
#include <gazebo/transport/transport.hh>
#include "gazebo/msgs/msgs.hh"
#include <common/common.hh>
#include <gazebo/physics/physics.hh>

namespace gazebo {

 typedef const boost::shared_ptr<const msgs::WorldStatistics> WorldStatsPtr;

 class trigger_bouncer {

   public:
        transport::NodePtr gn;         // Gazebo node.
        ros::NodeHandle rn;            // ROS node.
        transport::SubscriberPtr gSub; // Gazebo subscriber.
        transport::PublisherPtr gPub;  // Gazebo publisher.
        ros::Publisher rPub;           // ROS publisher.
        msgs::WorldStatistics wMsg;    // World statistics message to send back to gazebo.
        physics::WorldPtr world;


        // Constructor:
        trigger_bouncer(transport::NodePtr gNode, ros::NodeHandle rNode) {

            this->gn = gNode;
            this->rn = rNode;

            // Publisher on ROS topic:
            this->rPub = rn.advertise<std_msgs::UInt64>("AM_trigger",10);

            // Listen to Gazebo topic for AMM_triggers:
            this->gSub = gn->Subscribe("/gazebo/AMM_trigger", &trigger_bouncer::AMM_Go, this);

            // Use this topic to let Gazebo carry on after AMM finishes:
            this->gPub = gn->Advertise<gazebo::msgs::WorldStatistics>("/gazebo/AMM_response");
            // Initialise protobuf message:
            msgs::Time tMsg;
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

            // Get pointer to world, so we can unpause it as needed.
            //gazebo::physics::WorldPtr world = gazebo::physics::get_world("default");
        }
        
        void AMM_Go(WorldStatsPtr &wMsg) {
            // Copy the message for use elsewhere.
            this->wMsg = *wMsg;

            // Dump the message contents to stdout.
            /*std::string trigTop = gSub->GetTopic();
            unsigned int cbID = gSub->GetCallbackId();
            std::cout << "TrigTopic: " << trigTop << "\n"
                      << "cb ID: " << cbID << "\n"
                      << "Iters: " << wMsg->iterations() << "\n";
            */

            // Publish to ROS
            std_msgs::UInt64 iter;
            iter.data = wMsg->iterations();
            rPub.publish(iter);

            // done();  // For testing only.  See comments in defn of done().
        }
        
        void done() {
            // Send a message to let Gazebo carry on.
            // If we implement the actual AMM in ROS, this method should be the callback
            // for a subscriber to a ROS topic advertised by the AMM.  For now,
            // it just gets called from AMM_Go.
            
            // Send back the same wMsg to gazebo, so that it knows which sim step we've
            // finished with.
            gPub->Publish(wMsg);

            /* if (gPub->HasConnections()) {
              std::cout << "Connected.\n";
            }
            std::cout << "Worlds running = " << gazebo::physics::worlds_running() << "\n"; 
            */

            //world->SetPaused(0);
        }
 };
}

int main(int argc, char **argv)
{
    // Load gazebo
    gazebo::load(argc, argv);
    gazebo::run();

    // Create our node for communication
    gazebo::transport::NodePtr gNode(new gazebo::transport::Node());
    gNode->Init();

    // Initialise assertion monitor manager (AMM) ROS node:
    ros::init(argc, argv, "trigger_bouncer", ros::init_options::AnonymousName);
    ros::NodeHandle rNode;
    
    gazebo::trigger_bouncer tb(gNode,rNode);

    //std::cout << "Worlds running = " << gazebo::physics::worlds_running() << "\n";

    //gazebo::physics::WorldPtr world = gazebo::physics::get_world("default");

    std::cout << "Running trigger_bouncer...\n";
    ros::spin();
    std::cout << "\nFinished with trigger_bouncer.\n";
    
    // Make sure to shut everything down.
    gazebo::transport::fini();

    return 0;
}   
