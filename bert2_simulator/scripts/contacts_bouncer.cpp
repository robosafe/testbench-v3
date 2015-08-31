/* 
Contacts bouncer
This program relays messages from the gazebo topic /gazebo/default/physics/contacts
to the ROS topic /gazebo_contacts.  The purpose of this is to allow that information
to be accessed from python; Gazebo does not officially support a python API, although
pygazebo has recently emerged as an alternative option.

Created by David Western, August 2015.
*/

#include "ros/ros.h"
#include <iostream>
#include "std_msgs/String.h"
#include "std_msgs/UInt64.h"
#include <gazebo/gazebo.hh>
#include <gazebo/transport/transport.hh>
#include "gazebo/msgs/msgs.hh"         // Gazebo gazebo messages
#include <common/common.hh>
#include <gazebo_msgs/ContactsState.h> // ROS gazebo messages

namespace gazebo {

 typedef const boost::shared_ptr<const msgs::Contacts> contactsPtr;

 class contacts_bouncer {

   public:
        transport::NodePtr gn;         // Gazebo node.
        ros::NodeHandle rn;            // ROS node.
        transport::SubscriberPtr gSub; // Gazebo subscriber.
        ros::Publisher rPub;           // ROS publisher.
        msgs::Contacts gcMsg;          // Contacts message (from Gazebo)


        // Constructor:
        contacts_bouncer(transport::NodePtr gNode, ros::NodeHandle rNode) {

            this->gn = gNode;
            this->rn = rNode;

            // Publisher on ROS topic:
            this->rPub = rn.advertise<gazebo_msgs::ContactsState>("gazebo_contacts",10);

            // Listen to Gazebo topic for AMM_triggers:
            this->gSub = gn->Subscribe("/gazebo/default/physics/contacts", &contacts_bouncer::bounce, this);
        }
        
        void bounce(contactsPtr &gcMsg) {
            // Copy the message for use elsewhere.
            //this->cMsg = *cMsg;

            if (gcMsg->contact_size()>0) {
                // There's been a contact. Bounce it.

                gazebo_msgs::ContactsState rcMsg;     // Contacts message (to ROS)

                // First assemble the ROS message:
                //   Time
                rcMsg.header.stamp.sec = gcMsg->time().sec();
                rcMsg.header.stamp.nsec = gcMsg->time().nsec();
                //   Contacts
                gazebo_msgs::ContactState state;
                for (int k=0; k<gcMsg->contact_size(); k++) {
                    // For each contact, copy over the names of the colliding links.
                    state.collision1_name = gcMsg->contact(k).collision1();
                    state.collision2_name = gcMsg->contact(k).collision2();
                    /* N.B. Fields we're not copying over, but which we may want to include
                       in a later update: position, normal, depth, wrench, time (redundant
                       with gcMsg->time() ?), world.
                       See /usr/share/gazebo-1.9.6/gazebo/msgs/contacts.proto
                       and /opt/ros/hydro/share/gazebo_msgs/msg/ContactState.msg
                    */
                    rcMsg.states.push_back(state);
                }

                // Publish to ROS:
                rPub.publish(rcMsg);
            }
        }
 };
}

int main(int argc, char **argv)
{
    // Load gazebo
    gazebo::load(argc, argv);
    gazebo::run();

    // Create our gazebo node for communication
    gazebo::transport::NodePtr gNode(new gazebo::transport::Node());
    gNode->Init();

    // Initialise ROS node:
    ros::init(argc, argv, "contacts_bouncer", ros::init_options::AnonymousName);
    ros::NodeHandle rNode;
    
    gazebo::contacts_bouncer tb(gNode,rNode);

    std::cout << "Running contacts_bouncer...\n";
    ros::spin();
    std::cout << "\nFinished with contacts_bouncer.\n";
    
    // Make sure to shut everything down.
    gazebo::transport::fini();

    return 0;
}   
