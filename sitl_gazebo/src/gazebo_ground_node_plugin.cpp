/****************************************************************************
 *
 *   Copyright (c) 2018 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/
/**
 * @brief NRF Plugin
 *
 * This plugin simulates ground node
 *
 * @author Azeem Muzammil
 */

#include <common.h>
#include <sdf/sdf.hh>

#include <string>
#include <random>

#include <gazebo/common/Plugin.hh>
#include <gazebo/util/system.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <ignition/math.hh>

#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>

namespace gazebo {
  class GazeboGroundNodePlugin : public ModelPlugin {

    public: void getSdfParams(sdf::ElementPtr sdf) {
      this->namespace_.clear();
      if (sdf->HasElement("robotNamespace")) {
        this->namespace_ = sdf->GetElement("robotNamespace")->Get<std::string>();
      } else {
        gzerr << "[gazebo_ground_node_plugin] Please specify a robotNamespace.\n";
      }

      if (sdf->HasElement("nodeID")) {
        this->node_id_ = sdf->GetElement("nodeID")->Get<unsigned int>();
      } else {
        gzerr << "[gazebo_ground_node_plugin] Please specify a node id.\n";
      }
    }
    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf) {
	    getSdfParams(_sdf);
      // Store the pointer to the model
      this->model = _parent;
      this->world_ = this->model->GetWorld();

      #if GAZEBO_MAJOR_VERSION >= 9
        this->last_time_ = this->world_->SimTime();
        this->last_pub_time_ = this->world_->SimTime();
      #else
        this->last_time_ = this->world_->GetSimTime();
        this->last_pub_time_ = this->world_->GetSimTime();
      #endif

      // Listen to the update event. This event is broadcast every
      // simulation iteration.
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          std::bind(&GazeboGroundNodePlugin::OnUpdate, this));

      this->standard_normal_distribution_ = std::normal_distribution<double>(0.0, 0.1);
    }

    // Called by the world update start event
    public: void OnUpdate() {
      // Apply a small linear velocity to the model.
      //this->model->SetLinearVel(ignition::math::Vector3d(.3, 0, 0));

      #if GAZEBO_MAJOR_VERSION >= 9
        common::Time current_time = this->world_->SimTime();
      #else
        common::Time current_time = this->world_->GetSimTime();
      #endif
      double dt = (current_time - this->last_pub_time_).Double();
      double publish_rate = 5.0;

      if (dt > publish_rate) {
        if (this->current_node == 3 && this->is_validated_) {
          this->current_node = 1;
        } else {
          if (this->is_validated_) {
            this->current_node += 1;
          }
        }
        if (this->current_node == this->node_id_) {

          std::string message = "message";
          std:: cout << "Interruption message is set to True" << std::endl;
          std:: cout << "Message is received: " << "This is the message received from node " << this->node_id_ << std::endl;
          std:: cout << "Interruption message is set to False" << std::endl;
          std:: cout << "Message is Validating..." << std::endl;
          this->is_validated_ = randomBool();
          if (this->is_validated_) {
            std:: cout << "Validation Successful..." << std::endl;
            std:: cout << "Drone is moving to next node" << std::endl;
          } else {
            std:: cout << "Validation Failed..." << std::endl;
          }

        }

        this->last_pub_time_ = current_time;
      }

    }

    private: bool randomBool() {
      static auto gen = std::bind(std::uniform_int_distribution<>(0,1),std::default_random_engine());
      return gen();
    }

    // Pointer to the model
    private: physics::ModelPtr model;

    private: physics::WorldPtr world_;

    // Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;

    private: std::string namespace_;

    private: unsigned int node_id_;

    private: common::Time last_pub_time_;

    private: common::Time last_time_;

    private: int current_node = 0;

    private: std::normal_distribution<double> standard_normal_distribution_;

    private: std::default_random_engine random_generator_;

    private: bool is_validated_ = true;

  };
  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(GazeboGroundNodePlugin)
}

