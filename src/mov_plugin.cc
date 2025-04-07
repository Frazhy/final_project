#include <functional>
#include <gazebo/common/common.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <ignition/math/Vector3.hh>

namespace gazebo {
class MovPlugin : public ModelPlugin {
public:
  void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf) {
    // Store the pointer to the model
    this->model = _parent;

    this->iterations = 10 * 1000;
    if (_sdf->HasElement("iterations")) {
        this->iterations = _sdf->Get<int>("iterations");
    }

    this->velocity_x = 0.0;
    if (_sdf->HasElement("velocity_x")) {
        this->velocity_x = _sdf->Get<float>("velocity_x");
    }

    this->velocity_y = 0.0;
    if (_sdf->HasElement("velocity_y")) {
        this->velocity_y = _sdf->Get<float>("velocity_y");
    }

    // Listen to the update event. This event is broadcast every
    // simulation iteration.
    this->updateConnection = event::Events::ConnectWorldUpdateBegin(
        std::bind(&MovPlugin::OnUpdate, this));
  }

  // Called by the world update start event
public:
  void OnUpdate() {
    // Apply a small linear velocity to the model.

    if (this->counter < this->iterations) {
        this->model->SetLinearVel(ignition::math::Vector3d(this->velocity_x, this->velocity_y, 0.0));
        this->model->SetAngularVel(ignition::math::Vector3d(0.0, 0.0, 0.0));
    }
    else if (this->counter < 2*this->iterations){
        this->model->SetLinearVel(ignition::math::Vector3d(-this->velocity_x, -this->velocity_y, 0.0));
        this->model->SetAngularVel(ignition::math::Vector3d(0.0, 0.0, 0.0));
    }else{
        this->counter = 0;
    }
    this->counter++;
  }

  // Pointer to the model
private:
  physics::ModelPtr model;

private:
  int counter;
  int iterations;
  float velocity_x;
  float velocity_y;

  // Pointer to the update event connection
private:
  event::ConnectionPtr updateConnection;
};

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(MovPlugin)
} // namespace gazebo