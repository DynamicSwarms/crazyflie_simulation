#include "crazyflie_simulation/simulation/simulation.hpp"

#include "crazyflie_simulation/simulation/controller.hpp"
using namespace quadcopter;
Simulation::Simulation(const std::string& name, Eigen::Affine3d initial_pose)
    : m_name(name)
    , m_cmd()
    , m_quadrotor(std::make_shared<quadcopter::Quadrotor>())
    , m_controller(std::make_unique<CrazyflieController>())
{
    // Crazyflie parameters
    const Scalar mass = 0.032;
    const Scalar arm_length = 0.0325; 
    QuadrotorDynamics dynamics(mass, arm_length);

    m_quadrotor = std::make_shared<Quadrotor>(dynamics);
    QuadState initial_state;
    initial_state.setZero();
    initial_state.x[QS::POSX] = initial_pose.translation().x();
    initial_state.x[QS::POSY] = initial_pose.translation().y();
    initial_state.x[QS::POSZ] = initial_pose.translation().z();
    Eigen::Quaterniond q(initial_pose.linear());
    initial_state.x[QS::ATTW] = q.w();
    initial_state.x[QS::ATTX] = q.x();
    initial_state.x[QS::ATTY] = q.y();
    initial_state.x[QS::ATTZ] = q.z();

    m_quadrotor->setState(initial_state);
    m_quadrotor->setWorldBox((Matrix<3, 2>() << -10, 10, -10, 10, 0.0, 10).finished());

    m_cmd.t = 0.0;
    m_cmd.thrusts = quadcopter::Vector<4>::Zero();
}

Simulation::~Simulation()
{
}

void
Simulation::update(double d_t)
{
    m_controller->update(d_t, get_current_pose());
    std::vector<double> motor_commands;
    if (m_controller->get_motor_commands(motor_commands)) {
        m_cmd.thrusts = quadcopter::Vector<4>(motor_commands[0], motor_commands[1], motor_commands[2], motor_commands[3]);
    }
    std::vector<double> velocity_commands;
    if (m_controller->get_velocity_commands(velocity_commands)) {
        QuadState state;
        if (m_quadrotor->getState(&state))
        {
            state.x[QS::VELX] = velocity_commands[0];
            state.x[QS::VELY] = velocity_commands[1];
            state.x[QS::VELZ] = velocity_commands[2];
            state.x[QS::OMEX] = velocity_commands[3];
            state.x[QS::OMEY] = velocity_commands[4];
            state.x[QS::OMEZ] = velocity_commands[5];
            m_quadrotor->setState(state);
        }
    }
    
    m_cmd.t += d_t;  // time in seconds
    if (!m_quadrotor->run(m_cmd, d_t)) {
        std::cerr << "Quadrotor simulation step failed" << std::endl;
    } 
}

Eigen::Affine3d 
Simulation::get_current_pose()
{
    QuadState state;
    if (m_quadrotor->getState(&state)) {
        Eigen::Affine3d pose = Eigen::Affine3d::Identity();
        pose.translation() = Eigen::Vector3d(state.x[QS::POSX], state.x[QS::POSY], state.x[QS::POSZ]);
        pose.linear() = Eigen::Quaterniond(state.x[QS::ATTW], state.x[QS::ATTX], state.x[QS::ATTY], state.x[QS::ATTZ]).toRotationMatrix();
        return pose;
    } else {
        std::cerr << "Failed to get quadrotor state" << std::endl;
        return Eigen::Affine3d::Identity();
    }
}

void 
Simulation::set_target_pose(const Eigen::Affine3d &target_pose)
{
    m_controller->set_target_pose(target_pose);
}

double
Simulation::get_battery_voltage()
{
    return 4.2;    // Dummy value for battery voltage
}

double
Simulation::get_charge_current()
{
    return 0.0;    // Dummy value for charge current
}

double
Simulation::get_charge_state()
{
    return double(ChargeState::CHARGED); // Dummy value for charge state
} 

bool
Simulation::can_fly()
{
    return true; // Dummy value for can fly
}

bool
Simulation::is_flying()
{
    double z_global = get_current_pose().translation().z();
    return (z_global > 0.05); // Consider flying if above 5 cm
}

bool
Simulation::is_tumbled()
{
    return false; // Dummy value for is tumbled
}