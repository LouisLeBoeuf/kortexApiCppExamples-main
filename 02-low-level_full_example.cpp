#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <math.h>
#include <KDetailedException.h>
#include <BaseClientRpc.h>
#include <BaseCyclicClientRpc.h>
#include <ActuatorConfigClientRpc.h>
#include <SessionClientRpc.h>
#include <SessionManager.h>
#include <ActuatorConfigClientRpc.h>
#include <RouterClient.h>
#include <TransportClientUdp.h>
#include <TransportClientTcp.h>
#include "utilities.h"
#include <google/protobuf/util/json_util.h>
#include <stdio.h>
#include <cmath>

#if defined(_MSC_VER)
#include <Windows.h>
#else
#include <unistd.h>
#endif
#include <time.h>
namespace k_api = Kinova::Api;

#define IP_ADDRESS "192.168.1.10"
#define PORT 10000
#define PORT_REAL_TIME 10001

#define PROPORTIONAL_GAIN (2.2f) //for the gripper
#define MINIMAL_POSITION_ERROR  ((float)1.5) //for the gripper

constexpr auto TIMEOUT_DURATION = std::chrono::seconds(20);

/*****************************
 * set TIME_DURATION equal to the duration you want the controller to run in seconds CHARS*
 *****************************/

float TIME_DURATION = 15.7f; // Duration of the example (seconds)
bool write_data = true; //set to true to write your data

// Waiting time during actions, not typically used
const auto ACTION_WAITING_TIME = std::chrono::seconds(1);
// Create closure to set finished to true after an END or an ABORT
std::function<void(k_api::Base::ActionNotification)>
check_for_end_or_abort(bool& finished)
{
    return [&finished](k_api::Base::ActionNotification notification)
    {
        std::cout << "EVENT : " << k_api::Base::ActionEvent_Name(notification.action_event()) << std::endl;

        // The action is finished when we receive a END or ABORT event
        switch(notification.action_event())
        {
            case k_api::Base::ActionEvent::ACTION_ABORT:
            case k_api::Base::ActionEvent::ACTION_END:
                finished = true;
                break;
            default:
                break;
        }
    };
}

/*****************************
 * GetTickUs gets computer time passed in microseconds
 * the rest of these are base functions used in controllers
 * ignore these*
 *****************************/
int64_t GetTickUs()
{
#if defined(_MSC_VER)
    LARGE_INTEGER start, frequency;

    QueryPerformanceFrequency(&frequency);
    QueryPerformanceCounter(&start);

    return (start.QuadPart * 1000000)/frequency.QuadPart;
#else
    struct timespec start;
    clock_gettime(CLOCK_MONOTONIC, &start);

    return (start.tv_sec * 1000000LLU) + (start.tv_nsec / 1000);
#endif
}

std::function<void(k_api::Base::ActionNotification)>
create_event_listener_by_promise(std::promise<k_api::Base::ActionEvent>& finish_promise)
{
    return [&finish_promise] (k_api::Base::ActionNotification notification)
    {
        const auto action_event = notification.action_event();
        switch(action_event)
        {
            case k_api::Base::ActionEvent::ACTION_END:
            case k_api::Base::ActionEvent::ACTION_ABORT:
                finish_promise.set_value(action_event);
                break;
            default:
                break;
        }
    };
}

std::function<void(k_api::Base::ActionNotification)>
create_event_listener_by_ref(k_api::Base::ActionEvent& returnAction)
{
    return [&returnAction](k_api::Base::ActionNotification notification)
    {
        const auto action_event = notification.action_event();
        switch(action_event)
        {
            case k_api::Base::ActionEvent::ACTION_END:
            case k_api::Base::ActionEvent::ACTION_ABORT:
                returnAction = action_event;
                break;
            default:
                break;
        }
    };
}

std::function<void(k_api::Base::ActionNotification)>
create_action_event_listener_by_promise(std::promise<k_api::Base::ActionEvent>& finish_promise)
{
    return [&finish_promise] (k_api::Base::ActionNotification notification)
    {
        const auto action_event = notification.action_event();
        switch(action_event)
        {
            case k_api::Base::ActionEvent::ACTION_END:
            case k_api::Base::ActionEvent::ACTION_ABORT:
                finish_promise.set_value(action_event);
                break;
            default:
                break;
        }
    };
}

bool example_move_to_home_position(k_api::Base::BaseClient* base)
{
    // Make sure the arm is in Single Level Servoing before executing an Action
    auto servoingMode = k_api::Base::ServoingModeInformation();
    servoingMode.set_servoing_mode(k_api::Base::ServoingMode::SINGLE_LEVEL_SERVOING);
    base->SetServoingMode(servoingMode);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    // Move arm to ready position
    std::cout << "Moving the arm to a safe position" << std::endl;
    auto action_type = k_api::Base::RequestedActionType();
    action_type.set_action_type(k_api::Base::REACH_JOINT_ANGLES);
    auto action_list = base->ReadAllActions(action_type);
    auto action_handle = k_api::Base::ActionHandle();
    action_handle.set_identifier(0);
    for (auto action : action_list.action_list())
    {
        if (action.name() == "Home")
        {
            action_handle = action.handle();
        }
    }

    if (action_handle.identifier() == 0)
    {
        std::cout << "Can't reach safe position, exiting" << std::endl;
        return false;
    }
    else
    {
        // Connect to notification action topic
        std::promise<k_api::Base::ActionEvent> promise;
        auto future = promise.get_future();
        const auto notification_handle = base->OnNotificationActionTopic(
                create_action_event_listener_by_promise(promise),
                k_api::Common::NotificationOptions{}
        );

        base->ExecuteActionFromReference(action_handle);

        // Wait for action to finish
        const auto status = future.wait_for(TIMEOUT_DURATION);
        base->Unsubscribe(notification_handle);

        if(status != std::future_status::ready)
        {
            std::cout << "Timeout on action notification wait" << std::endl;
            return false;
        }

        return true;

    }
}

bool move_to_desired_angles(k_api::Base::BaseClient* base, float actuator0, float actuator1, float actuator2, float actuator3, float actuator4, float actuator5)
{
    std::cout << "Starting angular action movement ..." << std::endl;

    auto action = k_api::Base::Action();
    action.set_name("Example angular action movement");
    action.set_application_data("");

    auto reach_joint_angles = action.mutable_reach_joint_angles();
    auto joint_angles = reach_joint_angles->mutable_joint_angles();

    auto actuator_count = base->GetActuatorCount();

    // Arm straight up
    /*
    for (size_t i = 0; i < actuator_count.count(); ++i)
    {
        auto joint_angle = joint_angles->add_joint_angles();
        joint_angle->set_joint_identifier(i);
        joint_angle->set_value(0);
    }*/

    auto joint_angle = joint_angles->add_joint_angles();
    joint_angle->set_joint_identifier(0);
    joint_angle->set_value(actuator0);

    joint_angle = joint_angles->add_joint_angles();
    joint_angle->set_joint_identifier(1);
    joint_angle->set_value(actuator1);

    joint_angle = joint_angles->add_joint_angles();
    joint_angle->set_joint_identifier(2);
    joint_angle->set_value(actuator2);

    joint_angle = joint_angles->add_joint_angles();
    joint_angle->set_joint_identifier(3);
    joint_angle->set_value(actuator3);

    joint_angle = joint_angles->add_joint_angles();
    joint_angle->set_joint_identifier(4);
    joint_angle->set_value(actuator4);

    joint_angle = joint_angles->add_joint_angles();
    joint_angle->set_joint_identifier(5);
    joint_angle->set_value(actuator5);

    // Connect to notification action topic
    // (Promise alternative)
    // See cartesian examples for Reference alternative
    std::promise<k_api::Base::ActionEvent> finish_promise;
    auto finish_future = finish_promise.get_future();
    auto promise_notification_handle = base->OnNotificationActionTopic(
            create_event_listener_by_promise(finish_promise),
            k_api::Common::NotificationOptions()
    );

    std::cout << "Executing action" << std::endl;
    base->ExecuteAction(action);

    std::cout << "Waiting for movement to finish ..." << std::endl;

    // Wait for future value from promise
    // (Promise alternative)
    // See cartesian examples for Reference alternative
    const auto status = finish_future.wait_for(TIMEOUT_DURATION);
    base->Unsubscribe(promise_notification_handle);

    if(status != std::future_status::ready)
    {
        std::cout << "Timeout on action notification wait" << std::endl;
        return false;
    }
    const auto promise_event = finish_future.get();

    std::cout << "Angular movement completed" << std::endl;
    std::cout << "Promise value : " << k_api::Base::ActionEvent_Name(promise_event) << std::endl;

    return true;
}

class GripperLowLevel
{
public:
    GripperLowLevel(const std::string& ip_address, int port_real_time , int port, const std::string& username = "admin", const std::string& password = "admin"):
            m_ip_address(ip_address), m_port(port), m_port_real_time(port_real_time), m_username(username), m_password(password), m_proportional_gain(0.0)
    {
        m_router                    = nullptr;
        m_router_real_time          = nullptr;
        m_transport                 = nullptr;
        m_transport_real_time       = nullptr;
        m_session_manager           = nullptr;
        m_session_manager_real_time = nullptr;
        m_base                      = nullptr;
        m_base_cyclic               = nullptr;
    }

    ~GripperLowLevel()
    {
        // Restore servoing mode.
        if (m_base)
        {
            m_base->SetServoingMode(m_previous_servoing_mode);
        }

        // Close API sessions
        if (m_session_manager)
        {
            m_session_manager->CloseSession();
        }
        if (m_session_manager_real_time)
        {
            m_session_manager_real_time->CloseSession();
        }

        // Deactivate the router and cleanly disconnect from the transport object
        m_router->SetActivationStatus(false);
        m_transport->disconnect();
        m_router_real_time->SetActivationStatus(false);
        m_transport_real_time->disconnect();

        // Destroy the API
        delete m_base;
        delete m_base_cyclic;
        delete m_session_manager;
        delete m_session_manager_real_time;
        delete m_router;
        delete m_router_real_time;
        delete m_transport;
        delete m_transport_real_time;
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // void Init(float proportional_gain = 2.0)
    //
    //      Initializes gripper object. Connection is made to the base using credentials given at object's
    //      construction.
    // INPUT:
    //      float fProportionnal:   Proportional gain to be used when using the position loop (default value = 2.0)
    //
    // OUTPUT:
    //      None.
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    void Init(float proportional_gain = 2.0)
    {
        m_proportional_gain = proportional_gain;

        ///////////////////////////////////////////////////////////////////////////////////////////
        // UDP and TCP sessions are used in this example.
        // TCP is used to perform the change of servoing mode
        // UDP is used for cyclic commands.
        //
        // 2 sessions have to be created: 1 for TCP and 1 for UDP
        ///////////////////////////////////////////////////////////////////////////////////////////

        // TCP transport and router
        m_transport = new k_api::TransportClientTcp();
        m_transport->connect(m_ip_address, m_port);
        m_router = new k_api::RouterClient(m_transport, [](k_api::KError err){ std::cout << "_________ callback error _________" << err.toString(); });

        // UDP transport and router
        m_transport_real_time = new k_api::TransportClientUdp();
        m_transport_real_time->connect(m_ip_address, m_port_real_time);
        m_router_real_time = new k_api::RouterClient(m_transport_real_time, [](k_api::KError err){ std::cout << "_________ callback error _________" << err.toString(); });

        // Set session data connection information
        auto createSessionInfo = k_api::Session::CreateSessionInfo();
        createSessionInfo.set_username(m_username);
        createSessionInfo.set_password(m_password);
        createSessionInfo.set_session_inactivity_timeout(60000);   // (milliseconds)
        createSessionInfo.set_connection_inactivity_timeout(2000); // (milliseconds)

        std::cout << "Creating sessions for communication" << std::endl;

        // Session manager service wrapper
        m_session_manager = new k_api::SessionManager(m_router);
        m_session_manager->CreateSession(createSessionInfo);

        m_session_manager_real_time = new k_api::SessionManager(m_router_real_time);
        m_session_manager_real_time->CreateSession(createSessionInfo);

        // Although TCP can be used, it is best to use UDP router for cyclic
        m_base_cyclic = new k_api::BaseCyclic::BaseCyclicClient(m_router_real_time);

        // Base client can only be operated on a TCP connection
        m_base = new k_api::Base::BaseClient(m_router);

        // Get previous servoing mode
        m_previous_servoing_mode = m_base->GetServoingMode();

        // Set arm in low level servoing mode
        k_api::Base::ServoingModeInformation servoing_info;
        servoing_info.set_servoing_mode(k_api::Base::LOW_LEVEL_SERVOING);
        m_base->SetServoingMode(servoing_info);

        // Get the feedback from the robot
        k_api::BaseCyclic::Feedback base_feedback;
        base_feedback = m_base_cyclic->RefreshFeedback();

        // This is how to get the gripper's actual position from the base feedback
        float gripper_initial_position = base_feedback.interconnect().gripper_feedback().motor()[0].position();

        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        // Initialize actuator commands to current position.
        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        for (auto actuator : base_feedback.actuators())
        {
            k_api::BaseCyclic::ActuatorCommand* actuator_command;
            actuator_command = m_base_command.mutable_actuators()->Add();
            actuator_command->set_position(actuator.position());
            actuator_command->set_velocity(0.0);
            actuator_command->set_torque_joint(0.0);
            actuator_command->set_command_id(0);
        }

        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        // Initialize interconnect command to current gripper position.
        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        m_base_command.mutable_interconnect()->mutable_command_id()->set_identifier(0);

        m_gripper_motor_command = m_base_command.mutable_interconnect()->mutable_gripper_command()->add_motor_cmd();
        m_gripper_motor_command->set_position(gripper_initial_position );
        m_gripper_motor_command->set_velocity(0.0);
        m_gripper_motor_command->set_force(100.0);
    }


    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // void GoTo(float target_position)
    //
    //      Sends the gripper to a target position using a feed back loop made with base cyclic command.
    //      The method blocks until position is reached.
    //      The feedback loop used is a simple proportional loop.
    // INPUT:
    //      float target_position:  Target gripper position in percent (100.0 = fully closed, 0.0 = fully open)
    //
    // OUTPUT:
    //      Returns true if operation completed successfully, false otherwise.
    //
    // NOTES:
    //      - This method blocks until the requested position is reached.
    //      - If target position exceeds 100.0, its value is changed to 100.0.
    //      - If target position is below 0.0, its value is set to 0.0.
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    bool GoTo(float target_position)
    {
        k_api::BaseCyclic::Feedback base_feedback;
        float position_error;
        if (target_position > 100.0)
        {
            target_position = 100.0;
        }
        if (target_position < 0.0)
        {
            target_position = 0.0;
        }

        while (true)
        {
            try
            {
                float velocity;
                float actual_position;

                // Refresh cyclic data (send command and get feedback)
                base_feedback = m_base_cyclic->Refresh(m_base_command);
                actual_position = base_feedback.interconnect().gripper_feedback().motor()[0].position();

                position_error = target_position - actual_position;

                if (fabs(position_error) < MINIMAL_POSITION_ERROR)
                {
                    m_gripper_motor_command->set_velocity(0.0);
                    m_base_cyclic->Refresh(m_base_command);
                    break;
                }

                velocity = m_proportional_gain * fabs(position_error);
                if (velocity > 100.0)
                {
                    velocity = 100.0;
                }

                m_gripper_motor_command->set_position(target_position);
                m_gripper_motor_command->set_velocity(velocity);
            }
            catch(const std::exception& ex)
            {
                std::cerr << "Error occurred: " << ex.what() << std::endl;
                return false;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        // If we get here, operation completed successfully.
        return true;
    }

private:
    k_api::TransportClientTcp*            m_transport;
    k_api::TransportClientUdp*            m_transport_real_time;
    k_api::RouterClient*                  m_router;
    k_api::RouterClient*                  m_router_real_time;
    k_api::SessionManager*                m_session_manager;
    k_api::SessionManager*                m_session_manager_real_time;
    k_api::Base::BaseClient*              m_base;
    k_api::BaseCyclic::BaseCyclicClient*  m_base_cyclic;

    k_api::BaseCyclic::Command            m_base_command;
    k_api::GripperCyclic::MotorCommand*   m_gripper_motor_command;
    std::string                           m_username;
    std::string                           m_password;
    std::string                           m_ip_address;
    int                                   m_port_real_time;
    int                                   m_port;
    k_api::Base::ServoingModeInformation  m_previous_servoing_mode;
    float                                 m_proportional_gain;
};

/*****************************
 * Rename the ExampleController to something new
 * OR copy-paste your controller into here, replacing ExampleController
 * when you are done put your controller back into ControllerStorage file
 * AND copy paste the example controller back here after this comment CHARS*
 *****************************/

class trajectory{
    public:
        //variables
        float actuator0path[5] = {};
        float actuator1path[5] = {};
        float actuator2path[5] = {};
        float actuator3path[5] = {};
        float actuator4path[5] = {};
        float actuator5path[5] = {};
        float allpaths[6] = {actuator0path, actuator1path, actuator2path, actuator3path, actuator4path, actuator5path};
        //functions
        float desiredPosition(float current_time, float actuatorid){
            float dp = 0;
            float zeroth - 0;
            float first = 0;
            float second = 0;
            float third = 0;
            float fourth = 0;
            float fifth = 0;
            coef = allpaths[actuatorid];
            for (unsigned int i = 0; i < 6; i++) {
                zeroth = coef[0];
                first = coef[1]*current_time;
                second = coef[2]*current_time*current_time;
                third = coef[3]*current_time*current_time*current_time;
                fourth = coef[4]*current_time*current_time*current_time*current_time;
                fifth = coef[5]*current_time*current_time*current_time*current_time*current_time;;
                dp = zeroth+first+second+third+fourth+fifth;
            }
            return dp;
        };
};


bool ExampleController(k_api::Base::BaseClient* base, k_api::BaseCyclic::BaseCyclicClient* base_cyclic, k_api::ActuatorConfig::ActuatorConfigClient* actuator_config, int actuators_active [6] = {}){
    bool return_status = true;

    // Get actuator count
    unsigned int actuator_count = base->GetActuatorCount().count();

    // Clearing faults
    try
    {
        base->ClearFaults();
    }
    catch(...)
    {
        std::cout << "Unable to clear robot faults" << std::endl;
        return false;
    }

    k_api::BaseCyclic::Feedback base_feedback;
    k_api::BaseCyclic::Command  base_command;

    std::vector<float> commands;

    auto servoing_mode = k_api::Base::ServoingModeInformation();

    float timer_count = 0; //changed this to a float instead of integer to use in sin function CHARS
    int64_t now = 0;
    int64_t last = 0;

    /*****************************
     * Declare your variables CHARS*
     *****************************/

    trajectory testpath;
    testpath.actuator0path = [90 0 0 0 0 0];

    float error_position [6] = {}; // The error in position [6x1]
    float error_velocity [6] = {}; // The error in velocity [6x1]
    float current_position [6] = {}; // current position of each actuator [6x1]
    float current_velocity [6] = {}; // current velocity of each actuator [6x1]
    float torque_sent [6] = {}; //the torque value to send each actuator (calculated by controller) [6x1]
    //float current_torque [6] = {}; // current torque of each actuator [6x1]


    //the following are vector to store values for writing to files later ignore these
    std::vector<float> actual_positions0;
    std::vector<float> actual_velocities0;
    std::vector<float> desired_positions0;
    std::vector<float> desired_velocities0;
    std::vector<float> controller_output0;
    std::vector<float> desired_positions1;
    std::vector<float> actual_positions1;
    std::vector<float> controller_output1;
    std::vector<float> desired_positions2;
    std::vector<float> actual_positions2;
    std::vector<float> controller_output2;
    std::vector<float> desired_positions3;
    std::vector<float> actual_positions3;
    std::vector<float> controller_output3;
    std::vector<float> desired_positions4;
    std::vector<float> actual_positions4;
    std::vector<float> controller_output4;
    std::vector<float> desired_positions5;
    std::vector<float> actual_positions5;
    std::vector<float> controller_output5;
    std::vector<float> desired_positions6;
    std::vector<float> actual_positions6;
    std::vector<float> controller_output6;

    std::cout << "Initializing the arm for torque control example" << std::endl;
    try
    {
        // Set the base in low-level servoing mode
        servoing_mode.set_servoing_mode(k_api::Base::ServoingMode::LOW_LEVEL_SERVOING);
        base->SetServoingMode(servoing_mode);
        base_feedback = base_cyclic->RefreshFeedback();

        // Initialize each actuator to their current position
        for (unsigned int i = 0; i < actuator_count; i++)
        {
            commands.push_back(base_feedback.actuators(i).position());
            // Save the current actuator position, to avoid a following error
            base_command.add_actuators()->set_position(base_feedback.actuators(i).position());
        }

        // Send a first frame
        base_feedback = base_cyclic->Refresh(base_command);
        // Set first actuator in torque mode now that the command is equal to measure
        auto control_mode_message = k_api::ActuatorConfig::ControlModeInformation();
        control_mode_message.set_control_mode(k_api::ActuatorConfig::ControlMode::TORQUE);

        //sets each actuator you specified to low level control mode
        for (unsigned int i = 0; i < actuator_count; i++){
            if (actuators_active[i]>0){actuator_config->SetControlMode(control_mode_message, i+1);}
        };

        std::cout << "Running torque controller for " << TIME_DURATION << " seconds "<< std::endl;

        // Real-time loop
        while (timer_count < (TIME_DURATION * 1000))
        {
            now = GetTickUs(); //now and last are both in microseconds, timer_count in milliseconds CHARS

            if (now - last > 1000)
            {
                // Position command to first actuator is set to measured one to avoid following error to trigger When doing this instead
                // of disabling the following error, if communication is lost and actuator continues to move under torque command, resulting
                // position error with command will trigger a following error and switch back the actuator in position command to hold its position
                for (unsigned int i = 0; i < actuator_count; i++){
                    if (actuators_active[i] > 0){
                        base_command.mutable_actuators(i)->set_position(base_feedback.actuators(i).position());}
                }

                // Incrementing identifier ensures actuators can reject out of time frames
                base_command.set_frame_id(base_command.frame_id() + 1);
                if (base_command.frame_id() > 65535)
                    base_command.set_frame_id(0);


                //Saves current positions and velocities of each actuator and calculates the error between them CHARS
                for (unsigned int i = 0; i < actuator_count; i++) {
                    current_position[i] = base_feedback.actuators(i).position();
                    current_velocity[i] = base_feedback.actuators(i).velocity();
                    error_position[i] = testpath.desiredPosition(timer_count/1000,actuator_count))-current_position[i];
                    torque_sent[i] = Kp[i]*error_position[i]+Kd[i]*error_velocity[i];
                    if (torque_sent[i]>30) {torque_sent[i]=30;};
                    if (torque_sent[i]<-30) {torque_sent[i]=-30;};
                }

                // appends values to these vectors used to store values to write to a file
                if (write_data==true){
                    actual_positions0.push_back(current_position[0]);
                    actual_velocities0.push_back(current_velocity[0]);
                    desired_positions0.push_back(desired_position[0]);
                    desired_velocities0.push_back(desired_velocity[0]);
                    controller_output0.push_back(torque_sent[0]);
                    actual_positions1.push_back(current_position[1]);
                    desired_positions1.push_back(desired_position[1]);
                    controller_output1.push_back(torque_sent[1]);
                    actual_positions2.push_back(current_position[2]);
                    desired_positions2.push_back(desired_position[2]);
                    controller_output2.push_back(torque_sent[2]);
                    actual_positions3.push_back(current_position[3]);
                    desired_positions3.push_back(desired_position[3]);
                    controller_output3.push_back(torque_sent[3]);
                    actual_positions4.push_back(current_position[4]);
                    desired_positions4.push_back(desired_position[4]);
                    controller_output4.push_back(torque_sent[4]);
                    actual_positions5.push_back(current_position[5]);
                    desired_positions5.push_back(desired_position[5]);
                    controller_output5.push_back(torque_sent[5]);}

                //sends the torque to each actuator that is in low level control mode
                for (unsigned int i = 0; i < actuator_count; i++){
                    if (actuators_active[i] > 0){
                        base_command.mutable_actuators(i)->set_torque_joint(torque_sent[i]);}
                }

                /*****************************
                 * Here are some useful print commands
                //std::cout << "Actuator 0 error in position = " << error_position[0] << std::endl;
                //std::cout << "Actuator 0 error in velocity = " << error_velocity[0] << std::endl;
                //std::cout << "Actuator 0 desired position =" << desired_position[0] << std::endl;
                //std::cout << "Timer Count =" << timer_count << std::endl;
                //std::cout << "Actuator 0 Torque Sent =" << torque_sent[0] << std::endl;*
                *****************************/

                for (unsigned int idx = 0; idx < actuator_count; idx++)
                {
                    base_command.mutable_actuators(idx)->set_command_id(base_command.frame_id());
                }

                try
                {
                    base_feedback = base_cyclic->Refresh(base_command, 0);
                }
                catch (k_api::KDetailedException& ex)
                {
                    std::cout << "Kortex exception: " << ex.what() << std::endl;

                    std::cout << "Error sub-code: " << k_api::SubErrorCodes_Name(k_api::SubErrorCodes((ex.getErrorInfo().getError().error_sub_code()))) << std::endl;
                }
                catch (std::runtime_error& ex2)
                {
                    std::cout << "runtime error: " << ex2.what() << std::endl;
                }
                catch(...)
                {
                    std::cout << "Unknown error." << std::endl;
                }

                timer_count++;
                last = GetTickUs();
            }
        }

        //Below writes your data vectors to files
        if (write_data==true){

            ofstream fw("C:\\Users\\Chars-local\\Desktop\\KinovaOutputData\\0ActualPositions.txt", std::ofstream::out);
            if (fw.is_open())
            {
                //store array contents to text file
                for (unsigned int i = 0; i < actual_positions0.size(); i++) {
                    fw << actual_positions0.at(i) << "\n";
                }
                fw.close();
            }
            else cout << "Problem with opening 0ActualPositions file";

            ofstream fww("C:\\Users\\Chars-local\\Desktop\\KinovaOutputData\\0ActualVelocities.txt", std::ofstream::out);
            if (fww.is_open())
            {
                //store array contents to text file
                for (unsigned int i = 0; i < actual_velocities0.size(); i++) {
                    fww << actual_velocities0.at(i) << "\n";
                }
                fww.close();
            }
            else cout << "Problem with opening 0ActualVelocities file";

            ofstream fwww("C:\\Users\\Chars-local\\Desktop\\KinovaOutputData\\0DesiredPositions.txt", std::ofstream::out);
            if (fwww.is_open())
            {
                //store array contents to text file
                for (unsigned int i = 0; i < desired_positions0.size(); i++) {
                    fwww << desired_positions0.at(i) << "\n";
                }
                fwww.close();
            }
            else cout << "Problem with opening DesiredPositions0 file";

            ofstream fwwww("C:\\Users\\Chars-local\\Desktop\\KinovaOutputData\\0DesiredVelocities.txt", std::ofstream::out);
            if (fwwww.is_open())
            {
                //store array contents to text file
                for (unsigned int i = 0; i < desired_velocities0.size(); i++) {
                    fwwww << desired_velocities0.at(i) << "\n";
                }
                fwwww.close();
            }
            else cout << "Problem with opening 0DesiredVelocities file";

            ofstream faa("C:\\Users\\Chars-local\\Desktop\\KinovaOutputData\\0ControllerOutput.txt", std::ofstream::out);
            if (faa.is_open())
            {
                //store array contents to text file
                for (unsigned int i = 0; i < controller_output0.size(); i++) {
                    faa << controller_output0.at(i) << "\n";
                }
                faa.close();
            }
            else cout << "Problem with opening 0ControllerOutput.txt file";


            ofstream fa("C:\\Users\\Chars-local\\Desktop\\KinovaOutputData\\1ActualPositions.txt", std::ofstream::out);
            if (fa.is_open())
            {
                //store array contents to text file
                for (unsigned int i = 0; i < actual_positions1.size(); i++) {
                    fa << actual_positions1.at(i) << "\n";
                }
                fa.close();
            }
            else cout << "Problem with opening 1ActualPositions file";

            ofstream fb("C:\\Users\\Chars-local\\Desktop\\KinovaOutputData\\1DesiredPositions.txt", std::ofstream::out);
            if (fb.is_open())
            {
                //store array contents to text file
                for (unsigned int i = 0; i < desired_positions1.size(); i++) {
                    fb << desired_positions1.at(i) << "\n";
                }
                fb.close();
            }
            else cout << "Problem with opening 1DesiredPositions file";

            ofstream fcc("C:\\Users\\Chars-local\\Desktop\\KinovaOutputData\\1ControllerOutput.txt", std::ofstream::out);
            if (fcc.is_open())
            {
                //store array contents to text file
                for (unsigned int i = 0; i < controller_output1.size(); i++) {
                    fcc << controller_output1.at(i) << "\n";
                }
                fcc.close();
            }
            else cout << "Problem with opening 1ControllerOutput.txt file";


            ofstream fc("C:\\Users\\Chars-local\\Desktop\\KinovaOutputData\\2ActualPositions.txt", std::ofstream::out);
            if (fc.is_open())
            {
                //store array contents to text file
                for (unsigned int i = 0; i < actual_positions2.size(); i++) {
                    fc << actual_positions2.at(i) << "\n";
                }
                fc.close();
            }
            else cout << "Problem with opening 2ActualPositions file";

            ofstream fd("C:\\Users\\Chars-local\\Desktop\\KinovaOutputData\\2DesiredPositions.txt", std::ofstream::out);
            if (fd.is_open())
            {
                //store array contents to text file
                for (unsigned int i = 0; i < desired_positions2.size(); i++) {
                    fd << desired_positions2.at(i) << "\n";
                }
                fd.close();
            }
            else cout << "Problem with opening 2DesiredPositions file";

            ofstream fee("C:\\Users\\Chars-local\\Desktop\\KinovaOutputData\\2ControllerOutput.txt", std::ofstream::out);
            if (fee.is_open())
            {
                //store array contents to text file
                for (unsigned int i = 0; i < controller_output2.size(); i++) {
                    fee << controller_output2.at(i) << "\n";
                }
                fee.close();
            }
            else cout << "Problem with opening 2ControllerOutput file";


            ofstream fe("C:\\Users\\Chars-local\\Desktop\\KinovaOutputData\\3ActualPositions.txt", std::ofstream::out);
            if (fe.is_open())
            {
                //store array contents to text file
                for (unsigned int i = 0; i < actual_positions3.size(); i++) {
                    fe << actual_positions3.at(i) << "\n";
                }
                fe.close();
            }
            else cout << "Problem with opening 3ActualPositions file";

            ofstream ff("C:\\Users\\Chars-local\\Desktop\\KinovaOutputData\\3DesiredPositions.txt", std::ofstream::out);
            if (ff.is_open())
            {
                //store array contents to text file
                for (unsigned int i = 0; i < desired_positions3.size(); i++) {
                    ff << desired_positions3.at(i) << "\n";
                }
                ff.close();
            }
            else cout << "Problem with opening 3DesiredPositions file";

            ofstream fgg("C:\\Users\\Chars-local\\Desktop\\KinovaOutputData\\3ControllerOutput.txt", std::ofstream::out);
            if (fgg.is_open())
            {
                //store array contents to text file
                for (unsigned int i = 0; i < controller_output3.size(); i++) {
                    fgg << controller_output3.at(i) << "\n";
                }
                fgg.close();
            }
            else cout << "Problem with opening 3ControllerOutput.txt file";


            ofstream fg("C:\\Users\\Chars-local\\Desktop\\KinovaOutputData\\4ActualPositions.txt", std::ofstream::out);
            if (fg.is_open())
            {
                //store array contents to text file
                for (unsigned int i = 0; i < actual_positions4.size(); i++) {
                    fg << actual_positions4.at(i) << "\n";
                }
                fg.close();
            }
            else cout << "Problem with opening 4ActualPositions file";

            ofstream fh("C:\\Users\\Chars-local\\Desktop\\KinovaOutputData\\4DesiredPositions.txt", std::ofstream::out);
            if (fh.is_open())
            {
                //store array contents to text file
                for (unsigned int i = 0; i < desired_positions4.size(); i++) {
                    fh << desired_positions4.at(i) << "\n";
                }
                fh.close();
            }
            else cout << "Problem with opening 4DesiredPositions file";

            ofstream fii("C:\\Users\\Chars-local\\Desktop\\KinovaOutputData\\4ControllerOutput.txt", std::ofstream::out);
            if (fii.is_open())
            {
                //store array contents to text file
                for (unsigned int i = 0; i < controller_output4.size(); i++) {
                    fii << controller_output4.at(i) << "\n";
                }
                fii.close();
            }
            else cout << "Problem with opening 4ControllerOutput.txt file";


            ofstream fi("C:\\Users\\Chars-local\\Desktop\\KinovaOutputData\\5ActualPositions.txt", std::ofstream::out);
            if (fi.is_open())
            {
                //store array contents to text file
                for (unsigned int i = 0; i < actual_positions5.size(); i++) {
                    fi << actual_positions5.at(i) << "\n";
                }
                fi.close();
            }
            else cout << "Problem with opening 5ActualPositions file";

            ofstream fj("C:\\Users\\Chars-local\\Desktop\\KinovaOutputData\\5DesiredPositions.txt", std::ofstream::out);
            if (fj.is_open())
            {
                //store array contents to text file
                for (unsigned int i = 0; i < desired_positions5.size(); i++) {
                    fj << desired_positions5.at(i) << "\n";
                }
                fj.close();
            }
            else cout << "Problem with opening 5DesiredPositions file";

            ofstream fk("C:\\Users\\Chars-local\\Desktop\\KinovaOutputData\\5ControllerOutput.txt", std::ofstream::out);
            if (fk.is_open())
            {
                //store array contents to text file
                for (unsigned int i = 0; i < controller_output5.size(); i++) {
                    fk << controller_output5.at(i) << "\n";
                }
                fk.close();
            }
            else cout << "Problem with opening 5ControllerOutput.txt file";}

        //sets each actuator back to high level control mode
        for (unsigned int i = 0; i < actuator_count; i++){
            if (actuators_active[i] > 0){
                control_mode_message.set_control_mode(k_api::ActuatorConfig::ControlMode::POSITION);
                actuator_config->SetControlMode(control_mode_message, i+1);}
        }

        std::cout << "Torque control example clean exit" << std::endl;

    }
    catch (k_api::KDetailedException& ex)
    {
        std::cout << "API error: " << ex.what() << std::endl;
        return_status = false;
    }
    catch (std::runtime_error& ex2)
    {
        std::cout << "Error: " << ex2.what() << std::endl;
        return_status = false;
    }

    // Set the servoing mode back to Single Level
    servoing_mode.set_servoing_mode(k_api::Base::ServoingMode::SINGLE_LEVEL_SERVOING);
    base->SetServoingMode(servoing_mode);

    // Wait for a bit
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));

    return return_status;
} //Example*


/*****************************
 * In Main function call the controller you would like to run (See next comment section)*
 *****************************/

int main(int argc, char **argv)
{
    // Create API objects
    auto error_callback = [](k_api::KError err){ cout << "_________ callback error _________" << err.toString(); };

    auto transport = new k_api::TransportClientTcp();
    auto router = new k_api::RouterClient(transport, error_callback);
    transport->connect(IP_ADDRESS, PORT);

    auto transport_real_time = new k_api::TransportClientUdp();
    auto router_real_time = new k_api::RouterClient(transport_real_time, error_callback);
    transport_real_time->connect(IP_ADDRESS, PORT_REAL_TIME);

    // Set session data connection information
    auto create_session_info = k_api::Session::CreateSessionInfo();
    create_session_info.set_username("admin");
    create_session_info.set_password("admin");
    create_session_info.set_session_inactivity_timeout(60000);   // (milliseconds)
    create_session_info.set_connection_inactivity_timeout(2000); // (milliseconds)

    // Session manager service wrapper
    std::cout << "Creating sessions for communication" << std::endl;
    auto session_manager = new k_api::SessionManager(router);
    session_manager->CreateSession(create_session_info);
    auto session_manager_real_time = new k_api::SessionManager(router_real_time);
    session_manager_real_time->CreateSession(create_session_info);
    std::cout << "Sessions created" << std::endl;

    // Create services
    auto base = new k_api::Base::BaseClient(router);
    auto base_cyclic = new k_api::BaseCyclic::BaseCyclicClient(router_real_time);
    auto actuator_config = new k_api::ActuatorConfig::ActuatorConfigClient(router);

    /*****************************
     * This is the starting position of the arm, using high level control
     * set each actuator do a desired position CHARS*
    *****************************/
    move_to_desired_angles(base,90,340,214,0,310,90);

    //gripper stuff
    auto parsed_args = ParseExampleArguments(argc, argv);
    GripperLowLevel* gripper_low_level;
    gripper_low_level = new GripperLowLevel(parsed_args.ip_address, PORT_REAL_TIME, PORT,  parsed_args.username, parsed_args.password);
    gripper_low_level->Init(PROPORTIONAL_GAIN);

    /*****************************
      * AND MAKE SURE THE ROBOT IS CLEAR TO RUN WITHOUT INTERFERENCE
      * Replace "ExampleController" with the function name of your controller and hit run (shift+F10) CHARS*
      * set actuators_active to 1 for each actuator you want to control in low level mode
      * ie controlling only the base actuator for the example would look like    int actuators_active [] = {1,0,0,0,0,0};
      * move_to_desired_angles allows you to use high level control to put the arm in position before running a controller
     *****************************/
    int actuators_active [] = {1,0,0,0,0,0};

    trajectory quinticPolynomial;
    quinticPolynomial.actuator = 

    bool success = true;
    success &= gripper_low_level->GoTo(0);
    success &= ExampleController(base, base_cyclic, actuator_config, actuators_active);
    if (!success)
    {std::cout << "There has been an unexpected error in controller function." << endl;}


    // Close API session
    session_manager->CloseSession();
    session_manager_real_time->CloseSession();

    // Deactivate the router and cleanly disconnect from the transport object
    router->SetActivationStatus(false);
    transport->disconnect();
    router_real_time->SetActivationStatus(false);
    transport_real_time->disconnect();

    // Destroy the API
    delete gripper_low_level;
    delete base;
    delete base_cyclic;
    delete actuator_config;
    delete session_manager;
    delete session_manager_real_time;
    delete router;
    delete router_real_time;
    delete transport;
    delete transport_real_time;

    return success ? 0 : 1;
}
