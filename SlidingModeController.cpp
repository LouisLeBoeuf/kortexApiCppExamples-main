
/*
float sgn(float num1) {
    int result;

    if (num1 > 0)
        result = 1;
    if (num1 == 0.0f)
        result = 0;
    if (num1 < 0)
        result = -1;
    return result;

}

bool SlidingModeController(k_api::Base::BaseClient *base, k_api::BaseCyclic::BaseCyclicClient *base_cyclic,k_api::ActuatorConfig::ActuatorConfigClient *actuator_config) {
    bool return_status = true;

    // Get actuator count
    unsigned int actuator_count = base->GetActuatorCount().count();

    // Clearing faults
    try {
        base->ClearFaults();
    }
    catch (...) {
        std::cout << "Unable to clear robot faults" << std::endl;
        return false;
    }


    k_api::BaseCyclic::Feedback base_feedback;
    k_api::BaseCyclic::Command base_command;

    std::vector<float> commands;

    auto servoing_mode = k_api::Base::ServoingModeInformation();

    float timer_count = 0; //changed this to a float instead of integer to use in sin function CHARS
    int64_t now = 0;
    int64_t last = 0;

    /*****************************
     * Declare your variables CHARS*
     *****************************/

    float desired_position[6] = {}; // desired position of the robot [6x1] "variable_name [6] = {};" creates an 6X1 matrix of 0's
    float desired_velocity[6] = {}; // desired velocity of the robot float
    float error_position[6] = {}; // The error in position [6x1]
    float error_velocity[6] = {}; // The error in velocity [6x1]
    float current_position[6] = {}; // current position of each actuator [6x1]
    float current_velocity[6] = {}; // current velocity of each actuator [6x1]
    float torque_sent[6] = {}; //the torque value to send each actuator (calculated by controller) [6x1]
    float current_torque[6] = {}; // current torque of each actuator [6x1]
    float error_r[6] = {};
    float K1[6] = {};
    float K2[6] = {};
    float K3[6] = {};
    float K4[6] = {};
    float PositionGain[6] = {};
    float NormZ[6] = {};

    //the following are vector to store values for writing to files later
    std::vector<float> actual_positions0;
    std::vector<float> actual_velocities0;
    std::vector<float> desired_positions0;
    std::vector<float> desired_velocities0;

    std::vector<float> desired_positions1;
    std::vector<float> actual_positions1;

    std::vector<float> desired_positions2;
    std::vector<float> actual_positions2;

    std::vector<float> desired_positions3;
    std::vector<float> actual_positions3;

    std::vector<float> desired_positions4;
    std::vector<float> actual_positions4;

    std::vector<float> desired_positions5;
    std::vector<float> actual_positions5;

    std::vector<float> desired_positions6;
    std::vector<float> actual_positions6;


    std::cout << "Initializing the arm for torque control example" << std::endl;
    try {

        // Set the base in low-level servoing mode
        servoing_mode.set_servoing_mode(k_api::Base::ServoingMode::LOW_LEVEL_SERVOING);
        base->SetServoingMode(servoing_mode);
        base_feedback = base_cyclic->RefreshFeedback();

        // Initialize each actuator to their current position
        for (unsigned int i = 0; i < actuator_count; i++) {
            commands.push_back(base_feedback.actuators(i).position());

            // Save the current actuator position, to avoid a following error
            base_command.add_actuators()->set_position(base_feedback.actuators(i).position());
        }

        // Send a first frame
        base_feedback = base_cyclic->Refresh(base_command);

        // Set first actuator in torque mode now that the command is equal to measure
        auto control_mode_message = k_api::ActuatorConfig::ControlModeInformation();
        control_mode_message.set_control_mode(k_api::ActuatorConfig::ControlMode::TORQUE);

        /*****************************
         * Please set only the actuators you are controlling to torque mode
         * Uncontrolled actuators in torque mode will be loose (will fall from gravity)
         * Uncomment both lines for each actuator
         * Actuators here are numbered 1 to 6 CHARS*
        *****************************/

        int first_actuator_device_id = 1;
        actuator_config->SetControlMode(control_mode_message, first_actuator_device_id);

        int second_actuator_device_id = 2;
        actuator_config->SetControlMode(control_mode_message, second_actuator_device_id);

        int third_actuator_device_id = 3;
        actuator_config->SetControlMode(control_mode_message, third_actuator_device_id);

        int fourth_actuator_device_id = 4;
        actuator_config->SetControlMode(control_mode_message, fourth_actuator_device_id);

        int fifth_actuator_device_id = 5;
        actuator_config->SetControlMode(control_mode_message, fifth_actuator_device_id);

        int sixth_actuator_device_id = 6;
        actuator_config->SetControlMode(control_mode_message, sixth_actuator_device_id);

        std::cout << "Running torque controller for " << TIME_DURATION << " seconds " << std::endl;

        // Real-time loop
        while (timer_count < (TIME_DURATION * 1000)) {
            now = GetTickUs(); //now and last are both in microseconds CHARS
            //timer count is in milliseconds CHARS

            if (now - last > 1000) {
                // Position command to first actuator is set to measured one to avoid following error to trigger
                // Bonus: When doing this instead of disabling the following error, if communication is lost and first
                //        actuator continues to move under torque command, resulting position error with command will
                //        trigger a following error and switch back the actuator in position command to hold its position
                /*****************************
                 * Uncomment each line below corresponding to each actuator you are controlling to avoid error described above
                 * Actuators here are numbered from 0-5, 0 being the base
                *****************************/

                base_command.mutable_actuators(0)->set_position(base_feedback.actuators(0).position());
                base_command.mutable_actuators(1)->set_position(base_feedback.actuators(1).position());
                base_command.mutable_actuators(2)->set_position(base_feedback.actuators(2).position());
                base_command.mutable_actuators(3)->set_position(base_feedback.actuators(3).position());
                base_command.mutable_actuators(4)->set_position(base_feedback.actuators(4).position());
                base_command.mutable_actuators(5)->set_position(base_feedback.actuators(5).position());

                // Incrementing identifier ensures actuators can reject out of time frames
                base_command.set_frame_id(base_command.frame_id() + 1);
                if (base_command.frame_id() > 65535)
                    base_command.set_frame_id(0);

                /*****************************
                 * Below set your desired trajectories (0 to 360 degrees) and desired velocities (-25 to 25 deg/s)
                 * timer_count counts up in milliseconds, up to the TIME_DURATION set above
                 * ie: if TIME_DURATION = 15, timer_count counts from 0 to 15,000 CHARS*
                *****************************/

                desired_position[0] = 270 + 40 * sin(timer_count / 1000 /
                                                     2); // added the divide by 2 to get roughly 2.5 cycles in 15 seconds Range: [70-110deg] CHARS
                desired_velocity[0] = 1000 * 40 * cos(timer_count / 1000 / 2) / 2000; // Range: [-20,20] CHARS

                desired_position[1] = 300 + 30 * sin(timer_count / 1000 / 2);
                desired_velocity[1] = 1000 * 30 * cos(timer_count / 1000 / 2) / 2000;

                desired_position[2] = 280 + 40 * sin(timer_count / 1000 / 2);
                desired_velocity[2] = 1000 * 40 * cos(timer_count / 1000 / 2) / 2000;

                desired_position[3] = 180 + 40 * sin(timer_count / 1000 / 2);
                desired_velocity[3] = 1000 * 40 * cos(timer_count / 1000 / 2) / 2000;

                desired_position[4] = 60 + 40 * sin(timer_count / 1000 / 2);
                desired_velocity[4] = 1000 * 40 * cos(timer_count / 1000 / 2) / 2000;

                desired_position[5] = 90 + 40 * sin(timer_count / 1000 / 2);
                desired_velocity[5] = 1000 * 40 * cos(timer_count / 1000 / 2) / 2000;

                PositionGain[0] = 2.5;
                K1[0] = 1;
                K2[0] = 0.01;
                K3[0] = 0.01;
                K4[0] = 0.001;

                PositionGain[1] = 6;
                K1[1] = 2;
                K2[1] = 0.01;
                K3[1] = 0.01;
                K4[1] = 0.001;

                PositionGain[2] = 7;
                K1[2] = 1.75;
                K2[2] = 0.01;
                K3[2] = 0.01;
                K4[2] = 0.001;

                PositionGain[3] = 1.5;
                K1[3] = 1;
                K2[3] = 0.01;
                K3[3] = 0.01;
                K4[3] = 0.001;

                PositionGain[4] = 1.5;
                K1[4] = 1;
                K2[4] = 0.01;
                K3[4] = 0.01;
                K4[4] = 0.001;

                PositionGain[5] = 1.5;
                K1[5] = 1;
                K2[5] = 0.01;
                K3[5] = 0.01;
                K4[5] = 0.001;

                // this saves the current positions and velocities of each actuator and calculates the error between them CHARS
                for (unsigned int i = 0; i < actuator_count; i++) {
                    current_position[i] = base_feedback.actuators(i).position();
                    current_velocity[i] = base_feedback.actuators(i).velocity();
                    error_position[i] = desired_position[i] - current_position[i];
                    error_velocity[i] = desired_velocity[i] - current_velocity[i];

                    error_r[i] = error_velocity[i] + PositionGain[i] * error_position[i];
                    NormZ[i] = sqrtf(error_position[i] * error_position[i] + error_r[i] * error_r[i]);

                    torque_sent[i] = K1[i] * error_r[i] + error_position[i] +
                                     (K2[i] + K3[i] * NormZ[i] + K4[i] * NormZ[i] * NormZ[i]) * sgn(error_r[i]);

                    //saturate the torque sent
                    if (torque_sent[i] > 20) { torque_sent[i] = 20; };
                    if (torque_sent[i] < -20) { torque_sent[i] = -20; };
                }


                /*****************************
                 * Below appends values to these vectors
                 * The vectors are just used to store values to write to a file CHARS*
                *****************************/

                actual_positions0.push_back(current_position[0]);
                actual_velocities0.push_back(current_velocity[0]);
                desired_positions0.push_back(desired_position[0]);
                desired_velocities0.push_back(desired_velocity[0]);
                actual_positions1.push_back(current_position[1]);
                desired_positions1.push_back(desired_position[1]);
                actual_positions2.push_back(current_position[2]);
                desired_positions2.push_back(desired_position[2]);
                actual_positions3.push_back(current_position[3]);
                desired_positions3.push_back(desired_position[3]);
                actual_positions4.push_back(current_position[4]);
                desired_positions4.push_back(desired_position[4]);
                actual_positions5.push_back(current_position[5]);
                desired_positions5.push_back(desired_position[5]);

                /*****************************
                 * Send the torque to the actuators you are controlling (uncomment each respective line)
                 * Actuators are called 0 to 5 here CHARS*
                *****************************/

                // This sets the torque of actuators CHARS
                base_command.mutable_actuators(0)->set_torque_joint(torque_sent[0]);

                base_command.mutable_actuators(1)->set_torque_joint(torque_sent[1]);

                base_command.mutable_actuators(2)->set_torque_joint(torque_sent[2]);

                base_command.mutable_actuators(3)->set_torque_joint(torque_sent[3]);

                base_command.mutable_actuators(4)->set_torque_joint(torque_sent[4]);

                base_command.mutable_actuators(5)->set_torque_joint(torque_sent[5]);


                /*****************************
                 * Here are some useful print commands CHARS*
                *****************************/
                //std::cout << "error in position = " << error_position[0] << std::endl;
                //std::cout << "error in velocity = " << error_velocity[0] << std::endl;
                //std::cout << "desired position =" << desired_position[0] << std::endl;
                //std::cout << "Timer Count =" << timer_count << std::endl;
                std::cout << "Torque sent to 0 actuator" << torque_sent[0] << std::endl;
                std::cout << "Torque sent to 1 actuator" << torque_sent[1] << std::endl;
                std::cout << "Torque sent to 2 actuator" << torque_sent[2] << std::endl;
                std::cout << "Torque sent to 3 actuator" << torque_sent[3] << std::endl;
                std::cout << "Torque sent to 4 actuator" << torque_sent[4] << std::endl;
                std::cout << "Torque sent to 5 actuator" << torque_sent[5] << std::endl;
                //std::cout << "NormZ=" << NormZ[0] << std::endl;


                for (int idx = 0; idx < actuator_count; idx++) {
                    base_command.mutable_actuators(idx)->set_command_id(base_command.frame_id());
                }

                try {
                    base_feedback = base_cyclic->Refresh(base_command, 0);
                }
                catch (k_api::KDetailedException &ex) {
                    std::cout << "Kortex exception: " << ex.what() << std::endl;

                    std::cout << "Error sub-code: " << k_api::SubErrorCodes_Name(
                            k_api::SubErrorCodes((ex.getErrorInfo().getError().error_sub_code()))) << std::endl;
                }
                catch (std::runtime_error &ex2) {
                    std::cout << "runtime error: " << ex2.what() << std::endl;
                }
                catch (...) {
                    std::cout << "Unknown error." << std::endl;
                }

                timer_count++;
                last = GetTickUs();
            }
        }

        /*****************************
         * Below writes your vectors to files, separated by lines
         * No need to modify this unless you get an error CHARS*
        *****************************/

        ofstream fw("C:\\Users\\Chars-local\\Desktop\\KinovaOutputData\\0ActualPositions.txt", std::ofstream::out);
        if (fw.is_open()) {
            //store array contents to text file
            for (int i = 0; i < actual_positions0.size(); i++) {
                fw << actual_positions0.at(i) << "\n";
            }
            fw.close();
        } else cout << "Problem with opening 0ActualPositions file";

        ofstream fww("C:\\Users\\Chars-local\\Desktop\\KinovaOutputData\\0ActualVelocities.txt", std::ofstream::out);
        if (fww.is_open()) {
            //store array contents to text file
            for (int i = 0; i < actual_velocities0.size(); i++) {
                fww << actual_velocities0.at(i) << "\n";
            }
            fww.close();
        } else cout << "Problem with opening 0ActualVelocities file";

        ofstream fwww("C:\\Users\\Chars-local\\Desktop\\KinovaOutputData\\0DesiredPositions.txt", std::ofstream::out);
        if (fwww.is_open()) {
            //store array contents to text file
            for (int i = 0; i < desired_positions0.size(); i++) {
                fwww << desired_positions0.at(i) << "\n";
            }
            fwww.close();
        } else cout << "Problem with opening DesiredPositions0 file";

        ofstream fwwww("C:\\Users\\Chars-local\\Desktop\\KinovaOutputData\\0DesiredVelocities.txt", std::ofstream::out);
        if (fwwww.is_open()) {
            //store array contents to text file
            for (int i = 0; i < desired_velocities0.size(); i++) {
                fwwww << desired_velocities0.at(i) << "\n";
            }
            fwwww.close();
        } else cout << "Problem with opening 0DesiredVelocities file";

        std::cout << "Torque control example completed" << std::endl;

        ofstream fa("C:\\Users\\Chars-local\\Desktop\\KinovaOutputData\\1ActualPositions.txt", std::ofstream::out);
        if (fa.is_open()) {
            //store array contents to text file
            for (int i = 0; i < actual_positions1.size(); i++) {
                fa << actual_positions1.at(i) << "\n";
            }
            fa.close();
        } else cout << "Problem with opening 1ActualPositions file";

        ofstream fb("C:\\Users\\Chars-local\\Desktop\\KinovaOutputData\\1DesiredPositions.txt", std::ofstream::out);
        if (fb.is_open()) {
            //store array contents to text file
            for (int i = 0; i < desired_positions1.size(); i++) {
                fb << desired_positions1.at(i) << "\n";
            }
            fb.close();
        } else cout << "Problem with opening 1DesiredPositions file";


        ofstream fc("C:\\Users\\Chars-local\\Desktop\\KinovaOutputData\\2ActualPositions.txt", std::ofstream::out);
        if (fc.is_open()) {
            //store array contents to text file
            for (int i = 0; i < actual_positions2.size(); i++) {
                fc << actual_positions2.at(i) << "\n";
            }
            fc.close();
        } else cout << "Problem with opening 2ActualPositions file";

        ofstream fd("C:\\Users\\Chars-local\\Desktop\\KinovaOutputData\\2DesiredPositions.txt", std::ofstream::out);
        if (fd.is_open()) {
            //store array contents to text file
            for (int i = 0; i < desired_positions2.size(); i++) {
                fd << desired_positions2.at(i) << "\n";
            }
            fd.close();
        } else cout << "Problem with opening 2DesiredPositions file";


        ofstream fe("C:\\Users\\Chars-local\\Desktop\\KinovaOutputData\\3ActualPositions.txt", std::ofstream::out);
        if (fe.is_open()) {
            //store array contents to text file
            for (int i = 0; i < actual_positions3.size(); i++) {
                fe << actual_positions3.at(i) << "\n";
            }
            fe.close();
        } else cout << "Problem with opening 3ActualPositions file";

        ofstream ff("C:\\Users\\Chars-local\\Desktop\\KinovaOutputData\\3DesiredPositions.txt", std::ofstream::out);
        if (ff.is_open()) {
            //store array contents to text file
            for (int i = 0; i < desired_positions3.size(); i++) {
                ff << desired_positions3.at(i) << "\n";
            }
            ff.close();
        } else cout << "Problem with opening 3DesiredPositions file";


        ofstream fg("C:\\Users\\Chars-local\\Desktop\\KinovaOutputData\\4ActualPositions.txt", std::ofstream::out);
        if (fg.is_open()) {
            //store array contents to text file
            for (int i = 0; i < actual_positions4.size(); i++) {
                fg << actual_positions4.at(i) << "\n";
            }
            fg.close();
        } else cout << "Problem with opening 4ActualPositions file";

        ofstream fh("C:\\Users\\Chars-local\\Desktop\\KinovaOutputData\\4DesiredPositions.txt", std::ofstream::out);
        if (fh.is_open()) {
            //store array contents to text file
            for (int i = 0; i < desired_positions4.size(); i++) {
                fh << desired_positions4.at(i) << "\n";
            }
            fh.close();
        } else cout << "Problem with opening 4DesiredPositions file";


        ofstream fi("C:\\Users\\Chars-local\\Desktop\\KinovaOutputData\\5ActualPositions.txt", std::ofstream::out);
        if (fi.is_open()) {
            //store array contents to text file
            for (int i = 0; i < actual_positions5.size(); i++) {
                fi << actual_positions5.at(i) << "\n";
            }
            fi.close();
        } else cout << "Problem with opening 5ActualPositions file";

        ofstream fj("C:\\Users\\Chars-local\\Desktop\\KinovaOutputData\\5DesiredPositions.txt", std::ofstream::out);
        if (fj.is_open()) {
            //store array contents to text file
            for (int i = 0; i < desired_positions5.size(); i++) {
                fj << desired_positions5.at(i) << "\n";
            }
            fj.close();
        } else cout << "Problem with opening 5DesiredPositions file";


        /*****************************
        * Please set only the actuators you controlled back to position control
        * Uncomment both lines for each actuator CHARS*
         *****************************/

        // Set each actuator back to position control
        control_mode_message.set_control_mode(k_api::ActuatorConfig::ControlMode::POSITION);
        actuator_config->SetControlMode(control_mode_message, first_actuator_device_id);

        control_mode_message.set_control_mode(k_api::ActuatorConfig::ControlMode::POSITION);
        actuator_config->SetControlMode(control_mode_message, second_actuator_device_id);

        control_mode_message.set_control_mode(k_api::ActuatorConfig::ControlMode::POSITION);
        actuator_config->SetControlMode(control_mode_message, third_actuator_device_id);

        control_mode_message.set_control_mode(k_api::ActuatorConfig::ControlMode::POSITION);
        actuator_config->SetControlMode(control_mode_message, fourth_actuator_device_id);

        control_mode_message.set_control_mode(k_api::ActuatorConfig::ControlMode::POSITION);
        actuator_config->SetControlMode(control_mode_message, fifth_actuator_device_id);

        control_mode_message.set_control_mode(k_api::ActuatorConfig::ControlMode::POSITION);
        actuator_config->SetControlMode(control_mode_message, sixth_actuator_device_id);

        std::cout << "Torque control example clean exit" << std::endl;

    }
    catch (k_api::KDetailedException &ex) {
        std::cout << "API error: " << ex.what() << std::endl;
        return_status = false;
    }
    catch (std::runtime_error &ex2) {
        std::cout << "Error: " << ex2.what() << std::endl;
        return_status = false;
    }

    // Set the servoing mode back to Single Level
    servoing_mode.set_servoing_mode(k_api::Base::ServoingMode::SINGLE_LEVEL_SERVOING);
    base->SetServoingMode(servoing_mode);

    // Wait for a bit
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));

    return return_status;
}