
/*
bool ExampleController(k_api::Base::BaseClient *base, k_api::BaseCyclic::BaseCyclicClient *base_cyclic,k_api::ActuatorConfig::ActuatorConfigClient *actuator_config, int actuators_active[6] = {}) {
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
    //float current_torque [6] = {}; // current torque of each actuator [6x1]
    float Kp[6] = {}; //The kp, ki, and kd values for each actuator
    //float Ki [6] = {};
    float Kd[6] = {};

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

        //sets each actuator you specified to low level control mode
        for (unsigned int i = 0; i < actuator_count; i++) {
            if (actuators_active[i] > 0) {
                actuator_config->SetControlMode(control_mode_message, i + 1);
            }
        };

        std::cout << "Running torque controller for " << TIME_DURATION << " seconds " << std::endl;

        // Real-time loop
        while (timer_count < (TIME_DURATION * 1000)) {
            now = GetTickUs(); //now and last are both in microseconds CHARS
            //timer count is in milliseconds CHARS

            if (now - last > 1000) {
                // Position command to first actuator is set to measured one to avoid following error to trigger
                // Bonus: When doing this instead of disabling the following error, if communication is lost and
                //        actuator continues to move under torque command, resulting position error with command will
                //        trigger a following error and switch back the actuator in position command to hold its position
                for (unsigned int i = 0; i < actuator_count; i++) {
                    if (actuators_active[i] > 0) {
                        base_command.mutable_actuators(i)->set_position(base_feedback.actuators(i).position());
                    }
                }

                // Incrementing identifier ensures actuators can reject out of time frames
                base_command.set_frame_id(base_command.frame_id() + 1);
                if (base_command.frame_id() > 65535)
                    base_command.set_frame_id(0);

                /*****************************
                 * Below set your desired trajectories (with a range from 0 to 360 degrees) and desired velocities (-25 to 25 deg/s)
                 * The position is computed either as a static value (ex 10) or as a function (sin fxn ex given in code)
                 * Calculate the desired velocity as the derivative of the position function unless a static value is desired
                 * timer_count counts up in milliseconds, up to the TIME_DURATION set before
                 * ie: if TIME_DURATION = 15, timer_count counts from 0 to 15,000 CHARS
                 * DO NOT PUT A TRAJECTORY THAT PASSES THROUGH 0 or 360!!!*
                *****************************/

                desired_position[0] = 90 + 20 * sin(timer_count / 1000 /
                                                    2); // added the divide by 2 to get roughly 2.5 cycles in 15 seconds Range: [50-130deg] CHARS
                desired_velocity[0] = 1000 * 20 * cos(timer_count / 1000 / 2) / 2000;

                //desired_position[2] = 300+sin(timer_count/1000/2);
                //desired_velocity[2] = 1000*40*cos(timer_count/1000/2)/2000;

                //desired_position[3] = 180+40*sin(timer_count/1000/2);
                //desired_velocity[3] = 1000*40*cos(timer_count/1000/2)/2000;

                //desired_position[4] = 90+40*sin(timer_count/1000/2);
                //desired_velocity[4] = 1000*40*cos(timer_count/1000/2)/2000;

                //desired_position[5] = 180+40*sin(timer_count/1000/2);
                //desired_velocity[5] = 1000*40*cos(timer_count/1000/2)/2000;


                //kd and kp values
                Kp[0] = 0.45; //gives up to 21 with 360 degree error
                Kd[0] = 0.2;

                //Saves current positions and velocities of each actuator and calculates the error between them CHARS
                for (unsigned int i = 0; i < actuator_count; i++) {
                    current_position[i] = base_feedback.actuators(i).position();
                    current_velocity[i] = base_feedback.actuators(i).velocity();
                    error_position[i] = desired_position[i] - current_position[i];
                    error_velocity[i] = desired_velocity[i] - current_velocity[i];
                    torque_sent[i] = Kp[i] * error_position[i] + Kd[i] * error_velocity[i];
                    if (torque_sent[i] > 30) { torque_sent[i] = 30; };
                    if (torque_sent[i] < -30) { torque_sent[i] = -30; };
                }

                /*****************************
                 * Below appends values to these vectors used to store values to write to a file*
                *****************************/
                if (write_data == true) {
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
                    controller_output5.push_back(torque_sent[5]);
                }

                //sends the torque to each actuator that is in low level control mode
                for (unsigned int i = 0; i < actuator_count; i++) {
                    if (actuators_active[i] > 0) {
                        base_command.mutable_actuators(i)->set_torque_joint(torque_sent[i]);
                    }
                }

                /*****************************
                 * Here are some useful print commands
                //std::cout << "Actuator 0 error in position = " << error_position[0] << std::endl;
                //std::cout << "Actuator 0 error in velocity = " << error_velocity[0] << std::endl;
                //std::cout << "Actuator 0 desired position =" << desired_position[0] << std::endl;
                //std::cout << "Timer Count =" << timer_count << std::endl;
                //std::cout << "Actuator 0 Torque Sent =" << torque_sent[0] << std::endl;*
                *****************************/

                for (unsigned int idx = 0; idx < actuator_count; idx++) {
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

        //Below writes your data vectors to files, separated by lines
        if (write_data == true) {

            ofstream fw("C:\\Users\\Chars-local\\Desktop\\KinovaOutputData\\0ActualPositions.txt", std::ofstream::out);
            if (fw.is_open()) {
                //store array contents to text file
                for (unsigned int i = 0; i < actual_positions0.size(); i++) {
                    fw << actual_positions0.at(i) << "\n";
                }
                fw.close();
            } else cout << "Problem with opening 0ActualPositions file";

            ofstream fww("C:\\Users\\Chars-local\\Desktop\\KinovaOutputData\\0ActualVelocities.txt",
                         std::ofstream::out);
            if (fww.is_open()) {
                //store array contents to text file
                for (unsigned int i = 0; i < actual_velocities0.size(); i++) {
                    fww << actual_velocities0.at(i) << "\n";
                }
                fww.close();
            } else cout << "Problem with opening 0ActualVelocities file";

            ofstream fwww("C:\\Users\\Chars-local\\Desktop\\KinovaOutputData\\0DesiredPositions.txt",
                          std::ofstream::out);
            if (fwww.is_open()) {
                //store array contents to text file
                for (unsigned int i = 0; i < desired_positions0.size(); i++) {
                    fwww << desired_positions0.at(i) << "\n";
                }
                fwww.close();
            } else cout << "Problem with opening DesiredPositions0 file";

            ofstream fwwww("C:\\Users\\Chars-local\\Desktop\\KinovaOutputData\\0DesiredVelocities.txt",
                           std::ofstream::out);
            if (fwwww.is_open()) {
                //store array contents to text file
                for (unsigned int i = 0; i < desired_velocities0.size(); i++) {
                    fwwww << desired_velocities0.at(i) << "\n";
                }
                fwwww.close();
            } else cout << "Problem with opening 0DesiredVelocities file";

            ofstream faa("C:\\Users\\Chars-local\\Desktop\\KinovaOutputData\\0ControllerOutput.txt",
                         std::ofstream::out);
            if (faa.is_open()) {
                //store array contents to text file
                for (unsigned int i = 0; i < controller_output0.size(); i++) {
                    faa << controller_output0.at(i) << "\n";
                }
                faa.close();
            } else cout << "Problem with opening 0ControllerOutput.txt file";


            ofstream fa("C:\\Users\\Chars-local\\Desktop\\KinovaOutputData\\1ActualPositions.txt", std::ofstream::out);
            if (fa.is_open()) {
                //store array contents to text file
                for (unsigned int i = 0; i < actual_positions1.size(); i++) {
                    fa << actual_positions1.at(i) << "\n";
                }
                fa.close();
            } else cout << "Problem with opening 1ActualPositions file";

            ofstream fb("C:\\Users\\Chars-local\\Desktop\\KinovaOutputData\\1DesiredPositions.txt", std::ofstream::out);
            if (fb.is_open()) {
                //store array contents to text file
                for (unsigned int i = 0; i < desired_positions1.size(); i++) {
                    fb << desired_positions1.at(i) << "\n";
                }
                fb.close();
            } else cout << "Problem with opening 1DesiredPositions file";

            ofstream fcc("C:\\Users\\Chars-local\\Desktop\\KinovaOutputData\\1ControllerOutput.txt",
                         std::ofstream::out);
            if (fcc.is_open()) {
                //store array contents to text file
                for (unsigned int i = 0; i < controller_output1.size(); i++) {
                    fcc << controller_output1.at(i) << "\n";
                }
                fcc.close();
            } else cout << "Problem with opening 1ControllerOutput.txt file";


            ofstream fc("C:\\Users\\Chars-local\\Desktop\\KinovaOutputData\\2ActualPositions.txt", std::ofstream::out);
            if (fc.is_open()) {
                //store array contents to text file
                for (unsigned int i = 0; i < actual_positions2.size(); i++) {
                    fc << actual_positions2.at(i) << "\n";
                }
                fc.close();
            } else cout << "Problem with opening 2ActualPositions file";

            ofstream fd("C:\\Users\\Chars-local\\Desktop\\KinovaOutputData\\2DesiredPositions.txt", std::ofstream::out);
            if (fd.is_open()) {
                //store array contents to text file
                for (unsigned int i = 0; i < desired_positions2.size(); i++) {
                    fd << desired_positions2.at(i) << "\n";
                }
                fd.close();
            } else cout << "Problem with opening 2DesiredPositions file";

            ofstream fee("C:\\Users\\Chars-local\\Desktop\\KinovaOutputData\\2ControllerOutput.txt",
                         std::ofstream::out);
            if (fee.is_open()) {
                //store array contents to text file
                for (unsigned int i = 0; i < controller_output2.size(); i++) {
                    fee << controller_output2.at(i) << "\n";
                }
                fee.close();
            } else cout << "Problem with opening 2ControllerOutput file";


            ofstream fe("C:\\Users\\Chars-local\\Desktop\\KinovaOutputData\\3ActualPositions.txt", std::ofstream::out);
            if (fe.is_open()) {
                //store array contents to text file
                for (unsigned int i = 0; i < actual_positions3.size(); i++) {
                    fe << actual_positions3.at(i) << "\n";
                }
                fe.close();
            } else cout << "Problem with opening 3ActualPositions file";

            ofstream ff("C:\\Users\\Chars-local\\Desktop\\KinovaOutputData\\3DesiredPositions.txt", std::ofstream::out);
            if (ff.is_open()) {
                //store array contents to text file
                for (unsigned int i = 0; i < desired_positions3.size(); i++) {
                    ff << desired_positions3.at(i) << "\n";
                }
                ff.close();
            } else cout << "Problem with opening 3DesiredPositions file";

            ofstream fgg("C:\\Users\\Chars-local\\Desktop\\KinovaOutputData\\3ControllerOutput.txt",
                         std::ofstream::out);
            if (fgg.is_open()) {
                //store array contents to text file
                for (unsigned int i = 0; i < controller_output3.size(); i++) {
                    fgg << controller_output3.at(i) << "\n";
                }
                fgg.close();
            } else cout << "Problem with opening 3ControllerOutput.txt file";


            ofstream fg("C:\\Users\\Chars-local\\Desktop\\KinovaOutputData\\4ActualPositions.txt", std::ofstream::out);
            if (fg.is_open()) {
                //store array contents to text file
                for (unsigned int i = 0; i < actual_positions4.size(); i++) {
                    fg << actual_positions4.at(i) << "\n";
                }
                fg.close();
            } else cout << "Problem with opening 4ActualPositions file";

            ofstream fh("C:\\Users\\Chars-local\\Desktop\\KinovaOutputData\\4DesiredPositions.txt", std::ofstream::out);
            if (fh.is_open()) {
                //store array contents to text file
                for (unsigned int i = 0; i < desired_positions4.size(); i++) {
                    fh << desired_positions4.at(i) << "\n";
                }
                fh.close();
            } else cout << "Problem with opening 4DesiredPositions file";

            ofstream fii("C:\\Users\\Chars-local\\Desktop\\KinovaOutputData\\4ControllerOutput.txt",
                         std::ofstream::out);
            if (fii.is_open()) {
                //store array contents to text file
                for (unsigned int i = 0; i < controller_output4.size(); i++) {
                    fii << controller_output4.at(i) << "\n";
                }
                fii.close();
            } else cout << "Problem with opening 4ControllerOutput.txt file";


            ofstream fi("C:\\Users\\Chars-local\\Desktop\\KinovaOutputData\\5ActualPositions.txt", std::ofstream::out);
            if (fi.is_open()) {
                //store array contents to text file
                for (unsigned int i = 0; i < actual_positions5.size(); i++) {
                    fi << actual_positions5.at(i) << "\n";
                }
                fi.close();
            } else cout << "Problem with opening 5ActualPositions file";

            ofstream fj("C:\\Users\\Chars-local\\Desktop\\KinovaOutputData\\5DesiredPositions.txt", std::ofstream::out);
            if (fj.is_open()) {
                //store array contents to text file
                for (unsigned int i = 0; i < desired_positions5.size(); i++) {
                    fj << desired_positions5.at(i) << "\n";
                }
                fj.close();
            } else cout << "Problem with opening 5DesiredPositions file";

            ofstream fk("C:\\Users\\Chars-local\\Desktop\\KinovaOutputData\\5ControllerOutput.txt", std::ofstream::out);
            if (fk.is_open()) {
                //store array contents to text file
                for (unsigned int i = 0; i < controller_output5.size(); i++) {
                    fk << controller_output5.at(i) << "\n";
                }
                fk.close();
            } else cout << "Problem with opening 5ControllerOutput.txt file";
        }

        //sets each actuator back to high level control mode
        for (unsigned int i = 0; i < actuator_count; i++) {
            if (actuators_active[i] > 0) {
                control_mode_message.set_control_mode(k_api::ActuatorConfig::ControlMode::POSITION);
                actuator_config->SetControlMode(control_mode_message, i + 1);
            }
        }

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
} //Example*
