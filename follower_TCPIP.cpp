// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE

/////////////////                                                      Combine codes here
// This code polls the follower robot joint torques and sends them to the leader via tcp/ip wireless connection
// The polling is done using readOnce

#include <iostream>

#include <Eigen/Core>

#include <franka/duration.h>
#include <franka/exception.h>
#include <franka/model.h>
#include <franka/robot.h>

#include "examples_common.h"
#include <stdio.h> 
#include <sys/socket.h> 
#include <arpa/inet.h> 
#include <unistd.h> 
#include <string.h> 
#include <limits>
#include <thread>
#include <array>
#include <mutex>
#include <atomic>
#include "examples_common.h"
#include <franka/model.h>
#include <franka/rate_limiting.h>
#include <franka/duration.h>
#include <cmath>
#include <functional>

#include <fstream>
#include <iterator>
#include <algorithm>

#include "signal.h"    // for catching exit signals
#include "stdlib.h"
#include "ctype.h"
#include "sys/time.h"

#include <ncurses.h>
#include <assert.h>
#include <franka/gripper.h>

#include <vector>
#include <string>
#include <sstream>

#define PORT 8080
#define N 8 

using namespace std;
//using std::cout;
//using std::endl;

int sock = 0, valread; 
struct sockaddr_in serv_addr; 


/* ********************************************** A class to create and write data in a csv file. *****************************************************************/
class writeToCsv
{
    std::string fileName;
    std::string delimiter;
    int linesCount;

	public:
		writeToCsv(std::string filename, std::string delm = ",") :
				fileName(filename), delimiter(delm), linesCount(0)
		{}
    /*
     * Member function to store a range as comma seperated value
     */
    template<typename T>
    void addDatainRow(T first, T last);
};
/*
 * This Function accepts a range and appends all the elements in the range
 * to the last row, seperated by delimiter (Default is comma)
 */
template<typename T>
void writeToCsv::addDatainRow(T first, T last)
{
    std::fstream file;
    // Open the file in truncate mode if first line else in Append Mode
    file.open(fileName, std::ios::out | (linesCount ? std::ios::app : std::ios::trunc));
    // Iterate over the range and add each lement to file seperated by delimiter.
    for (; first != last; )
    {
        file << *first;
        if (++first != last)
            file << delimiter;
    }
    file << "\n";
    linesCount++;
    // Close the file
    file.close();
}

/* ********************************************** A class to create and write data in a csv file. *****************************************************************/

std::vector<std::string> timestamp()
{
	//Declare argument for time
	time_t tt;
	
	//Declare a variable to store return value of localtime(). It is independent of system time
	struct tm * _ti;
	
	//Apply time()
	time (&tt);
	
	//Using localtime()
	_ti = localtime(&tt);
	
	string timeee = asctime(_ti);
	
	// split string timeee into time day and year
	string buf;                                      // buffer string
	std::stringstream ss(timeee);                         // Insert the string into a stream
	
	std::vector<std::string> _bits;                 //Create a vector called _bits to hold split words
	
	while (ss >> buf)
		_bits.push_back(buf);
	
	return _bits;
}

int main(int argc, char** argv) 
{
	vector<array<double, N> > logPos; 
	vector<array<double, N> > logVel; 
	
	vector<array<double, 3> > sendAndReceiveTimeLog;
	sendAndReceiveTimeLog.push_back({0.0, 0.0, 0.0});
	
	string robot_address = argv[1];

	using std::chrono::high_resolution_clock;
    using std::chrono::duration_cast;
    using std::chrono::duration;
    using std::chrono::milliseconds;
	double buffer[1024] = {0}; 
	// Check whether the required arguments were passed  
	if (argc != 2) 
	{
		std::cerr << "Usage: " << argv[0] << " <robot-hostname>" << std::endl;
		return -1;
	}
	
	const double print_rate = 20.0;
	const double log_rate = 20.0;
	
	//Initialise data fields for the polling and communication thread
	struct
	{
		std::mutex mutex;
		bool has_data;
		std::array<double, 14> leaderJointPositionAndVelocity;
		std::array<double, 7> leaderJointPosition;
		std::array<double, 7> leaderJointVelocity;
		franka::RobotState robot_state;
	}joint_parameters{};
	
	struct
	{
		std::mutex mutex;
		bool has_data;
		franka::RobotState robot_state;
		franka::GripperState gripper_state;
	}log_parameters{};
	
	std::atomic_bool running{true};
	
	std::thread pollFollowerParametersAndCommunicateWithLeader([print_rate, &running, &joint_parameters, &buffer, &sendAndReceiveTimeLog]()
	{
		auto loop_time = high_resolution_clock::now();
		while(running)         // while true
		{
			// Make thread sleep for specific amount of time to control the cycle rate
			std::this_thread::sleep_for(
			std::chrono::milliseconds(static_cast<int>((1.0 / print_rate * 1000.0))));
			std::array<double, 3> sendAndReceiveTime;
			
			//Try to lock  data to avoid read write collisions
			if(joint_parameters.mutex.try_lock())
			{			
				if(joint_parameters.has_data) // Set true in the control loop
				{
					//Create standard arrays in which to copy the follower robot's joint parameters
					std::array<double, 7> followerExternalJointTorque;
					std::array<double, 7> followerjointPosition;
					std::array<double, 7> followerjointVelocity;
				
					for(size_t i=0; i<7; i++)
					{
						followerExternalJointTorque[i] = joint_parameters.robot_state.tau_ext_hat_filtered[i]; // Poll the follower robot's external joints' torques
						followerjointPosition[i] = joint_parameters.robot_state.q[i]; // Poll the follower robot's joints' positions
						followerjointVelocity[i] = joint_parameters.robot_state.dq[i];	// Poll the follower robot's joints' velocities
					}
					
					auto s_r_t1 = high_resolution_clock::now();                                                                                    //Timer
					/////////////////////////////////////////////   Send follower robot external joint torques to leader robot   ///////////
					send(sock , (char *) &followerExternalJointTorque, sizeof(followerExternalJointTorque) , 0 ); 
					/////////////////////////////////////////////   Send follower robot joint torque to leader    ///////////	
					auto s_r_t2 = high_resolution_clock::now();                                                                                    //Timer
					
					
					///////////////////////////////////////////// Read incoming messages and save into an array   ///////////
					bzero(buffer,1024);     //Clear buffer
					valread = read( sock , buffer, 1024); 
					auto s_r_t3 = high_resolution_clock::now();                                                                                    //Timer
					if(valread != 0)                              // If there are packets received
					{
						// Copy received data from buffer to an array to be accessed in the impedance control callback
						//The joint_parameters struct can be used to send data to and from the thread
						for (int i = 0; i<14; i++ )
						{	
							joint_parameters.leaderJointPositionAndVelocity[i] = buffer[i];  
							//std::cout << joint_parameters.leaderJointPositionAndVelocity[i];
						}
						//Split the received data into joint positions and velocities
						for(int i = 0; i < 7; i++)
						{
							joint_parameters.leaderJointPosition[i] = joint_parameters.leaderJointPositionAndVelocity[i];
							joint_parameters.leaderJointVelocity[i] = joint_parameters.leaderJointPositionAndVelocity[i+7];
						}
						
						//Print out received parameters
						std::cout << "Received Leader Joints' Positions and Velocities: " ;
						for (int i = 0; i<14; i++ )
						{	
							std::cout << joint_parameters.leaderJointPositionAndVelocity[i];
							if(i<13)
							{
								std::cout << ", ";	
							}
						}
						printf("\n");
						std::cout << "Leader Joints' Positions " ;
						for (int i = 0; i<7; i++ )
						{	
							std::cout << joint_parameters.leaderJointPosition[i];
							if(i<6)
							{
								std::cout << ", ";	
							}
						}
						printf("\n");
						std::cout << "Leader Joints' Velocities " ;
						for (int i = 0; i<7; i++ )
						{	
							std::cout << joint_parameters.leaderJointVelocity[i];
							if(i<6)
							{
								std::cout << ", ";	
							}
						}
						printf("\n");	
					}	
					else
					{
						cout << "threads exited: No packets received" << endl;
						exit (-1);
					}	
					///////////////////////////////////////////// Read incoming messages and save into an array   ///////////

					joint_parameters.has_data = false;
					
					duration<double, std::milli>	t_send = s_r_t2 - s_r_t1;	
					duration<double, std::milli>	t_receive = s_r_t3 - s_r_t2;
					std::cout << "Send: " << t_send.count() << " ms" << std::endl;
					std::cout << "Receive: " << t_receive.count() << " ms" << std::endl;
					
					//std::array<double, 3> sendAndReceiveTime;
					sendAndReceiveTime[0] = t_send.count();
					sendAndReceiveTime[1] = t_receive.count();
					
					sendAndReceiveTimeLog.push_back(sendAndReceiveTime);
					
					
					duration<double, std::milli>  t_loop = high_resolution_clock::now() - loop_time;
					loop_time = high_resolution_clock::now();
					
					sendAndReceiveTime[2] = t_loop.count();
					sendAndReceiveTimeLog.push_back(sendAndReceiveTime);
					
					printf("Loop: %f ms  \n", t_loop.count());	
					
					
					joint_parameters.has_data = false;
				} //has data

				joint_parameters.mutex.unlock();
			}// Mutex lock
			
			
			
		}// End of while loop
		
	}); // End of thread pollFollowerExternalJointTorques
	
		//THREAD FOR Logging robot parameters
	std::thread log([log_rate, &running, &log_parameters, &logPos, &logVel, &robot_address, &sendAndReceiveTimeLog]()
	{
		franka::Gripper gripper(robot_address);
		franka::GripperState gripper_state;
		
		gripper.homing();
		
		// TIME KEEPING
		std::vector<std::string> _time_stamp = timestamp();		
		string times_tamp = _time_stamp[0] + "_" + _time_stamp[1] + "_" + _time_stamp[2] + "_" + _time_stamp[3] + "_" + _time_stamp[4];  // csv file name
		
		
	    auto init_log_time = high_resolution_clock::now(); // Reference time4
	    
	    logPos.push_back({ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 }); 
		logVel.push_back({ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 }); 
		
		bool gripper_flag = true;
		gripper.move(0.075, 0.1);
		
		while(running)         // while true
		{
			// Make thread sleep for specific amount of time to control the cycle rate
			std::this_thread::sleep_for(
			std::chrono::milliseconds(static_cast<int>((1.0 / log_rate * 1000.0))));
			
			//Try to lock  data to avoid read write collisions
			if(log_parameters.mutex.try_lock())
			{
				if(log_parameters.has_data)
				{
					//Create standard arrays in which to copy the follower robot's joint parameters
					std::array<double, 8> polledFollowerJointPosition;
					std::array<double, 8> polledFollowerJointVelocity;
					
					for(int i = 0; i<7; i++)
					{
						polledFollowerJointPosition[i+1] = log_parameters.robot_state.q[i];
						polledFollowerJointVelocity[i+1] = log_parameters.robot_state.dq[i];	
					}
					
					auto log_time = high_resolution_clock::now();
					duration<double> elapsed_time = (log_time - init_log_time);
					
					polledFollowerJointPosition[0] = elapsed_time.count();
					polledFollowerJointVelocity[0] = elapsed_time.count();

					logPos.push_back(polledFollowerJointPosition);
					logVel.push_back(polledFollowerJointVelocity);
					
					log_parameters.has_data = false;
				}//has_data	
				log_parameters.mutex.unlock();
			}//mutex
			initscr();
			noecho();
			cbreak();
			//keypad(stdscr, TRUE);
			nodelay(stdscr, TRUE);
			scrollok(stdscr, TRUE);
			int keypad_command = getch();
			endwin();			
			if(keypad_command == -1)
			{
						
			}
			else if(keypad_command == 53)
			{
				if(gripper_flag)
				{
					gripper.grasp(0.01, 0.1, 60, 0.04, 0.04);
					gripper_flag = false; 
				}
				else
				{
					gripper.move(0.075, 0.1);
					gripper_flag = true; 
				}
			}
			else if(keypad_command == 48)
			{
				writeToCsv write_position("csv/followerPosition_TCPIP_wired_" + times_tamp + ".csv");
				writeToCsv write_velocity("csv/followerVelocity_TCPIP_wired_"  + times_tamp + ".csv");
				
				writeToCsv write_sendAndReceiveTimes("csv/sendAndReceiveTimes_TCPIP_wired_"  + times_tamp + ".csv");
				
				int rows_pos = logPos.size();
				int rows_vel = logVel.size();
				int rowsTime = sendAndReceiveTimeLog.size();
				
				for(int i=0; i < rows_pos; i++ )
				{
					write_position.addDatainRow(logPos[i].begin(), logPos[i].end());
				}
				for(int i=0; i < rows_vel; i++ )
				{
					write_velocity.addDatainRow(logVel[i].begin(), logVel[i].end());
				}
				for(int i=0; i < rowsTime; i++ )
				{
					write_sendAndReceiveTimes.addDatainRow(sendAndReceiveTimeLog[i].begin(), sendAndReceiveTimeLog[i].end());
				}
				exit (-1);
			}
			else
			{
				printf("Key pressed:  %d  \n", keypad_command);				
			}

			// ADD ANOTHER TIME TO LOG
			//log_time  = logtime. pushback() auto loop = high_resolution_clock::now();

		}// End of while loop
		
	}); // End of thread log
	
	/* Now we are going to connect and move the robot */

	try
	{
		//Connect to robot
		franka::Robot robot(argv[1]);                                   // All operations on the arm are performed through franka::Robot object. robot is a type of franka::Robot
		setDefaultBehavior(robot);

		// First move the robot to a suitable joint configuration
		std::array<double, 7> q_goal = {{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}};
		MotionGenerator motion_generator(0.1, q_goal);
		printf("WARNING: This example will move the robot! \n");
		printf("Please make sure to have the user stop button at hand!\n");
		printf("Robot Will Move In 5 seconds... \n");
		
		robot.control(motion_generator);
		
		printf("Finished moving to initial joint configuration.\n");

		// Set additional parameters always before the control loop, NEVER in the control loop!
		// Set collision behavior.
		robot.setCollisionBehavior(
			{{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
			{{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
			{{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}},
			{{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}});

		///////////////////////////////////////////////////// Create a socket and request cponnection with leader robot   ////////////////////////////////
		if ((sock = socket(AF_INET, SOCK_STREAM, 0)) < 0) 
		{ 
			printf("\n Socket creation error \n"); 
			return -1; 
		}
		else
		{
			printf("\n Socket created \n"); 
		} 

		serv_addr.sin_family = AF_INET; 
		serv_addr.sin_port = htons(PORT); 
		
		// Convert IPv4 and IPv6 addresses from text to binary form 
		if(inet_pton(AF_INET, "192.168.10.1", &serv_addr.sin_addr)<=0)                        //Wired
		{ 
			printf("\nInvalid address/ Address not supported \n"); 
			return -1; 
		} 

		if (connect(sock, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) < 0) 
		{ 
			printf("\n Ooops, Connection Failed \n"); 
			return -1; 
		} 
		else
		{
			printf("\n Connection Established \n"); 
		}
		///////////////////////////////////////////////////// /////////////////////////////////////////////////////////////////////////////////////////////
		
										// Set gains for the joint impedance control.
		// Stiffness
		const std::array<double, 7> k_gains = {{80.0, 80.0, 80.0, 80.0, 50.0, 20.0, 20.0}};
		// Damping
		const std::array<double, 7> d_gains = {{50.0, 50.0, 50.0, 50.0, 10.0, 10.0, 5.0}};
		
		// Define callback for the joint torque control loop.
		std::function<franka::Torques(const franka::RobotState&, franka::Duration)>
        impedance_control_callback = 
            [&joint_parameters, &log_parameters, k_gains, d_gains](const franka::RobotState& state, franka::Duration /*period*/) -> franka::Torques 
                {
					std::array<double, 14> receivedLeaderJointPositionAndVelocity;  
					std::array<double, 7> receivedLeaderJointPosition {{0.0,0.0,0.0,0.0,0.0,0.0,0.0}}; 
					std::array<double, 7> receivedLeaderJointVelocity {{0.0,0.0,0.0,0.0,0.0,0.0,0.0}}; 
					std::array<double, 7> tau_d_calculated {{0.0,0.0,0.0,0.0,0.0,0.0,0.0}};	
					
					//Update data to the thread and use data received from the leader robot
					if(joint_parameters.mutex.try_lock())    //Lock when updating so that no other thread can access the parameters whilst updating. effected in the thread
					{
						joint_parameters.has_data = true;
						joint_parameters.robot_state = state;
						receivedLeaderJointPositionAndVelocity = joint_parameters.leaderJointPositionAndVelocity;
						receivedLeaderJointPosition = joint_parameters.leaderJointPosition;
						receivedLeaderJointVelocity = joint_parameters.leaderJointVelocity;
						joint_parameters.mutex.unlock();	// unlock when update completes
					}
					
					if(log_parameters.mutex.try_lock())    //Lock when updating so that no other thread can access the parameters whilst updating. effected in the thread
					{
						log_parameters.has_data = true;
						log_parameters.robot_state = state;
						log_parameters.mutex.unlock();	// unlock when update completes
					}
					// Compute torque command from joint impedance control law.
					// Note: The answer to our Cartesian pose inverse kinematics is always in state.q_d with one
					// time step delay.
					
					if(receivedLeaderJointPosition[6] != 0)   // Just ensure that random zero values are not used in calculations 
					{
						for (size_t i = 0; i < 7; i++) 
						{
							tau_d_calculated[i] = (k_gains[i] * (receivedLeaderJointPosition[i] - state.q[i]) + d_gains[i]  * (receivedLeaderJointVelocity[i]-state.dq[i]));
						}
					}	
					else // failsafe torque command
					{	
						tau_d_calculated = {{0.0,0.0,0.0,0.0,0.0,0.0,0.0}};
					}
					
					//Send torque command
					return tau_d_calculated;		
				};
				
		// Start real-time control loop.
		robot.control(impedance_control_callback);
	}
	catch(const franka::Exception& ex)
	{
		running = false;
		//std::cerr << ex.what() << std::endl;
	}
	if(pollFollowerParametersAndCommunicateWithLeader.joinable())
	{
		pollFollowerParametersAndCommunicateWithLeader.join();
	}
	if(log.joinable())
	{
		log.join();
	}
	
	return 0;
}
