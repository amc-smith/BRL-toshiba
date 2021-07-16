// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE

#include <array>
#include <atomic>
#include <cmath>
#include <functional>
#include <iostream>
#include <iterator>
#include <mutex>
#include <thread>
#include <franka/exception.h>
#include <franka/model.h>
#include <franka/rate_limiting.h>
#include <franka/robot.h>

#include "examples_common.h"

#include <sys/socket.h> 
#include <franka/duration.h>
#include <limits>
#include <netinet/in.h> 
#include <string.h> 
#include <stdlib.h> 
#include <stdio.h> 
#include <unistd.h> 
#include <iostream>
#include <Eigen/Core>

#include <fstream>
#include <iterator>
#include <algorithm>

#include "signal.h"    // for catching exit signals
#include "stdlib.h"
#include "ctype.h"
#include "sys/time.h"

#include <assert.h>
#include <franka/gripper.h>

#include <vector>
#include <string>
#include <sstream>

#define PORT 8080                          //Define TCPIP port
#define N 8

using namespace std;

using std::chrono::high_resolution_clock;
using std::chrono::duration_cast;
using std::chrono::duration;
using std::chrono::milliseconds;


//////////////////////////////////////////////////  Declare variables for the TCPIP socket   ///////////////////////////////////////////////////////////
int server_fd, new_socket, valread; 
struct sockaddr_in address; 
int opt = 1; 
int addrlen = sizeof(address); 
double buffer[1024] = {0};

/* 
 * A class to create and write data in a csv file.
 * */
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


////////////////////////////////////////////////   Get Day, Date and Time independent of of the system day, date and time   ///////////////////////////////////
std::vector<std::string> timestamp()
{
	/*
	 * Declare argument for time
	 */
	time_t tt;
	
	/*
	 * Declare a variable to store return value of localtime(). It is independent of system time
	 */
	struct tm * _ti;
	
	//Apply time()
	time (&tt);
	
	//Using localtime()
	_ti = localtime(&tt);
	string timeee = asctime(_ti);
	
	// split string timeee into time day and year
	string buf;                                           // buffer string
	std::stringstream ss(timeee);                         // Insert the string into a stream
	
	std::vector<std::string> _bits;                       //Create a vector called _bits to hold split words
	/* 
	 * store each split word in the vector 
	 */
	while (ss >> buf)
		_bits.push_back(buf);
	
	return _bits;
}
////////////////////////////////////////////////   Get Day, Date and Time independent of of the system day, date and time   ///////////////////////////////////


int main(int argc, char** argv) 
{
	vector<array<double, N> > logPos;                         // Create a vector of arrays to store leader robot joint positions 
	vector<array<double, N> > logVel;                         // Create a vector of arrays to store leader robot joint velocities
	
	vector<array<double, 3> > sendAndReceiveTimeLog;
	sendAndReceiveTimeLog.push_back({ 0.0, 0.0, 0.0});
	
	string time_stamp;                                        
		
	/* 
	 * Check whether the required arguments were passed.
	 */
		if (argc != 2) 
		{
			std::cerr << "Usage: " << argv[0] << " <robot-hostname>" << std::endl;
			return -1;
		}
		
		// Set the comms and log rates (Hz)
		const double comms_rate = 20.0;
		const double log_rate = 20.0;
	
	
	//////////////////////////////////////////   Initialize data fields for different threads.  ///////////////////////////////////////////////////////////
	struct 
	{
		std::mutex mutex;
		bool has_data;
		std::array<double, 7> receivedFollowerExternalJointTorque;
		franka::RobotState robot_state;
	}joint_parameters{};
	
	struct
	{
		std::mutex mutex;
		bool has_data;
		bool log;
		franka::RobotState robot_state;
		franka::GripperState gripper_state;
	}log_parameters{};
	
	std::atomic_bool running{true};  
	//////////////////////////////////////////   Initialize data fields for different threads.  ///////////////////////////////////////////////////////////
	
	
	std::thread pollLeaderParametersAndCommunicateWithFollower([comms_rate, &running, &logPos, &logVel, &sendAndReceiveTimeLog, &joint_parameters, &log_parameters, &time_stamp]() 
		{
			auto loop_time = high_resolution_clock::now();
			while (running) 
			{
				// Sleep to achieve the desired comms rate.
				std::this_thread::sleep_for(
				std::chrono::milliseconds(static_cast<int>((1.0 / comms_rate * 1000.0))));
				std::array<double, 3> sendAndReceiveTime;
				// Try to lock data to avoid read write collisions.
				if (joint_parameters.mutex.try_lock()) 
				{	
					if(joint_parameters.has_data)
					{
						//Create standard arrays in which to copy the leader joint position and velocity
						std::array<double, 7> leaderJointPosition;
						std::array<double, 7> leaderJointVelocity;
						std::array<double, 7> leaderJointTorque;
						
			
						for (size_t i = 0; i < 7; i++) 
						{
							leaderJointPosition[i] = joint_parameters.robot_state.q[i];
							leaderJointVelocity[i] = joint_parameters.robot_state.dq[i];
							leaderJointTorque[i] = joint_parameters.robot_state.tau_J[i];
						}
						
						//???????????????????????????????????????????????   Combine the leader robot's joints' positions and velocities into a single array
						int noOfElements = sizeof(leaderJointPosition)/sizeof(leaderJointPosition[0]);
						std::array<double, 14> leaderJointPositionAndVelocity;
						for(int i= 0; i < 14; i++)
						{
							if(i < noOfElements)
							{
								leaderJointPositionAndVelocity[i] = leaderJointPosition[i];
							}
							else
							{
								leaderJointPositionAndVelocity[i] = leaderJointVelocity[i - noOfElements];
							}
						}
						
						/////////////////////////////////////////    Read incoming messages and save in an array  ///////////////////////////////////////////////////////
						auto s_r_t1 = high_resolution_clock::now();                                                                                                        //Timer
						bzero(buffer, 1024);
						valread = read( new_socket , buffer, 1024); 
						if(valread != 0)
						{
							for (int i=0; i < 7; i++)
							{
								joint_parameters.receivedFollowerExternalJointTorque[i] = buffer[i];
							}				
							//std::cout << "Received Follower Robot External Joint Torques: ";
							for (int i = 0; i < 7; i++)
							{ 
								std::cout << joint_parameters.receivedFollowerExternalJointTorque[i];
							}
							std::fill_n(buffer,7,0);	                     // Clear the buffer array. This ensures that no values are printed out if nothing is received
						}
						else
						{
							running = false;
						}	
						/////////////////////////////////////////    Read incoming messages and save in an array  ///////////////////////////////////////////////////////
						auto s_r_t2 = high_resolution_clock::now();	                                                                                                      //Timer
						
						////////////////////////////////////////////   Send leader robot joint position and velocities to the follower robot   ///////////////////     
						send(new_socket, (char *) &leaderJointPositionAndVelocity, sizeof(leaderJointPositionAndVelocity), 0);
						auto s_r_t3 = high_resolution_clock::now();                                                                                                       //Timer
						////////////////////////////////////////////   Send leader robot joint position and velocities to the follower robot   ///////////////////
						
						joint_parameters.has_data = false;
						
						duration<double, std::milli>  t_receive = s_r_t2 - s_r_t1;
						duration<double, std::milli>  t_send = s_r_t3 - s_r_t2;
						std::cout << "Send: "<<t_send.count() << "ms" << std::endl;
						std::cout << "Receive: "<<t_receive.count() << "ms" << std::endl;
						
						//std::array<double, 3> sendAndReceiveTime;
						sendAndReceiveTime[0] = t_send.count();
						sendAndReceiveTime[1] = t_receive.count();					
						//sendAndReceiveTimeLog.push_back(sendAndReceiveTime);
						
						duration<double, std::milli> t_loop = high_resolution_clock::now() - loop_time;
						loop_time =  high_resolution_clock::now();
						
						sendAndReceiveTime[2] =  t_loop.count();	
						sendAndReceiveTimeLog.push_back(sendAndReceiveTime);
						
						printf("Loop: %f ms  \n", t_loop.count());
						
						
						joint_parameters.has_data = false;
					}//Has data
					joint_parameters.mutex.unlock();
				}//mutex.try_lock
				
			}//while				
					
			writeToCsv write_position("csv/leaderPosition_TCPIP_wired_" + time_stamp + ".csv");
			writeToCsv write_velocity("csv/leaderVelocity_TCPIP_wired_" + time_stamp + ".csv");
			writeToCsv write_sendAndReceiveTimes("csv/sendAndReceiveTimes_TCPIP_wired_" + time_stamp + ".csv");
			
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

		});  // End of thread pollLeaderJointPositionAndVelocity (Lambda function)
	

//THREAD FOR Logging robot parameters
	std::thread log([log_rate, &running, &log_parameters, &logPos, &logVel, &time_stamp]()
	{
		//TIME KEEPING
		std::vector<std::string> _time_stamp = timestamp();
		time_stamp = _time_stamp[0] + "_" + _time_stamp[1] + "_" + _time_stamp[2] + "_" + _time_stamp[3] + "_" + _time_stamp[4];  // csv file name
		
		auto init_log_time = high_resolution_clock::now();   // Reference time
		
		logPos.push_back({ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 });
		logVel.push_back({ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 });
		
		while(running)         // while true
		{
			// Make thread sleep for specific amount of time to control the cycle rate
			std::this_thread::sleep_for(
			std::chrono::milliseconds(static_cast<int>((1.0 / log_rate * 1000.0))));
		
			//Try to lock  data to avoid read write collisions
			if(log_parameters.mutex.try_lock())
			{
				if(log_parameters.has_data) // Set true in the control loop
				{
					//Create standard arrays in which to copy the follower robot's joint parameters
					std::array<double, 8> polledLeaderJointPosition;         // First element contains time, remaining 7 is for joint positions
					std::array<double, 8> polledLeaderJointVelocity;
						
					for(int i = 0; i<7; i++)
					{
						polledLeaderJointPosition[i+1] = log_parameters.robot_state.q[i];
						polledLeaderJointVelocity[i+1] = log_parameters.robot_state.dq[i];	
					}
					
					auto log_time = high_resolution_clock::now();
					duration<double> elasped_time = (log_time - init_log_time);

					polledLeaderJointPosition[0] = elasped_time.count();  // time in seconds is the first element of the polledFollowerJointPosition array
					polledLeaderJointVelocity[0] = elasped_time.count();
					
					logPos.push_back(polledLeaderJointPosition);
					logVel.push_back(polledLeaderJointVelocity);
						
					log_parameters.has_data = false;			
							
				} //has data
					log_parameters.mutex.unlock();
			}// Mutex lock

		}// End of while loop
			
	}); // End of thread log	
		
	/* Now we will connect to and move the robot */
		
	try 
	{
		// Connect to robot.
		franka::Robot robot(argv[1]);
		setDefaultBehavior(robot);

		// First move the robot to a suitable initial joint configuration
		std::array<double, 7> q_goal = {{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}};                   // M_PI_4 == pi/4, pi radian is 180 degrees
		MotionGenerator motion_generator(0.1, q_goal);                                                      // 0.5 is the speed factor. It ranges from 0 to 1
		printf("WARNING: This example will move the robot! \n");
		printf("Please make sure to have the user stop button at hand!\n");
		printf("Robot Will Move In 5 seconds... \n");                                                                        
		
		robot.control(motion_generator);                                                                    // initiate the motion generator
		printf("Finished moving to initial joint configuration.\n");

		// Set additional parameters always before the control loop, NEVER in the control loop!
		// Set collision behavior.
		robot.setCollisionBehavior(
			{{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},                       // lower_torque_thresholds_during_acceleration (Nm)   ,   upper_torque_thresholds__during_acceleration (Nm)   For each joint
			{{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},                       // lower_torque_thresholds_nominal (Nm)    ,   upper_torque_thresholds_nominal (Nm)         For each joint               
			{{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}},                                   // lower_force_thresholds_during_acceleration  (N) ,   upper_force_thresholds__during_acceleration (N)       (x,y,z,R,P,Y)
			{{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}});                                  // lower_force_thresholds_nominal (N)   ,   upper_force_thresholds_nominal (N)
	 

		////////////////////////////// Creating socket file descriptor //////////////////////////////
		if ((server_fd = socket(AF_INET, SOCK_STREAM, 0)) == 0) 
		{ 
			perror("socket failed"); 
			exit(EXIT_FAILURE); 
		} 
			
		// Socket options
		if (setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR | SO_REUSEPORT, &opt, sizeof(opt))) 
		{ 
			perror("setsockopt"); 
			exit(EXIT_FAILURE); 
		} 
		address.sin_family = AF_INET; 
		address.sin_addr.s_addr = INADDR_ANY; 
		address.sin_port = htons( PORT ); 
			
		// Forcefully attaching socket to the port 8080 
		if (bind(server_fd, (struct sockaddr *)&address, 
										sizeof(address))<0) 
		{ 
			perror("bind failed"); 
			exit(EXIT_FAILURE); 
		} 
		if (listen(server_fd, 3) < 0) 
		{ 
			perror("listen"); 
			exit(EXIT_FAILURE); 
		} 
		else{std::cout << "Waiting for Client connection" << std::endl;}
		if ((new_socket = accept(server_fd, (struct sockaddr *)&address, 
						(socklen_t*)&addrlen))<0) 
		{ 
			perror("accept"); 
			exit(EXIT_FAILURE); 
		} 
		else
		{std::cout << "connected" << std::endl;}
			
		////////////////////////////// Creating socket file descriptor //////////////////////////////

		// Define callback for the joint torque control loop.
		std::function<franka::Torques(const franka::RobotState&, franka::Duration)>
		   impedance_control_callback =
			   [&joint_parameters, &log_parameters](const franka::RobotState& state, franka::Duration /*period*/) -> franka::Torques 
			   {
					std::array<double, 7> desired_leader_tau {{0.0,0.0,0.0,0.0,0.0,0.0,0.0}};	
					std::array<double, 7> zeroTorque {{0.0,0.0,0.0,0.0,0.0,0.0,0.0}}; 
					// Update data to print.
					if (joint_parameters.mutex.try_lock()) 
					{
						joint_parameters.has_data = true;
						joint_parameters.robot_state = state;

						// Leader control = zeroTorque - (G * follower_tau_external)
						for(int i=0; i<7; i++)
						{
							desired_leader_tau[i] = zeroTorque[i] - (0.4 * (joint_parameters.receivedFollowerExternalJointTorque[i]));	
						}
					
						joint_parameters.mutex.unlock();
					}
					
					if(log_parameters.mutex.try_lock())    //Lock when updating so that no other thread can access the parameters whilst updating. effected in the thread
					{
						log_parameters.has_data = true;
						log_parameters.robot_state = state;
						log_parameters.mutex.unlock();	// unlock when update completes
					}
				  // Send torque command.
				  return desired_leader_tau;
			};
			robot.control(impedance_control_callback); // Main control loop
	} 
	  catch (const franka::Exception& ex) 
	  {
		running = false;
		std::cerr << ex.what() << std::endl;
	  }
	  if (pollLeaderParametersAndCommunicateWithFollower.joinable()) 
	  {
		pollLeaderParametersAndCommunicateWithFollower.join();
	  }
	  
	  if(log.joinable())
		{
			log.join();
		}
	  
	  return 0;
}
