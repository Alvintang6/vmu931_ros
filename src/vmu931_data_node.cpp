#include "vmu931.h"

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <stdio.h> //printf
#include <stdlib.h> //exit

void usage(char **argv);


const int DATA_SIZE=10; 
//const int MAX_READS=1000;


int main(int argc, char **argv)
{     
	ros::init(argc, argv, "vmu931_data"); 
	ros::NodeHandle nh;

	ros::Publisher pub = nh.advertise<sensor_msgs::Imu>("/imu/data",1000);


	struct vmu *vmu=NULL;
	// suppose we are intersted in euler, magnetometer and accelerometer
	// note that quaternions and heading use different data types
	//struct vmu_txyz euler_data[DATA_SIZE];
	struct vmu_twxyz quat_data[DATA_SIZE];
	struct vmu_txyz accel_data[DATA_SIZE];
	struct vmu_txyz gyro_data[DATA_SIZE];

	struct vmu_data data={0}; //the library will return data here and note number of read values in data.size
	struct vmu_size size={0}; //this notes sizes of our arrays, data.size is refreshed with this value before read

	size.accel = size.quat = size.gyro = DATA_SIZE;	

	data.quat = quat_data;
	data.gyro = gyro_data;
	data.accel = accel_data;
	data.size = size;
	
	std::string tty_port;
	ros::param::get("~port",tty_port);
	
	const char *tty_device=tty_port.c_str();

	int ret, reads=0;
	
	
	
	if( (vmu=vmu_init(tty_device)) == NULL)
	{
		perror(	"unable to initialize VMU931\n\n"
				"hints:\n"
				"- it takes a few seconds after plugging in to initialize device\n"
				"- make sure you are using correct tty device (dmesg after plugging vmu)\n"
				"- if all else fails unplug/plug VMU931\n\n"
				"error details");
		return 1;
	}	
	
	if( vmu_stream(vmu, VMU_STREAM_QUAT |VMU_STREAM_GYRO | VMU_STREAM_ACCEL) == VMU_ERROR )
	{
		perror("failed to stream euler/mag/accel data");
		exit(EXIT_FAILURE);
}
		





        ros::Rate rate(200);
        
	while( ((ret=vmu_read_all(vmu, &data)) != VMU_ERROR))
	{
		
	    sensor_msgs::Imu vmu_data;
    		vmu_data.header.stamp = ros::Time::now();
    		vmu_data.header.frame_id = "imu_link";

	for(int i=0;i<data.size.gyro;++i){
    //should add calibration bias to  received angular velocity and pass to to corrected IMU data object
		vmu_data.angular_velocity.x = data.gyro[i].x;
  		vmu_data.angular_velocity.y = data.gyro[i].y;
  		vmu_data.angular_velocity.z = data.gyro[i].z;
	}
	
	for(int i=0;i<data.size.quat;++i){
		vmu_data.orientation.x = data.quat[i].x;
		vmu_data.orientation.y = data.quat[i].y;	
		vmu_data.orientation.z = data.quat[i].z;
		vmu_data.orientation.w = data.quat[i].w;
	}


    //pass calibrated acceleration to corrected IMU data object
	for(int i=0;i<data.size.accel;++i){
		vmu_data.linear_acceleration.x = data.accel[i].x;
	        vmu_data.linear_acceleration.y = data.accel[i].y;
		vmu_data.linear_acceleration.z = data.accel[i].z;
	}
		
		
		data.size = size;	
		//refresh the sizes of the arrays for data streams
  
  	pub.publish(vmu_data);
  	rate.sleep();
	



	//terminate after reading MAX_READS times
		//remove those lines if you want to read infinitely
		//if(++reads >= MAX_READS) 
	
		//break;
				
	}

		
	if(ret == VMU_ERROR)
		perror("failed to read from VMU931");
	else
		printf("success reading from VMU931, bye...\n");
	
	vmu_close(vmu);
	
	return 0;
}











void usage(char **argv)
{
	printf("Usage:\n");
	printf("%s tty_device\n\n", argv[0]);
	printf("examples:\n");
	printf("%s /dev/ttyACM0\n", argv[0]);
}
