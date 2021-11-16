#include <ros/ros.h>
#include <std_msgs/UInt8.h>
#include <sensor_msgs/LaserScan.h>

#include <unistd.h>
#include <string.h>

#include <queue>
#include <vector>

#define TOPIC_BUFFER_SIZE 100
#define FULL_SCAN_TOLERANCE 50

std::queue<std::vector<uint8_t>> buffer;

uint8_t last_byte, counter;
bool frame_started;
uint16_t no_bytes;

float distance_array[620];

int seq = 0;

float max_distance = 8.0f;
float min_distance = 0.2f; 

float max_angle = 360;
float min_angle = 0;

int start_stop_angle = -1;

std::string frame_id = "lase_frame";

ros::Publisher pub;

sensor_msgs::LaserScan laser_scan;

void print_front()
{
	for(int i=0;i<buffer.front().size();i++)
		printf("%x ",buffer.front()[i]);
	printf("\n");
}

float angle_to_rad(float angle)
{
	return (angle*3.1415)/180;
}

void initial_scan_setup(sensor_msgs::LaserScan *laser_scan)
{
	//laser_scan->header.seq = 0;
	laser_scan->header.stamp = ros::Time::now();
	laser_scan->header.frame_id = frame_id;

	laser_scan->angle_min = angle_to_rad(min_angle);
	laser_scan->angle_max = angle_to_rad(max_angle);

	laser_scan->range_min = min_distance;
	laser_scan->range_max = max_distance;
}

bool process_frame(std::vector<uint8_t> buffer)
{
	//ROS_INFO("processing");
	uint16_t diatance_byte, start_angle_byte, stop_angle_byte;
	uint8_t no_sample_points;
	float start_angle, end_angle, diff_angle, angle_i, distance, angle_cor;
	int index = 0, start_index, end_index, counter = 2;

	if(buffer[2] != 1)
	{
		no_sample_points = buffer[3];
		//printf("points: %d\n",no_sample_points);

		if(no_sample_points > 10)
		{
			start_angle_byte = buffer[5] << 8 | buffer[4];
			stop_angle_byte = buffer[7] << 8 | buffer[6];

			start_angle = (float)(start_angle_byte >> 1) / 64;
			end_angle = (float)(stop_angle_byte >> 1) / 64;

			if(start_angle > end_angle)
				diff_angle = 360 - start_angle + end_angle;
			else
				diff_angle = end_angle - start_angle;

			start_index = start_angle / (diff_angle / no_sample_points);
			end_index = end_angle / (diff_angle / no_sample_points);

			//printf("%.4f\n", diff_angle / no_sample_points);

			if(start_stop_angle == -1)
				start_stop_angle = start_angle;

			//printf("%.4f %.4f %.4f\n", start_angle, end_angle, diff_angle);
			//printf("%d %d %d\n", start_index, end_index, start_stop_angle);

			if(abs(0.615 - diff_angle / no_sample_points) <= 0.01)
			{

				for(int i=1; i<no_sample_points*2-2; i+=2)
				{
					angle_i = (diff_angle / (no_sample_points - 1)) * (counter - 1) + start_angle;

					diatance_byte = buffer[10+i] << 8 | buffer[10+i-1];
					distance = ((float)diatance_byte) / 4 / 1000;

					if(distance == 0)
						angle_cor = 0;
					else
						angle_cor = atan(21.8*((155.3-distance*1000)/(155.3+distance*1000)));
					angle_i = angle_i + angle_cor;

					angle_i = 720 - angle_i;

					if(angle_i > 360)
						angle_i -= 360;

					index = angle_i / (diff_angle / no_sample_points);
					//printf("%.4f %d\n", angle_i, index);

					if(distance < 0 || angle_i < min_angle || angle_i > max_angle || distance < min_distance || distance > max_distance)
					{
						distance = 0;
						angle_i = 0;
					}

					distance_array[index] = distance;

					counter++;
				}

			}
			//else
				//ROS_INFO("angle not good");
		}
		//else
			//ROS_INFO("no points");
	}
	else
		//if(abs(start_stop_angle - start_angle) < FULL_SCAN_TOLERANCE)
		//if(start_stop_index > start_index && start_stop_index <= end_index)
		{
			//ROS_INFO("PUBLISH");

			initial_scan_setup(&laser_scan);

			laser_scan.header.stamp = ros::Time::now();
			laser_scan.angle_increment = angle_to_rad(0.625);
			laser_scan.time_increment = 0;
			laser_scan.scan_time = 0;
			laser_scan.ranges = std::vector<float>(distance_array, distance_array + sizeof(distance_array)/sizeof(distance_array[0]));
			laser_scan.header.seq = seq;
			seq++;

			pub.publish(laser_scan);

			//for(int i=0;i<600;i++)
				//distance_array[i] = 0;

			//if(ioctl(port, TCFLSH, 0) < 0)
				//ROS_INFO("error at reseting");
		}

		//ROS_INFO("new scan");

	return false;
}

void process_byte(uint8_t byte)
{
	//printf("dad\n");
	if(frame_started && counter <= no_bytes)
	{
		//ROS_INFO("Test for size.");
		switch(counter)
		{
			case 1:
			{
				//printf("byte: %x\n", byte);
				no_bytes += byte*2+1;
				//printf("no. bytes: %d\n", no_bytes);
				if(no_bytes <= 90)
					buffer.back().push_back(byte);
				else
					frame_started = false;
			} break;
			default: buffer.back().push_back(byte);
		}

		if(counter == no_bytes-1)
		{
			//print_front();
			//ROS_INFO("Frame ended.");
			frame_started = false;
		}

		counter++;
		return;
	}

	if(buffer.size() > 1)
	{
		process_frame(buffer.front());
		//ROS_INFO("Pop.");
		buffer.pop();
	}

	if((byte << 8 | last_byte) == 0x55aa)
	{
		//ROS_INFO("New Frame.");
		frame_started = true;
		counter = 0;
		no_bytes = 7;
		buffer.push(std::vector<uint8_t>());
		buffer.back().push_back(last_byte);
		buffer.back().push_back(byte);
		return;
	}
	else
		last_byte = byte;
}

void recevied_bytes(const std_msgs::UInt8 msg)
{
	//printf("Process: %x\n",msg.data);
	process_byte(msg.data);
	ros::spinOnce();
}

int main(int argc, char** argv)
{
	last_byte = 0;
	counter = 0;
	frame_started = false;
	no_bytes = 12;

   	ros::init(argc,argv,"process_node");
   	ros::NodeHandle nh;

	std::string read_topic_name = "/bytes";
	std::string scan_topic_name = "/scan_laser";

	nh.getParam("raw_topic", read_topic_name);
	nh.getParam("scan_topic", scan_topic_name);

	nh.getParam("max_angle", max_angle);
	nh.getParam("min_angle", min_angle);

	nh.getParam("max_distance", max_distance);
	nh.getParam("min_distance", min_distance);

	nh.getParam("frame_id", frame_id);

	ros::Subscriber sub = nh.subscribe(read_topic_name, TOPIC_BUFFER_SIZE, recevied_bytes);

	pub = nh.advertise<sensor_msgs::LaserScan>(scan_topic_name, TOPIC_BUFFER_SIZE);

	ROS_INFO("Waiting to process.");
	ros::Duration(5).sleep();
	ROS_INFO("Starting to process.");

	ros::spin();

   	return 0;
}
