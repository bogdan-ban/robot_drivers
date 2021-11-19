#include "lidar_driver/lidar_process.h"

Lidar_Process::Lidar_Process(ros::NodeHandle *nh)
{
	nh->getParam("raw_topic", read_topic_name);
	nh->getParam("scan_topic", scan_topic_name);

	nh->getParam("max_angle", max_angle);
	nh->getParam("min_angle", min_angle);

	nh->getParam("max_distance", max_distance);
	nh->getParam("min_distance", min_distance);

	nh->getParam("frame_id", frame_id);

	sub = nh->subscribe(read_topic_name, TOPIC_BUFFER_SIZE, &Lidar_Process::recevied_bytes, this);

	pub = nh->advertise<sensor_msgs::LaserScan>(scan_topic_name, TOPIC_BUFFER_SIZE);
}

float Lidar_Process::angle_to_rad(float angle)
{
	return (angle*3.1415)/180;
}

void Lidar_Process::initial_scan_setup(sensor_msgs::LaserScan *laser_scan)
{
	laser_scan->header.stamp = ros::Time::now();
	laser_scan->header.frame_id = frame_id;

	laser_scan->angle_min = angle_to_rad(min_angle);
	laser_scan->angle_max = angle_to_rad(max_angle);

	laser_scan->range_min = min_distance;
	laser_scan->range_max = max_distance;
}

void Lidar_Process::process_frame(std::vector<uint8_t> buffer)
{
	uint16_t diatance_byte, start_angle_byte, stop_angle_byte;
	uint8_t no_sample_points;
	float start_angle, end_angle, diff_angle, angle_i, distance, angle_cor;
	int index = 0, start_index, end_index, counter = 2;

	if(buffer[2] != 1)
	{
		no_sample_points = buffer[3];

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

			if(abs(0.615 - diff_angle / no_sample_points) <= 0.05)
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

					if(distance < 0 || angle_i < min_angle || angle_i > max_angle || distance < min_distance || distance > max_distance)
					{
						distance = 0;
						angle_i = 0;
					}

					distance_array[index] = distance;

					counter++;
				}

			}
		}
	}
	else
	{
		initial_scan_setup(&laser_scan);

		laser_scan.header.stamp = ros::Time::now();
		laser_scan.angle_increment = angle_to_rad(0.625);
		laser_scan.time_increment = 0;
		laser_scan.scan_time = 0;
		laser_scan.ranges = std::vector<float>(distance_array, distance_array + sizeof(distance_array)/sizeof(distance_array[0]));
		laser_scan.header.seq = seq;
		seq++;

		pub.publish(laser_scan);
	}
}

void Lidar_Process::recevied_bytes(const lidar_driver::frame msg)
{
	process_frame(msg.frame_bytes);
}
