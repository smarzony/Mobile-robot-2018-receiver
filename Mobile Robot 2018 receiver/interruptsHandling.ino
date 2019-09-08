void leftSpeedSensorInterrupt()
{
	if (now - last_velo_measure_left > VELOCITY_MEASURING_MINIMAL_PERIOD)
	{
		last_velo_measure_left = now;
		speed_left_count++;
	}

}

void rightSpeedSensorInterrupt()
{
	if (now - last_velo_measure_right > VELOCITY_MEASURING_MINIMAL_PERIOD)
	{
		last_velo_measure_right = now;
		speed_right_count++;
	}
}

void countLeftSpeed()
{
	measured_speed_left = speed_left_count;
	speed_left_count = 0;
}

void countRightSpeed()
{
	measured_speed_right = speed_right_count;
	speed_right_count = 0;
}
