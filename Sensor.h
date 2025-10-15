// Sensor.h
// Line Following Robot Starter program 
// Created by Bijan Ashouri
// February 2025
// CECS 346 SPRING 2025
// California State University, Long Beach
// 4/25/2025

#include <stdint.h>

// PE0 connect to sensor 7; PE1 connects to Sensor 0
// TODO: find the bit addresses

void Sensor_Init(void);
uint8_t Sensor_CollectData(void);
