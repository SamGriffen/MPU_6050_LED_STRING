/**
 * @file leds.h
 * @author Sam Griffen
 * @brief Functions for controlling the LEDs
 * @version 0.1
 * @date 2022-05-27
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#ifndef LED_H
#define LED_H

#include "Arduino.h"
#include "FastLED.h"

// ================================================================
// ===                       RGB CONTROL                        ===
// ================================================================

#define MAX 60
#define MIN -60

// Number of LEDs present on the strip
#define LED_COUNT 50

// Pin connected to RGB led string
#define DATA_PIN 10

#define RANDOM_MAX 10

#define ON_DELAY_RAND 10

// Method to set up LED string
void LED_begin();

void LED_test();

void update_string(int step);

// Method to step the value of a given LED. Will step based on the maximum step value 
void tick_LED(int ind, int max_step);

#endif

// // Superseded helper methods

// // https://gist.github.com/postspectacular/2a4a8db092011c6743a7