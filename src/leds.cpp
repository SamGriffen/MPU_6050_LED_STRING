#include "leds.h"

// Intensity of each LED - Used for pulsing effect
int LED_value[LED_COUNT];

// Direction each LED is travelling - Used to create pulsing effect
// 1 for up, -1 for down
int LED_direction[LED_COUNT];


// Colour storage
int H = 0;
int S = 255;

long int last_millis = 0;

CRGB leds[LED_COUNT];

void LED_begin(){
    FastLED.addLeds<WS2811, DATA_PIN>(leds, LED_COUNT);
}

// Traverse string and update each LED
void update_string(int step){
    for(int led_ind = 0; led_ind < LED_COUNT; led_ind++){
        tick_LED(led_ind, step);
    }
    FastLED.show();

    if(millis() - last_millis > 100){
        last_millis = millis();
        H++;
        if(H > 255)H=0;
    }
}

// Update the status of an LED
void tick_LED(int led_ind, int step){
    // If the LED is off, randomly determine whether to turn it on
    if(LED_value[led_ind] == 0){
        if(random(0, RANDOM_MAX) != 0)return;

        // Turn LED on
        LED_value[led_ind] = 0;
        LED_direction[led_ind] = 1;
    }

    // Step the LED brightness
    LED_value[led_ind] += LED_direction[led_ind] * step;

    // Constrain the brightness
    if(LED_value[led_ind] < 0){
        LED_value[led_ind] = 0;
    }
    else if(LED_value[led_ind] > 255){
        LED_value[led_ind] = 255;
        if(random(0, ON_DELAY_RAND)==0)LED_direction[led_ind] = -1;
    }

    // Set up LED
    leds[led_ind].setHSV(H, S, LED_value[led_ind]);
}

void LED_test(){
    for(int i = 0; i < LED_COUNT; i++){
        leds[i].setHSV(i*4, 187, 255);
        FastLED.show();     
    }
    delay(500);
    for(int i = 0; i < LED_COUNT; i++){
        leds[i].setHSV(i*4, 187, 0);
        FastLED.show();     
    }
    delay(500);
}