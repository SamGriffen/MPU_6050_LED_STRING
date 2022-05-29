#include "leds.h"

// Intensity of each LED - Used for pulsing effect
int LED_intensity[LED_COUNT];

CRGB leds[LED_COUNT];


void LED_begin(){
    FastLED.addLeds<WS2811, DATA_PIN>(leds, LED_COUNT);
}

void LED_test(){
    for(int i = 0; i < LED_COUNT; i++){
        leds[i].setRGB((255-LED_COUNT)+i, 0, 0);
        FastLED.show();     
    }
    delay(500);
    for(int i = LED_COUNT-1; i >=0; i--){
        leds[i].setRGB(0, 0, 0);
        FastLED.show();     
    }
    delay(500);
}