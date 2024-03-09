#define BLINK_CYCLE 200 // Length of blink cycle (ms)
#define BLINK_ON    50  // Duration of ON time (ms)

static void warning_blink(){
  // Blink the neopixel for warning
  if(warning){
    if(((millis()-warning_timer)%BLINK_CYCLE)<BLINK_ON){
      // On for BLINK_ON(ms)
      pixels.setPixelColor(0, pixels.Color(150, 0, 150, 150));
    }
    else{
      // Off for reset
      pixels.clear();
      warning = BLINK_COUNT-int((millis()-warning_timer)/BLINK_CYCLE)-1;
    }
  }
  pixels.show();
}
