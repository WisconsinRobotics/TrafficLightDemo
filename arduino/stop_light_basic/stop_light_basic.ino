/** 
 * stop_light_basic.ino
 * Powers a traffic light
 * Mapping:
 * a -> RED_LED_PIN, slow
 * b -> RED_LED_PIN, fast
 * c -> YELLOW_LED_PIN, slow
 * d -> YELLOW_LED_PIN, fast
 * e -> GREEN_LED_PIN, slow
 * f -> rapidly cycle through all
 * g -> turn off
 * Author: Zach Waltz
 * Revised: William Jen
 */

// milliseconds //
#define SLOW_FLASH_INTERVAL     700
#define FAST_FLASH_INTERVAL     350
#define PARTY_FLASH_INTERVAL    100

#define RED_LED_PIN     8
#define YELLOW_LED_PIN  9
#define GREEN_LED_PIN   10

// note: any invalid character turns the traffic light off
char state = 'h';

void setup()
{
  Serial.begin(9600);
  pinMode(RED_LED_PIN, OUTPUT); 
  pinMode(YELLOW_LED_PIN, OUTPUT); 
  pinMode(GREEN_LED_PIN, OUTPUT);

  // POST routine
  // turn all on, then off
  digitalWrite(RED_LED_PIN, LOW); 
  digitalWrite(YELLOW_LED_PIN, LOW); 
  digitalWrite(GREEN_LED_PIN, LOW);
  delay(1000); 
  digitalWrite(RED_LED_PIN, HIGH); 
  digitalWrite(YELLOW_LED_PIN, HIGH); 
  digitalWrite(GREEN_LED_PIN, HIGH);
}

void loop()
{
    if (Serial.available())
    {
        // SET NEXT STATE
        // READ SERIAL MESSAGES
        state = Serial.read();
    
        // CONSUME THE REST OF BUFFER
        while (Serial.available()) 
            Serial.read(); 
    }
    
    // select action based on current state
    switch (state)
    {
        case 'a':
            slow_red();
            break;
        case 'b':
            fast_red();
            break;
        case 'c':
            slow_yellow();
            break;
        case 'd':
            fast_yellow();
            break;
        case 'e':
            slow_green();
            break;
        case 'f':
            fast_green();
            break;
        case 'g':
            party_mode();
            break;
        default:
            break;
    }
}

void toggle_leds(bool red_on, bool yellow_on, bool green_on, int interval)
{
    digitalWrite(RED_LED_PIN, red_on ? LOW : HIGH );
    digitalWrite(YELLOW_LED_PIN, yellow_on ? LOW : HIGH);
    digitalWrite(GREEN_LED_PIN, green_on ? LOW : HIGH);
    delay(interval);
}

void slow_green()
{
    toggle_leds(false, false, true, SLOW_FLASH_INTERVAL);
    toggle_leds(false, false, false, SLOW_FLASH_INTERVAL);
}

void fast_green()
{
    toggle_leds(false, false, true, FAST_FLASH_INTERVAL);
    toggle_leds(false, false, false, FAST_FLASH_INTERVAL);
}

void slow_yellow()
{
    toggle_leds(false, true, false, SLOW_FLASH_INTERVAL);
    toggle_leds(false, false, false, SLOW_FLASH_INTERVAL);
}

void fast_yellow()
{
    toggle_leds(false, true, false, FAST_FLASH_INTERVAL);
    toggle_leds(false, false, false, FAST_FLASH_INTERVAL); 
}

void slow_red()
{
    toggle_leds(true, false, false, SLOW_FLASH_INTERVAL);
    toggle_leds(false, false, false, SLOW_FLASH_INTERVAL);   
}

void fast_red()
{
    toggle_leds(true, false, false, FAST_FLASH_INTERVAL);
    toggle_leds(false, false, false, FAST_FLASH_INTERVAL); 
}

void party_mode()
{
    toggle_leds(true, true, false, PARTY_FLASH_INTERVAL);
    toggle_leds(true, false, true, PARTY_FLASH_INTERVAL);
    toggle_leds(false, true, false, PARTY_FLASH_INTERVAL);
    toggle_leds(false, true, true, PARTY_FLASH_INTERVAL);
    toggle_leds(true, true, true, PARTY_FLASH_INTERVAL);
}

