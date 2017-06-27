// *************************************************************************************
// TimingLight.ino - Arduino Program for Toastmasters Automatic Timing Light
//    Author:         John Miller
//    Revision:       1.0.0
//    Date:           6/26/2017
//    Project Source: https://github.com/jpsrmiller/auto-timing-light
// 
// This Arduino program performs all functions for the Automatic Timing Light including:
//     o Selection of Time Limits (e.g. 5 to 7 min) or Manual Mode
//     o Stop-watch Timer (Start, Stop and Reset)
//     o Turning the Red/Yellow/Green Lights On and Off
//     o Manually changing the light when the Manual Button is pressed
//
// The following hardware is attached to the Arduino
//     o 16x2 Character LCD with I2C
//     o Rotary Encoder with Selector Switch
//     o Pushbutton for Manually Changing Light
//     o 4-Channel 5-Volt Relay Module
//
// *** Detailed Decription of Timing Light Operation ***
//  - LCD Displays the Selected Speech Time and the Elapsed Time.  For example:
//              ----------------
//              Time: 5 to 7 min
//                   3:45
//              ----------------
//  - User turns the Rotary Encoder Left or Right to select the Speech Time.
//    Speech time limits are defined in 'time_limits' array.  Example speech
//    times are 1-2 minutes, 2-3 minutes, 3-5 minutes, etc.  Turning Rotary
//    all the way to the Left will select Manual Model
//  - Press the Rotary Encoder Select Button to Start or Stop the Timer.
//  - Hold down the Rotary Encoder Select Button to Reset the Timer to 0:00
//  - While the Timer is running, and not in Manual Mode the Lights turn On and Off 
//    automatically at the Following Times:
//          When timer starts, all lights are Off
//          At minimum time (example 5 minutes) GREEN light turns On (Yellow/Red are Off)
//          At middle time (example 6 minutes) YELLOW light turns On (Green/Red are Off)
//          At maximum time (example 7 minutes) RED light turns On (Yellow/Green are Off)
//  - While in Manual Mode or while the timer is Stopped and the time is 0:00, the Light
//    is changed by pressing the Manual button.  Light is advanced as follows:
//           NONE --> GREEN --> YELLOW --> RED
//  - Selected time Limits are stored in EEPROM and loaded on program start-up.
//    In other words, the program "remembers" the last time limit selected
//
// *** External Libraries Required ***
// The following libraries must be downloaded and copied into the Arduino "libraries"
// folder in order for the program to compile:
//    o OneButton - https://github.com/mathertel/OneButton
//    o LiquidCrystal_I2C - https://github.com/fdebrabander/Arduino-LiquidCrystal-I2C-library
//
// *********************************************************************************

#include <Wire.h>
#include <EEPROM.h>
#include <LiquidCrystal_I2C.h>
#include <OneButton.h>

// Most I2C LCD's have an I2C Address of either 0x27 or 0x3F
// If the LCD doesn't work with one address, try the other
//#define LCD_I2C_ADDRESS 0x27
#define LCD_I2C_ADDRESS 0x3F

// Define the IO Pins Used
#define PIN_ROTARY_CLK				2		// Used for generating interrupts using CLK signal
#define PIN_ROTARY_DAT				3		// Used for reading DT signal
#define PIN_ROTARY_SW	  			4		// Used for the Rotary push button switch
#define PIN_LIGHT_GREEN				5		// Output pin for Green Light
#define PIN_LIGHT_YELLOW			6		// Output pin for Yellow Light
#define PIN_LIGHT_RED		  		7		// Output pin for Red Light
#define PIN_MANUAL_BUTTON			14		// Pushbutton to change light in Manual Mode

// Used to specify which light is currently on
#define LIGHT_NONE	  				0		// All Lights Off
#define LIGHT_GREEN		  			1		// Green Light On
#define LIGHT_YELLOW	  			2		// Yellow Light On
#define LIGHT_RED				    	3		// Red Light On

//EEPROM Address for storing the last selected time
#define TIME_LIM_INDEX_EEPROM_ADDRESS	0

LiquidCrystal_I2C lcd(LCD_I2C_ADDRESS,16,2);

// OneButton class handles Debounce and detects button press
OneButton btnMan(PIN_MANUAL_BUTTON, HIGH);	// Pushbutton for Manual Mode
OneButton btnRot(PIN_ROTARY_SW, HIGH);		  // Rotary Select button

volatile int time_lim_index = 0;    // Time limit currently selected via Rotary Encoder
                  
unsigned long start_millis;         // millis() value at time Start is pressed
int start_elapsed_sec;              // Elapsed seconds at time Start is pressed
int elapsed_sec;                    // Total Elapsed Seconds
int old_elapsed_sec;                // Previous Total Elapsed Seconds

int timer_running;                  // TRUE if the timer is currently running

// Defines the minimum and maximum times in minutes.  
//    {0,0} means Manual Mode.  {5,7} means 5 to 7 minutes.
int time_limits[13][2] = { { 0,0 },{ 1,2 },{ 2,3 },{ 3,5 },{ 4,6 },{ 5,7 },{ 6,8 },
                           { 7,9 },{ 8,10 },{ 10,12 },{ 10,14 },{ 12,15 },{ 15,20 } };
const int time_limit_count = 13;

int min_min;   // Minimum Minutes for selected Time Limit
int max_min;   // Maximum Minutes for selected Time Limit
int min_sec;   // Minimum Seconds for selected Time Limit (Green Light turns on)
int max_sec;   // Maximum Seconds for selected Time Limit (Red Light turns on)
int mid_sec;   // Mid-point Seconds for selected Time Limit (Yellow Light turns on)

int manual_mode;                // TRUE if currently in Manual Mode
int time_lim_changed;           // TRUE if selected Time Limit has changed
int manual_mode_light_select;   // In Manual mode, defines the Light selected

int rotary_disabled;            // TRUE to Disable Rotary Encoder
int old_light_index;            // Light that was on at previous cycle

// ****************************************************************************
// rotaryClick() - Called by the Interrupt pin, whenever the Rotary
//                 Encoder was turned
// ****************************************************************************
void rotaryClick() {                    

	static unsigned long                lastInterruptTime = 0;
	unsigned long                       interruptTime = millis();

	if (rotary_disabled) return;

	// If interrupts come faster than 5ms, assume it's a bounce and ignore
	if (interruptTime - lastInterruptTime > 5) {
		// A HIGH value on the Rotary DATA pin means that the Rotary Encoder was
    // turned to the Left (Counter-Clockwise).  A LOW value on Rotary DATA
    // means that the Rotary Encoder was turned to the Right (Clockwise)		
		if (digitalRead(PIN_ROTARY_DAT))
			rotaryDown(); 
		else
			rotaryUp();
	}
	lastInterruptTime = interruptTime;
}

// ****************************************************************************
// setup() - Initialization Function
// ****************************************************************************
void setup()
{
  // Initialize the I2C LCD
	lcd.begin();
	lcd.backlight();

  // Set the Directions of the I/O Pins
	pinMode(PIN_ROTARY_CLK, INPUT);
	pinMode(PIN_ROTARY_DAT, INPUT);
	pinMode(PIN_ROTARY_SW, INPUT);
	pinMode(PIN_MANUAL_BUTTON, INPUT_PULLUP);
	pinMode(PIN_LIGHT_GREEN, OUTPUT);
	pinMode(PIN_LIGHT_YELLOW, OUTPUT);
	pinMode(PIN_LIGHT_RED, OUTPUT);

	// Relay module takes IN=HIGH for Open and IN=LOW for Close
	digitalWrite(PIN_LIGHT_GREEN, HIGH);
	digitalWrite(PIN_LIGHT_YELLOW, HIGH);
	digitalWrite(PIN_LIGHT_RED, HIGH);
	
	// Initialize to No Light On for Manual Mode
	old_light_index = LIGHT_NONE;
	manual_mode_light_select = LIGHT_NONE;

  // Interrupt to detect Clicks on the Rotary Encoder
	attachInterrupt(0, rotaryClick, FALLING);

  // Initialize the Manual Light Advance Button
	btnMan.attachClick(&btnManClick);
	
  // Initialize the Rotary Encoder Press Button
	btnRot.attachClick(&btnRotClick);
	btnRot.attachLongPressStart(&btnRotLongPress);
	btnRot.setPressTicks(2000);

  // Read the last Time Limit Selected from EEPROM and write to LCD
	readSelectedTime();
	time_lim_changed = 1;
	refreshElapsedTime();
	
	// Rotary Encoder Enabled
	rotary_disabled = 0;	
}

// ****************************************************************************
// rotaryUp() - When Rotary Encoder is turned to the Right (clockwise)
//              increase the selected Time Limits 
// ****************************************************************************
void rotaryUp()
{
	time_lim_index++;
	if (time_lim_index >= time_limit_count) time_lim_index = time_limit_count - 1;
	time_lim_changed = 1;
}

// ****************************************************************************
// rotaryDown() - When Rotary Encoder is turned to the Left (counter-clockwise)
//              decrease the selected Time Limits 
// ****************************************************************************
void rotaryDown()
{
	time_lim_index--;
	if (time_lim_index<0) time_lim_index = 0;
	time_lim_changed = 1;
}

// ****************************************************************************
// btnManClick() - Manual Light Advance button is clicked 
// ****************************************************************************
void btnManClick()
{
  // If Timer is Running, and Not Manual Mode, do nothing
	if (timer_running && !manual_mode) return;

  // Increment the selected light: NONE --> GREEN --> YELLOW --> RED --> NONE
	manual_mode_light_select++;
	if (manual_mode_light_select > LIGHT_RED)
		manual_mode_light_select = LIGHT_NONE;
}

// ****************************************************************************
// btnRotClick() - Rotary Encoder Select Button is pressed 
// ****************************************************************************
void btnRotClick()
{
	startPressed();
}

// ****************************************************************************
// btnRotLongPress() - Rotary Encoder Select Button is Held Down (Long Press) 
// ****************************************************************************
void btnRotLongPress()
{
	resetPressed();
}

// ****************************************************************************
// loop() - Main Program Loop Function 
// ****************************************************************************
void loop()
{
  // Check for button press
	btnMan.tick();
	btnRot.tick();

  // Do the "Normal" Timing Light Loop Function
	normalLoop();

 // Loop functions for other modes could be implemented here.
 // ....
}

// ****************************************************************************
// normalLoop() - Main Timing Light Loop function.  This handles the Timer,
//                Light Change, and Reset
// ****************************************************************************
void normalLoop()
{

  // If the Timer is Running, calculate the Elapsed Seconds based on the
  // time since the Start button was pressed, and the Elaped Time at
  // the Start button press
	if (timer_running)
		elapsed_sec = start_elapsed_sec + ((millis() - start_millis) / 1000);

  // Update the Elapsed Time on the LCD only if it has changed since the 
  // last normalLoop() cycle
	if (elapsed_sec != old_elapsed_sec)
		refreshElapsedTime();
	old_elapsed_sec = elapsed_sec;

  // If the Time Limit has changed, because the Rotary Encoder was turned,
  // update memory and LCD with the new Time Limits, and save the selected
  // Time Limits to the EEPROM
	if (time_lim_changed)
	{
		refreshTimeLim();
		writeSelectedTime();
	}

  // Update the Lights On
	updateLights();
	
	delay(10); // Delay because the loop does not need to execute continuously
}

// ****************************************************************************
// refreshElapsedTime() - Calculate the Elapsed Minutes and Seconds and
//                        print to the LCD
// ****************************************************************************
void refreshElapsedTime() {

	int mins;
	int sec;

	lcd.setCursor(0, 1);  // Elapsed Time is on Second Row of LCD

  // Calculate Minutes and Seconds
	mins = elapsed_sec / 60;
	sec = elapsed_sec - (mins * 60);

  // If the Timer was Stopped (and not yet Reset), then show the word "STOP"
  // next to the eleapsed time
	if (!timer_running && elapsed_sec>0)
		lcd.print(F("STOP  "));
	else
		lcd.print(F("      "));

  // Display Elapsed Time in Minutes and Seconds
	lcd.print(mins);
	lcd.print(F(":"));
	if (sec<10) lcd.print(F("0"));
	lcd.print(sec);
	lcd.print(F("    "));
}

// ****************************************************************************
// startPressed() - Rotary Encoder (Start/Stop) button was Pressed
// ****************************************************************************
void startPressed()
{
  // If Timer is Running, then Stop the Timer
	if (timer_running)
	{
		timer_running = 0;
		start_elapsed_sec = elapsed_sec;
	}

  // If Timer was Not Running, then Start the Timer
	else
	{
		timer_running = 1;
		start_millis = millis();

    // Turn Off all Lights when Timer Starts, unless in Manual Mode
		if (!manual_mode) manual_mode_light_select = LIGHT_NONE;
	}

  // Update the Elapsed Time on the LCD
	refreshElapsedTime();
}

// ****************************************************************************
// resetPressed() - Rotary Encoder Switch was Held (Long Press) for Reset
// ****************************************************************************
void resetPressed()
{
  // Reset Elapsed Time to Zero, only if Timer is Not Running
	if (timer_running == 0)
	{
		elapsed_sec = 0;
		start_elapsed_sec = 0;
	}
}

// ****************************************************************************
// writeSelectedTime() - Stores the Selected Time Limits in EEPROM.  This 
//                       allows the Timing Light to "Remember" the last
//                       selected time after Power-down / Power-up
// ****************************************************************************
void writeSelectedTime()
{
	eeprom_write_byte(TIME_LIM_INDEX_EEPROM_ADDRESS, (byte)time_lim_index);
}

// ****************************************************************************
// readSelectedTime() - Read the last Selected Time Limits from EEPROM
//                      This is called during Setup
// ****************************************************************************
void readSelectedTime()
{
	time_lim_index = eeprom_read_byte(TIME_LIM_INDEX_EEPROM_ADDRESS);
	if (time_lim_index<0) time_lim_index = 0;
	if (time_lim_index>time_limit_count - 1) time_lim_index = time_limit_count - 1;
}

// ****************************************************************************
// refreshTimeLim() - After the Selected Time Limit changes, this updates
//                    the associated variables and LCD
// ****************************************************************************
void refreshTimeLim()
{
    // Calculate Minimum, Middle, and Maximum Seconds
		min_min = time_limits[time_lim_index][0];
		max_min = time_limits[time_lim_index][1];
		min_sec = min_min * 60;
		max_sec = max_min * 60;
		mid_sec = (max_sec + min_sec) / 2;

  // A selected Time Limit index of 0 means MANUAL MODE
	if (time_lim_index == 0)
		manual_mode = 1;
	else
		manual_mode = 0;

  // Copy the new Time Limits to LCD
	timeLim2LCD();
	time_lim_changed = 0;
}

// ****************************************************************************
// timeLim2LCD() - Prints the Selected Time Limit to the LCD
// ****************************************************************************
void timeLim2LCD() {

	lcd.setCursor(0, 0);  // Time Limit is on First Row
	
	if (manual_mode)
	{
    // Manual Mode Selected
		lcd.print(F("  Manual Mode    "));
	}
	else
	{
		lcd.print(F("Time: "));
		lcd.print(min_min);

    // For a time limit >= 10 minutes write it in the form:
    //         Time: 10-12 min
    // For a time limit < 10 minutes write it in the form:
    //         Time: 5 to 7 min
    // This will ensure it fits within 16 characters
		if (max_min >= 10)
			lcd.print(F("-"));
		else
			lcd.print(F(" to "));

		lcd.print(max_min);
		lcd.print(F(" min  "));		
	}
}

// ****************************************************************************
// updateLights() - Sets the correct light on (NONE, GREEN, YELLOW, or RED)
//                  based on the Elapsed Time, or the Manual Light Select
// ****************************************************************************
void updateLights()
{
	int s;

	s = elapsed_sec;

	if (manual_mode || (!timer_running && s == 0))
	{
    // If in Manual Mode, set whatever light was selected with Manual button
		setLightOn(manual_mode_light_select);
	}
	else
	{
    // Set light on based upon the elapsed time:
    //   Less than minimum time --> NONE
    //   Greater than minimum time, but less than middle time --> GREEN
    //   Greater than middle time, but less than maximum time --> YELLOW
    //   Greater than maximum time --> RED
		if (s<min_sec || s == 0) setLightOn(LIGHT_NONE);
		if (s >= min_sec && s<mid_sec) setLightOn(LIGHT_GREEN);
		if (s >= mid_sec && s<max_sec)  setLightOn(LIGHT_YELLOW);
		if (s >= max_sec)  setLightOn(LIGHT_RED);
	}
}

// ****************************************************************************
// setLightOn() - Sets the value of the OUTPUT PINS so that the correct light 
//                (NONE, GREEN, YELLOW, or RED) is shown on
// ****************************************************************************
void setLightOn(int light_index)
{

	if (light_index != old_light_index) // Only do anything if the Light has changed
	{
    // Disable the Rotary Encoder for 100 ms.  This will prevent any electrical
    // noise caused by Relay Contacts closing or opening from causing a false
    // click in the Rotary Encoder 
		rotary_disabled = 1;
		delay(100);
   
		switch (light_index)
		{
		case LIGHT_NONE:
			setOutput(PIN_LIGHT_GREEN, 0);
			setOutput(PIN_LIGHT_YELLOW, 0);
			setOutput(PIN_LIGHT_RED, 0);
			break;
		case LIGHT_GREEN:
			setOutput(PIN_LIGHT_GREEN, 1);
			setOutput(PIN_LIGHT_YELLOW, 0);
			setOutput(PIN_LIGHT_RED, 0);
			break;
		case LIGHT_YELLOW:
			setOutput(PIN_LIGHT_GREEN, 0);
			setOutput(PIN_LIGHT_YELLOW, 1);
			setOutput(PIN_LIGHT_RED, 0);
			break;
		case LIGHT_RED:
			setOutput(PIN_LIGHT_GREEN, 0);
			setOutput(PIN_LIGHT_YELLOW, 0);
			setOutput(PIN_LIGHT_RED, 1);
			break;
		}

    // After opening/closing any relay contacts, pause another 100 ms before
    // Re-enabling the Rotary Encoder to prevent interferance from electrical
    // noise.
		delay(100);
		rotary_disabled = 0;
		
		// Store the old Select Light value
		old_light_index = light_index;
	}
}

// ****************************************************************************
// setOutput() - Converts an input of 0=OFF and 1=ON to a pin output of
//               HIGH = OFF and LOW = ON
// ****************************************************************************
void setOutput(int pin, int state)
{
	// Relay module requires a Ground to Energize relay and +5V to de-energize
	// Thus turning Light On, requires a LOW on the output

	if (state>0)
		digitalWrite(pin, LOW);
	else
		digitalWrite(pin, HIGH);
}
