#include "Arduino.h"
#include <Wire.h>
#include <EEPROM.h>
#include "LiquidCrystal_I2C.h"
#include "OneButton.h"


// set the LCD address to 0x27 for a 20 chars 4 line display
// Set the pins on the I2C chip used for LCD connections:
//                    addr, en,rw,rs,d4,d5,d6,d7,bl,blpol
// Alternately, some boards use 0x3F as the I2C Address

//#define LCD_I2C_ADDRESS 0x27
#define LCD_I2C_ADDRESS 0x3F

#define PIN_ROTARY_CLK				2		// Used for generating interrupts using CLK signal
#define PIN_ROTARY_DAT				3		// Used for reading DT signal
#define PIN_ROTARY_SW				4		// Used for the Rotary push button switch
#define PIN_LIGHT_GREEN				5		// Output pin for Green Light
#define PIN_LIGHT_YELLOW			6		// Output pin for Yellow Light
#define PIN_LIGHT_RED				7		// Output pin for Red Light
#define PIN_MANUAL_BUTTON			14		// Pushbutton to change light in Manual Mode

#define LIGHT_NONE					0		// All Lights Off
#define LIGHT_GREEN					1		// Green Light On
#define LIGHT_YELLOW				2		// Yellow Light On
#define LIGHT_RED					3		// Red Light On

#define TIME_LIM_INDEX_EEPROM_ADDRESS	0

LiquidCrystal_I2C lcd(LCD_I2C_ADDRESS, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);  // Set the LCD I2C address

OneButton btnMan(PIN_MANUAL_BUTTON, HIGH);	// Pushbutton for Manual Mode
OneButton btnRot(PIN_ROTARY_SW, HIGH);		// Rotary Select button

volatile int time_lim_index = 0;
                  
unsigned long start_millis;
int start_elapsed_sec;
int elapsed_sec;
int old_elapsed_sec;

int timer_running;


int time_limits[13][2] = { { 0,0 },{ 1,2 },{ 2,3 },{ 3,5 },{ 4,6 },{ 5,7 },{ 6,8 },{ 7,9 },{ 8,10 },{ 10,12 },{ 10,14 },{ 12,15 },{ 15,20 } };
const int time_limit_count = 13;

int min_min;
int max_min;
int min_sec;
int max_sec;
int mid_sec;

int manual_mode;
int time_lim_changed;
int manual_mode_light_select;

int rotary_disabled;
int old_light_index;

void rotaryClick() {                    

	static unsigned long                lastInterruptTime = 0;
	unsigned long                       interruptTime = millis();

	if (rotary_disabled) return;

	// If interrupts come faster than 5ms, assume it's a bounce and ignore
	if (interruptTime - lastInterruptTime > 5) {
		if (digitalRead(PIN_ROTARY_DAT))
			rotaryDown();
		else
			rotaryUp();
	}
	lastInterruptTime = interruptTime;
}


void setup()
{
	lcd.begin(16, 2);
	lcd.backlight();

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
	old_light_index = LIGHT_NONE;
	manual_mode_light_select = LIGHT_NONE;

	attachInterrupt(0, rotaryClick, FALLING);
	
	btnMan.attachClick(&btnManClick);
	btnMan.setPressTicks(2000);
  
	btnRot.attachClick(&btnRotClick);
	btnRot.attachLongPressStart(&btnRotLongPress);
	btnRot.setPressTicks(2000);

	readSelectedTime();
	time_lim_changed = 1;
	refreshElapsedTime();
	rotary_disabled = 0;	
}

void lcdDebug(String s)
{
	lcd.setCursor(0, 0);
	lcd.print(s);
	lcd.print("              ");
	delay(3000);
}

void rotaryUp()
{
	time_lim_index++;
	if (time_lim_index >= time_limit_count) time_lim_index = time_limit_count - 1;
	time_lim_changed = 1;
}

void rotaryDown()
{
	time_lim_index--;
	if (time_lim_index<0) time_lim_index = 0;
	time_lim_changed = 1;
}


void btnManClick()
{
	if (timer_running && !manual_mode) return;

	manual_mode_light_select++;
	if (manual_mode_light_select > LIGHT_RED)
		manual_mode_light_select = LIGHT_NONE;
}

void btnRotClick()
{
	startPressed();
}

void btnRotLongPress()
{
	resetPressed();
}

void loop()
{
	btnMan.tick();
	btnRot.tick();

	normalLoop();

}


void normalLoop()
{

	if (timer_running)
		elapsed_sec = start_elapsed_sec + ((millis() - start_millis) / 1000);

	if (elapsed_sec != old_elapsed_sec)
		refreshElapsedTime();
	old_elapsed_sec = elapsed_sec;

	if (time_lim_changed)
	{
		refreshTimeLim();
		writeSelectedTime();
	}

	updateLights();
	delay(10);

}

void refreshElapsedTime() {

	int mins;
	int sec;

	lcd.setCursor(0, 1);

	mins = elapsed_sec / 60;
	sec = elapsed_sec - (mins * 60);

	if (!timer_running && elapsed_sec>0)
		lcd.print(F("STOP  "));
	else
		lcd.print(F("      "));

	lcd.print(mins);
	lcd.print(F(":"));
	if (sec<10) lcd.print(F("0"));
	lcd.print(sec);
	lcd.print(F("    "));
}


void startPressed()
{
	if (timer_running)
	{
		timer_running = 0;
		start_elapsed_sec = elapsed_sec;
	}
	else
	{
		timer_running = 1;
		start_millis = millis();
		if (!manual_mode) manual_mode_light_select = LIGHT_NONE;
	}
	refreshElapsedTime();
}

void resetPressed()
{
	if (timer_running == 0)
	{
		elapsed_sec = 0;
		start_elapsed_sec = 0;
	}
}

void writeSelectedTime()
{
	eeprom_write_byte(TIME_LIM_INDEX_EEPROM_ADDRESS, (byte)time_lim_index);
}

void readSelectedTime()
{
	time_lim_index = eeprom_read_byte(TIME_LIM_INDEX_EEPROM_ADDRESS);
	if (time_lim_index<0) time_lim_index = 0;
	if (time_lim_index>time_limit_count - 1) time_lim_index = time_limit_count - 1;
}

void refreshTimeLim()
{
		min_min = time_limits[time_lim_index][0];
		max_min = time_limits[time_lim_index][1];
		min_sec = min_min * 60;
		max_sec = max_min * 60;
		mid_sec = (max_sec + min_sec) / 2;

	if (time_lim_index == 0)
		manual_mode = 1;
	else
		manual_mode = 0;

	timeLim2LCD();
	time_lim_changed = 0;
}

void timeLim2LCD() {

	lcd.setCursor(0, 0);
	if (manual_mode)
	{
		lcd.print(F("  Manual Mode    "));
	}
	else
	{
		lcd.print(F("Time: "));
		lcd.print(min_min);

		if (max_min >= 10)
			lcd.print(F("-"));
		else
			lcd.print(F(" to "));

		lcd.print(max_min);
		lcd.print(F(" min  "));		
	}
}

void updateLights()
{
	int s;

	s = elapsed_sec;

	if (manual_mode || (!timer_running && s == 0))
	{
		setLightOn(manual_mode_light_select);
	}
	else
	{
		if (s<min_sec || s == 0) setLightOn(LIGHT_NONE);
		if (s >= min_sec && s<mid_sec) setLightOn(LIGHT_GREEN);
		if (s >= mid_sec && s<max_sec)  setLightOn(LIGHT_YELLOW);
		if (s >= max_sec)  setLightOn(LIGHT_RED);
	}
}

void setLightOn(int light_index)
{

	if (light_index != old_light_index)
	{
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
	
		delay(100);
		rotary_disabled = 0;
		old_light_index = light_index;
	}
}

void setOutput(int pin, int state)
{
	// Relay module requires a Ground to Energize relay and +5V to de=energize
	// Thus turning Light On, requires a LOW on the output

	if (state>0)
		digitalWrite(pin, LOW);
	else
		digitalWrite(pin, HIGH);
}
