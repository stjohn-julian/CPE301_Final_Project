#include <DHT.h>
#include <LiquidCrystal.h>
#include <RTClib.h>
#include <Stepper.h>

#define ADMUX_REG (*(volatile uint8_t *)(0x7C))  // ADMUX register
#define ADCSRB_REG (*(volatile uint8_t *)(0x7B)) // ADCSRB register
#define ADCSRA_REG (*(volatile uint8_t *)(0x7A)) // ADCSRA register
#define ADC_DATA_REG (*(volatile uint16_t *)(0x78)) // ADC Data register (16 bits)

#define WATER_SENSOR_CHANNEL A1 //Water Sensor connection
#define DHTPIN 2           // Digital pin connected to the DHT sensor
#define DHTTYPE DHT11      // DHT 11 sensor is used in this example
#define FAN_PIN 8           // Digital pin connected to the fan motor
#define STEPPER_PIN1 10     // Digital pin connected to stepper motor IN1
#define STEPPER_PIN2 11     // Digital pin connected to stepper motor IN2
#define STEPPER_PIN3 12     // Digital pin connected to stepper motor IN3
#define STEPPER_PIN4 13     // Digital pin connected to stepper motor IN4
#define BUTTON_UP 6         // Digital pin connected to the button for vent control (up)
#define BUTTON_DOWN 7       // Digital pin connected to the button for vent control (down)
#define START_BUTTON 4      // Digital pin connected to the start button
#define STOP_BUTTON 5       // Digital pin connected to the stop button

// LEDs
#define ERROR_LED 12        // Digital pin connected to the Error LED
#define DISABLED_LED 11     // Digital pin connected to the Disabled LED
#define IDLE_LED 10         // Digital pin connected to the Idle LED
#define RUNNING_LED 9       // Digital pin connected to the Running LED

// LCD configuration
#define RS_PIN 34           // Register Select pin
#define ENABLE_PIN 23       // Enable pin
#define D4_PIN 6            // LCD D4 pin
#define D5_PIN 5            // LCD D5 pin
#define D6_PIN 4            // LCD D6 pin
#define D7_PIN 3            // LCD D7 pin

// RTC Configuration
RTC_DS3231 rtc;  // Create an RTC object

// Create instances
DHT dht(DHTPIN, DHTTYPE);
LiquidCrystal lcd(RS_PIN, ENABLE_PIN, D4_PIN, D5_PIN, D6_PIN, D7_PIN);
Stepper stepper(2048, STEPPER_PIN1, STEPPER_PIN2, STEPPER_PIN3, STEPPER_PIN4);

void handleErrorState();
void handleRunningState();
void reportTime();

int adc_read(unsigned char adc_channel_num) {
  // Clear the channel selection bits (MUX 4:0)
  *my_ADMUX &= 0b11100000;
  // Clear the channel selection bits (MUX 5)
  *my_ADCSRB &= 0b11110111;
  // Set the channel number
  if (adc_channel_num > 7) 
  {
    // Set the channel selection bits, but remove the most significant bit (bit 3)
    adc_channel_num -= 8;
    // Set MUX bit 5
    *my_ADCSRB |= 0b00001000;
  }
  // Set the channel selection bits
  *my_ADMUX += adc_channel_num;
  // Set bit 6 of ADCSRA to 1 to start a conversion
  *my_ADCSRA |= 0x40;
  // Wait for the conversion to complete
  while ((*my_ADCSRA & 0x40) != 0);
  // Return the result in the ADC data register
  return *my_ADC_DATA;
}

// UART Configuration
void U1Init() 
{
  UBRR1H = (unsigned char)(103 >> 8); // Set baud rate to 9600
  UBRR1L = (unsigned char)103;
  UCSR1B = (1 << RXEN1) | (1 << TXEN1); // Enable receiver and transmitter
  UCSR1C = (1 << UCSZ11) | (1 << UCSZ10); // Set frame format: 8data, 1stop bit
}

unsigned char U1kbhit() 
{
  return UCSR1A & (1 << RXC1); // Check RXC1 bit
}

unsigned char U1getchar() 
{
  while (!(UCSR1A & (1 << RXC1))) // Wait until RXC1 bit is set
    ;
  return UDR1;
}

void U1putChar(unsigned char U1pdata) 
{
  while (!(UCSR1A & (1 << UDRE1))) // Wait until UDRE1 bit is set
    ;
  UDR1 = U1pdata;
}

void U1putString(const char *str) 
{
  int iter = 0;
  while (str[iter] != '\0') 
  {
    U1putChar(str[iter]);
    iter++;
  }
}

// Variables
bool systemEnabled = true;
bool fanRunning = false;
bool stopButtonPressed = false;
bool startButtonPressed = false;

// State enumeration
enum SystemState 
{
  DISABLED,
  IDLE,
  ERROR_STATE,
  RUNNING
};

SystemState currentState = DISABLED;

void setupPins() 
{
  // Set pins as inputs or outputs using bitwise operations
  // DDRB is the Data Direction Register for PORTB
  // Here, we set pin 8 (FAN_PIN) as an output, and pin 4 (START_BUTTON) and pin 5 (STOP_BUTTON) as inputs with pull-ups
  DDRB |= (1 << FAN_PIN);
  DDRD &= ~((1 << START_BUTTON) | (1 << STOP_BUTTON));
  PORTD |= (1 << START_BUTTON) | (1 << STOP_BUTTON);  // Enable internal pull-up resistors
}

void setup() 
{
  Serial.begin(9600);
  U1Init();
  dht.begin();
  setupPins();
  pinMode(WATER_SENSOR_PIN, INPUT);
  lcd.begin(16, 2);

  if (!rtc.begin()) 
  {
    Serial.println("RTC Disconnected");
    while (1);
  }

  if (rtc.lostPower()) 
  {
    Serial.println("Setting Time");
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }
}

void loop() 
{
  // Handle button presses using interrupts
  if (startButtonPressed) 
  {
    startButtonPressed = false;
    handleStartButton();
  }

  if (stopButtonPressed) 
  {
    stopButtonPressed = false;
    handleStopButton();
  }

  // State machine
  switch (currentState) 
  {
    case DISABLED:
      handleDisabledState();
      break;

    case IDLE:
      handleIdleState();
      break;

    case ERROR_STATE:
      handleErrorState();
      break;

    case RUNNING:
      handleRunningState();
      break;

    default:
      break;
  }

  // Real-time clock reporting for all states
  reportTime();
}

void handleStartButton() 
{
  if (currentState == DISABLED) 
  {
    currentState = IDLE;
    PORTB |= (1 << IDLE_LED);
    PORTB &= ~((1 << DISABLED_LED) | (1 << ERROR_LED) | (1 << RUNNING_LED));
    Serial.println("System enabled. Transitioned to IDLE state.");
  }
}

void handleStopButton() 
{
  if (currentState != DISABLED) 
  {
    currentState = DISABLED;
    stopFan();
    PORTB &= ~((1 << IDLE_LED) | (1 << ERROR_LED) | (1 << RUNNING_LED));
    PORTB |= (1 << DISABLED_LED);
    Serial.println("System disabled. Transitioned to DISABLED state.");
  }
}

void handleDisabledState() 
{
  stopFan();
  PORTB &= ~((1 << IDLE_LED) | (1 << ERROR_LED) | (1 << RUNNING_LED));
  PORTB |= (1 << DISABLED_LED);
  // Yellow LED is ON

  // No monitoring of temperature or water in DISABLED state
}

void handleIdleState() 
{
  PORTB |= (1 << IDLE_LED);
  PORTB &= ~((1 << DISABLED_LED) | (1 << ERROR_LED) | (1 << RUNNING_LED));
  // Green LED is ON

  // Exact time stamp recording
  Serial.println("Transitioned to IDLE state - " + getTimeAndDate());

  // Water level monitoring
  int waterSensorValue = readWaterLevel());
  if (waterSensorValue < 280) 
  {
    currentState = ERROR_STATE;
    Serial.println("Water level too low! Transitioned to ERROR state.");
    return;
  }

  // Continuous monitoring of temperature and humidity
  float humidity = dht.readHumidity();
  float temperature = dht.readTemperature();

  // Example: Sending data over UART
  U1putString("Temperature: ");
  U1putString(String(temperature).c_str());
  U1putString("C, Humidity: ");
  U1putString(String(humidity).c_str());
  U1putString("%\n");

}

void handleErrorState() 
{
  // Turn off the fan
  stopFan();

  // Display an error message on the LCD
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("ERROR: Water low");
  lcd.setCursor(0, 1);
  lcd.print("Press RESET");

  // Turn on the ERROR LED
  PORTB |= (1 << ERROR_LED);
  PORTB &= ~((1 << IDLE_LED) | (1 << DISABLED_LED) | (1 << RUNNING_LED));

  // Wait for the reset button to be pressed
  while (true) 
  {
    if (digitalRead(STOP_BUTTON) == LOW) 
    {
      delay(50); // Debouncing delay
      while (digitalRead(STOP_BUTTON) == LOW){}
      break;
    }
  }

  currentState = IDLE;
  PORTB |= (1 << IDLE_LED);
  PORTB &= ~((1 << ERROR_LED) | (1 << DISABLED_LED) | (1 << RUNNING_LED));
  lcd.clear();
  Serial.println("Reset. Transitioned to IDLE state.");
}

void handleRunningState() 
{
  // Turn on the fan
  startFan();

  // Display fan status on the LCD
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Fan is running");
  lcd.setCursor(0, 1);
  lcd.print("Temp: ");
  lcd.print(dht.readTemperature());
  lcd.print("C");

  // Turn on the RUNNING LED
  PORTB |= (1 << RUNNING_LED);
  PORTB &= ~((1 << IDLE_LED) | (1 << DISABLED_LED) | (1 << ERROR_LED));

  // Check if temperature falls below a threshold
  float temperature = dht.readTemperature();
  if (temperature < 20.0) {
    currentState = IDLE;
    stopFan();
    PORTB |= (1 << IDLE_LED);
    PORTB &= ~((1 << ERROR_LED) | (1 << DISABLED_LED) | (1 << RUNNING_LED));
    lcd.clear();
    Serial.println("Temperature Low, Idling");
  }
}

int readWaterLevel() {
  // Use your specific channel number for the water level sensor
  int sensorValue = adc_read(WATER_SENSOR_CHANNEL);

  // Map the sensor value to a range (adjust the values based on your sensor characteristics)
  int waterLevel = map(sensorValue, 0, 1023, 0, 100);

  return waterLevel;
}

void reportTime() 
{
  // Report the current time using RTC
  DateTime now = rtc.now();
  Serial.print("Time: ");
  Serial.print(now.year(), DEC);
  Serial.print('/');
  Serial.print(now.month(), DEC);
  Serial.print('/');
  Serial.print(now.day(), DEC);
  Serial.print(" ");
  Serial.print(now.hour(), DEC);
  Serial.print(':');
  Serial.print(now.minute(), DEC);
  Serial.print(':');
  Serial.print(now.second(), DEC);
  Serial.println();
}



