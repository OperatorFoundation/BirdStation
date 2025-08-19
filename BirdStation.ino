/**
BirdStation - Backyard bird monitoring station
Built for the Songbird hardware platform.

This is an automated bird detection and environmental monitoring system
designed to provide research-grade data for citizen contribution to science and conservation.

Educational Note:
  This system demonstrates embedded audio processing, multi-sensor integration, and power efficient field deployment.
*/

// ============================================================================
// CONFIGURATION AND INCLUDES
// ============================================================================

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <TimeLib.h>

// Module Headers
#include "config.h"
#include "AudioDetector.h"
#include "LocationManager.h"
#include "PowerManager.h"
#include "DataLogger.h"
#include "DisplayManager.h"

// ============================================================================
// DEBUG SYSTEM
// ============================================================================

// Debug levels: 0=off, 1=basic, 2=verbose
#define DEBUG_LEVEL 2

#define DEBUG_PRINT(level, msg) \
    if (DEBUG_LEVEL >= level) { \
      Serial.print("["); \
      Serial.print(millis()/1000); \
      Serial.print("s] "); \
      Serial.print(msg); \
    }

#define DEBUG_PRINTLN(level, msg) \
    if (DEBUG_LEVEL >= level) { \
        Serial.print("["); \
        Serial.print(millis()/1000); \
        Serial.print("s] "); \
        Serial.println(msg); \
    }

// ============================================================================
// GLOBAL OBJECTS
// ============================================================================

// Core system modules
AudioDetector audio_detector;
LocationManager location_manager;
PowerManager power_manager;
DataLogger data_logger;
DisplayManager display_manager;

// System State
enum SystemState {
  STATE_INITIALIZING,
  STATE_IDLE,
  STATE_DETECTING,
  STATE_RECORDING,
  STATE_SAVING,
  STATE_LOW_POWER,
  STATE_ERROR
}

SystemState current_state = STATE_INITIALIZING;
uint32_t state_change_time = 0;

// Timing Variables
uint32_t last_detection_check = 0;
uint32_t last_env_reading = 0;
uint32_t last_display_update = 0;
uint32_t last_gps_check = 0;
uint32_t last_status_report = 0;

// Detection Statistics
uint32_t daily_detection_count = 0;
uint32_t total_detection_count = 0;
uint32_t last_detection_time = 0;

// ============================================================================
// SETUP FUNCTION
// ============================================================================

void setup() 
{
  // Initialize serial communication for debugging
  Serial.begin(115200);

  printStartupBanner();

  // Initialize hardware pins
  initializePins();

  // Initialize I2C buses
  // Wire = default I2C for BME280 and LIS3DH
  // Wire1 = secondary I2C for OLED display
  Wire.begin();
  Wire.setClock(400000);  // 400kHz for faster communication
  Wire1.begin();
  Wire1.setClock(400000);

  // Initialize Each Module
  bool init_success = true;

  DEBUG_PRINTLN(1, "Initializing Power Manager...");
  if (!power_manager.begin())
  {
    DEBUG_PRINTLN(1, "ERROR: Power Manager failed to initialize");
    init_success = false;
  }
  else 
  {
    DEBUG_PRINTLN(1, "Power Manager: OK");
    power_manager.printStatus();
  }

  DEBUG_PRINTLN(1, "Initializing Display...");
  if (!display_manager.begin()) 
  {
    DEBUG_PRINTLN(1, "WARNING: Display failed to initialize (non-critical)");
    // Don't set init_success = false, display is non-critical
  } 
  else 
  {
    DEBUG_PRINTLN(1, "Display: OK");
    display_manager.showStartup();
  }
  
  DEBUG_PRINTLN(1, "Initializing SD Card and Data Logger...");
  if (!data_logger.begin()) 
  {
    DEBUG_PRINTLN(1, "ERROR: SD Card/Data Logger failed to initialize");
    init_success = false;
    display_manager.showError("SD Card Failed");
  } 
  else 
  {
    DEBUG_PRINTLN(1, "Data Logger: OK");
    data_logger.printStats();
  }
  
  DEBUG_PRINTLN(1, "Initializing Location Manager (GPS + Accelerometer)...");
  if (!location_manager.begin()) 
  {
    DEBUG_PRINTLN(1, "WARNING: Location Manager partially initialized");
    // Don't fail completely - GPS might need time to get fix
  } 
  else 
  {
    DEBUG_PRINTLN(1, "Location Manager: OK");
  }
  
  DEBUG_PRINTLN(1, "Initializing Audio Detector...");
  if (!audio_detector.begin()) 
  {
    DEBUG_PRINTLN(1, "ERROR: Audio Detector failed to initialize");
    init_success = false;
    display_manager.showError("Audio Failed");
  } 
  else 
  {
    DEBUG_PRINTLN(1, "Audio Detector: OK");
  }

  // Set system time (will be updated by GPS when available)
  setTime(12, 0, 0, 1, 1, 2025);  // Default time until GPS sync

  // Load configuration from SD card if available
  loadConfiguration();

  if (init_success)
  {
    current_state = STATE_IDLE;
    DEBUG_PRINTLN(1, "=== BirdStation Ready ===");
    display_manager.showStatus(daily_detection_count, 
                                power_manager.getBatteryPercent(),
                                location_manager.getCurrentLocationName());
    
    // Log system startup
    data_logger.logSystemEvent("STARTUP", "BirdStation initialized");
  }
  else 
  {
    current_state = STATE_ERROR;
    DEBUG_PRINTLN(1, "=== BirdStation Initialization Failed ===");
    display_manager.showError("Init Failed");

    // Log system startup failure
    data_logger.logSystemEvent("STARTUP", "BirdStation failed initialization");
  }
}


// ============================================================================
// MAIN LOOP
// ============================================================================

void loop() 
{
  // Get current time for this loop iteration
  uint32_t current_millis = millis();

  // Always update critical systems
  power_manager.update();
  handleUserInput();

  // Handles state
  switch (current_state) 
  {
    case STATE_IDLE:
      handleIdleState(current_millis);
      break;

    case STATE_DETECTING:
      handleDetectingState(current_millis);
      break;

    case STATE_RECORDING:
      handleRecordingState(current_millis);
      break;

    case STATE_SAVING:
      handleSavingState(current_millis);
      break;
    
    case STATE_LOW_POWER:
      handleLowPowerState(current_millis);
      break;

    case STATE_ERROR:
      handleErrorState(current_millis);
      break;
  }

  // Regardless of state...
  performPeriodicUpdates(current_millis);

  // Small delay to prevent CPU hogging
  delay(10);
}

// ============================================================================
// STATE HANDLERS
// ============================================================================

void handleIdleState(uint32_t current_millis)
{
  // Check for bird sounds
  if (current_millis - last_detection_check > 100) // Check every 100 milliseconds
  {
    last_detection_check = current_millis;

    if (audio_detector.checkForBird())
    {
      DEBUG_PRINTLN(1, "Heard bird! Recording...");
      changeState(STATE_DETECTING);

      // FIXME: Placeholder - Flash LED to indicate detection
      digitalWrite(LED_1_PIN, HIGH);
    }
  }

  if (power_manager.shouldEnterLowPower())
  {
    DEBUG_PRINTLN(1, "Entering low power mode");
    changeState(STATE_LOW_POWER);
  }
}

void handleDetectingState(uint32_t current_millis)
{
  // Analyze the audio to confirm if it is likely a bird
  AudioQuality quality = audio_detector.analyzeAudioQuality();

  if (quality.overall_quality >= 2) // Fair or better quality
  {
    DEBUG_PRINT(2, "Audio quality: ");
    DEBUG_PRINTLN(2, quality.overall_quality);

    // Start recording
    audio_detector.startRecording();
    changeState(STATE_RECORDING);
    last_detection_time = current_millis;
  }
  else 
  {
    // False trigger, return to idle state
    DEBUG_PRINTLN(2, "False trigger, returning to idle");
    digitalWrite(LED_1_PIN, LOW);
    changeState(STATE_IDLE);
  }
}

void handleRecordingState(uint32_t current_millis)
{
  // Record for 30 seconds
  if (current_millis - state_change_time > 30000)
  {
    DEBUG_PRINTLN(1, "Recording complete, saving...");

    audio_detector.stopRecording();
    changeState(STATE_SAVING);

    // Turn off detection LED
    digitalWrite(LED_1_PIN, LOW);
  }
  else 
  {
    // Update display with recording progress
     if (current_millis - last_display_update > 1000)
     {
      uint8_t seconds_remaining = 30 - ((current_millis - state_change_time) / 1000);
      display_manager.showRecording(seconds_remaining);
      last_display_update = current_millis;
     }
  }
}

void handleSavingState(uint32_t current_millis)
{
  // Create detection record
  BirdDetection detection;
  detection.timestamp = now();
  detection.location = location_manager.getCurrentLocation();
  detection.environment = location_manager.getCurrentEnvironment();
  detection.audio_quality = audio_detector.getLastQuality();

  // Generate filename
  sprintf(detection.audio_filename, "bird_%08lu.wav", detection.timestamp);

  // Save audio file and metadata
  if (audio_detector.saveRecording(detection.audio_filename))
  {
    // Log detection to CSV
    data_logger.logDetection(detection);


    // Update counters
    daily_detection_count++;
    total_detection_count++;

    DEBUG_PRINT(1, "Detection saved: ");
    DEBUG_PRINTLN(1, detection.audio_filename);

    // Update display
    display_manager.showStatus(daily_detection_count,
                              power_manager.getBatteryPercent(),
                              location_manager.getCurrentLocationName());
  }
  else
  {
    DEBUG_PRINTLN(1, "ERROR: Failed to save recording");
    display_manager.showError("Save Failed");
  }

  changeState(STATE_IDLE);
}

void handleLowPowerState(uint32_t current_millis)
{
  // In low power mode, check less frequently
  if (current_millis - last_detection_check > 1000) // Check every second
  {
    last_detection_check = current_millis;

    // Check if we can exit low power mode
    if (!power_manager.shouldEnterLowPower())
    {
      DEBUG_PRINTLN(1, "Exiting low power mode");
      changeState(STATE_IDLE);
    }
  }

  // Display low power warning
  if (current_millis - last_display_update > 5000) 
  {
      display_manager.showLowPower(power_manager.getBatteryPercent());
      last_display_update = current_millis;
  }
}

void handleErrorState(uint32_t current_millis)
{
  // Flash error LED
  static bool led_state = false;
  static uint32_t last_flash = 0;

  if (current_millis - last_flash > 500)
  {
    led_state = !led_state;
    digitalWrite(LED_2_PIN, led_state);
    last_flash = current_millis;
  }

  // Try to recover every 30 seconds
  if (current_millis - state_change_time > 30000)
  {
    DEBUG_PRINTLN(1, "Attempting to recover from error state...");
    setup(); // Reinitialize
  }
}


// ============================================================================
// PERIODIC UPDATES
// ============================================================================

void performPeriodicUpdates(uint32_t current_millis)
{
  // Update environmental readings every minute
  if (current_millis - last_env_reading > 60000)
  {
    last_env_reading = current_millis;
    location_manager.updateEnvironmentalData();

    // TODO: Farenheit
    DEBUG_PRINT(2, "Environment: ");
    DEBUG_PRINT(2, location_manager.getCurrentEnvironment().temperature);
    DEBUG_PRINT(2, "°C, ");
    DEBUG_PRINT(2, location_manager.getCurrentEnvironment().humidity);
    DEBUG_PRINTLN(2, "%");
  }

  // Update display every 5 seconds (when not in special state)
  if (current_state == STATE_IDLE && current_millis - last_display_update > 5000)
  {
    last_display_update = current_millis;
    display_manager.showStatus(daily_detection_count,
                              power_manager.getBatteryPercent(),  
                              location_manager.getCurrentLocationName());
  }

  // Check GPS daily (or when forced by user)
  if (current_millis - last_gps_check > 86400000) // 24 hours
  {
    last_gps_check = current_millis;
    DEBUG_PRINTLN(1, "Performing daily GPS check...");
    location_manager.performGPSCheck();
  }

  // Status report every 10 minutes (debug mode)
  if (DEBUG_LEVEL >= 1 && current_millis - last_status_report > 600000) 
  {
    last_status_report = current_millis;
    printStatusReport();
  }

  // Reset daily counter at midnight
  if (hour() == 0 && minute() == 0 && second() < 2) 
  {
    daily_detection_count = 0;
    data_logger.logSystemEvent("DAILY_RESET", "Daily counter reset");
  }
}


// ============================================================================
// USER INPUT HANDLING
// ============================================================================

void handleUserInput()
{
  static uint32_t button_press_time[4] = {0, 0, 0, 0};
  static bool button_pressed[4] = {false, false, false, false};
  static bool long_press_handled[4] = {false, false, false, false};

  // Check each button
  checkButton(BTN_UP_PIN, 0, button_press_time, button_pressed, long_press_handled);
  checkButton(BTN_DOWN_PIN, 1, button_press_time, button_pressed, long_press_handled);
  checkButton(BTN_LEFT_PIN, 2, button_press_time, button_pressed, long_press_handled);
  checkButton(BTN_RIGHT_PIN, 3, button_press_time, button_pressed, long_press_handled);

  // Check for button combinations
  if (digitalRead(BTN_UP_PIN) == LOW && digitalRead(BTN_DOWN_PIN) == LOW)
  {
    // Force GPS check
    DEBUG_PRINTLN(1, "User forced GPS check");
    location_manager.performGPSCheck();
    last_gps_check = millis();
  }
}

void checkButton(int pin, int index, uint32_t* press_time, bool* pressed, bool* long_handled)
{
  if (digitalRead(pin) == LOW) // Button Pressed (active low)
  {
    if (!pressed[index])
    {
      pressed[index] = true;
      press_time[index] = millis();
      long_handled[index] = false;
    }
    else if (!long_handled[index] && millis() - press_time[index] > 2000)
    {
      // Long press detected
      handleLongPress(pin);
      long_handled[index] = true;
    }
  }
  else  // Button released
  {
    if (pressed[index] && !long_handled[index]) // Short press
    {
      handleShortPress(pin);
    }
    pressed[index] = false;
  }
}

void handleShortPress(int pin)
{
  DEBUG_PRINT(2, "Short press: ");
  DEBUG_PRINTLN(2, pin);

  switch(pin) 
  {
    case BTN_UP_PIN:
        display_manager.nextScreen();
        break;

    case BTN_DOWN_PIN:
        display_manager.previousScreen();
        break;

    case BTN_LEFT_PIN:
        display_manager.scrollLeft();
        break;

    case BTN_RIGHT_PIN:
        display_manager.scrollRight();
        break;
  }
}

void handleLongPress(int pin) 
{
    DEBUG_PRINT(2, "Long press: ");
    DEBUG_PRINTLN(2, pin);
    
    switch(pin) 
    {
      case BTN_UP_PIN:
        display_manager.adjustBrightness();
        break;
    }
}


// ============================================================================
// HELPER FUNCTIONS
// ============================================================================

void initializePins() 
{
  // User interface pins
  pinMode(BTN_UP_PIN, INPUT_PULLUP);
  pinMode(BTN_DOWN_PIN, INPUT_PULLUP);
  pinMode(BTN_LEFT_PIN, INPUT_PULLUP);
  pinMode(BTN_RIGHT_PIN, INPUT_PULLUP);
  pinMode(LED_1_PIN, OUTPUT);
  pinMode(LED_2_PIN, OUTPUT);
  
  // Power monitoring pins
  pinMode(BATTERY_VOLTAGE_PIN, INPUT);
  pinMode(SOLAR_VOLTAGE_PIN, INPUT);
  
  // Turn off LEDs initially
  digitalWrite(LED_1_PIN, LOW);
  digitalWrite(LED_2_PIN, LOW);
}

void changeState(SystemState new_state) 
{
  DEBUG_PRINT(2, "State change: ");
  DEBUG_PRINT(2, stateToString(current_state));
  DEBUG_PRINT(2, " -> ");
  DEBUG_PRINTLN(2, stateToString(new_state));
  
  current_state = new_state;
  state_change_time = millis();
}

const char* stateToString(SystemState state) 
{
  switch(state) 
  {
    case STATE_INITIALIZING: return "INITIALIZING";
    case STATE_IDLE: return "IDLE";
    case STATE_DETECTING: return "DETECTING";
    case STATE_RECORDING: return "RECORDING";
    case STATE_SAVING: return "SAVING";
    case STATE_LOW_POWER: return "LOW_POWER";
    case STATE_ERROR: return "ERROR";
    default: return "UNKNOWN";
  }
}

void loadConfiguration() 
{
  // Load configuration from SD card
  if (data_logger.configExists()) 
  {
    DEBUG_PRINTLN(1, "Loading configuration from SD card... (Not Implemented)");
    // TODO: Implement configuration loading
  } 
  else 
  {
    DEBUG_PRINTLN(1, "No configuration file found, using defaults");
    saveDefaultConfiguration();
  }
}

void saveDefaultConfiguration() 
{
  // Save default configuration to SD card
  data_logger.logSystemEvent("CONFIG", "Default configuration saved");
  // TODO: Implement configuration saving
}

void printStartupBanner() 
{
  Serial.println();
  Serial.println("=========================================");
  Serial.println("     BirdStation v0.0.1");
  Serial.println("     Solar-Powered Bird Monitor");
  Serial.println("     Running on Songbird Hardware");
  Serial.println("=========================================");
  Serial.println();
}

void printStatusReport() 
{
  Serial.println("\n=== Status Report ===");
  Serial.print("Uptime: ");
  Serial.print(millis() / 1000 / 60);
  Serial.println(" minutes");
  
  Serial.print("State: ");
  Serial.println(stateToString(current_state));
  
  Serial.print("Detections Today: ");
  Serial.println(daily_detection_count);
  
  Serial.print("Total Detections: ");
  Serial.println(total_detection_count);
  
  Serial.print("Battery: ");
  Serial.print(power_manager.getBatteryPercent());
  Serial.println("%");
  
  Serial.print("Solar: ");
  Serial.println(power_manager.isSolarCharging() ? "Charging" : "Not charging");
  
  Serial.print("Location: ");
  Serial.println(location_manager.getCurrentLocationName());
  
  Serial.print("GPS Status: ");
  Serial.println(location_manager.getGPSStatusString());
  
  Serial.print("SD Card Free: ");
  Serial.print(data_logger.getFreeSpaceMB());
  Serial.println(" MB");
  
  Serial.println("====================\n");
}
