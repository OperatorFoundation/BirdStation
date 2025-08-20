/**
* config.h - BirdStation Configuration File
* 
* Hardware pin definitions, constants,
* and system configuration parameters.
* 
*/

#ifndef CONFIG_H
#define CONFIG_H

// ============================================================================
// HARDWARE PIN DEFINITIONS
// ============================================================================

// Audio System (Handled by Teensy Audio Library)
#define I2S_DIN_PIN      7    // I2S data in
#define I2S_DOUT_PIN     8    // I2S data out
#define I2S_SCLK_PIN     21   // I2S serial clock
#define I2S_LRCLK_PIN    20   // I2S left/right clock
#define SYS_MCLK_PIN     23   // System main clock

// Power Monitoring
#define BATTERY_VOLTAGE_PIN  A0  // Analog pin for battery voltage divider
#define SOLAR_VOLTAGE_PIN    A1  // Analog pin for solar panel voltage

// User Interface
#define BTN_UP_PIN       5    // Navigation up / settings
#define BTN_DOWN_PIN     4    // Navigation down / settings
#define BTN_LEFT_PIN     6    // Previous day's summary
#define BTN_RIGHT_PIN    3    // Next day's summary
#define LED_1_PIN        14   // Blue LED - Activity indicator
#define LED_2_PIN        15   // Pink LED - Low battery warning

// Display (I2C on Wire1)
#define OLED_SDA_PIN     17   // Secondary I2C data
#define OLED_SCL_PIN     16   // Secondary I2C clock
#define OLED_ADDRESS     0x3C // I2C address for SSD1306
#define OLED_WIDTH       128  // Display width in pixels
#define OLED_HEIGHT      32   // Display height in pixels

// SD Card
#define SDCARD_CS_PIN    10

// GPS Module (Serial)
#define GPS_SERIAL       Serial1  // Hardware serial for GPS
#define GPS_BAUD         9600     // GPS module baud rate

// Environmental Sensor (BME280 on Wire)
#define BME280_ADDRESS   0x77     // I2C address (0x77 or 0x76)

// Accelerometer (LIS3DH on Wire)
#define LIS3DH_ADDRESS   0x18     // I2C address (0x18 or 0x19)

// ============================================================================
// AUDIO PROCESSING PARAMETERS
// ============================================================================

// Frequency ranges for bird detection (Hz)
#define BIRD_FREQ_MIN    1000     // Minimum frequency for bird calls
#define BIRD_FREQ_MAX    8000     // Maximum frequency for bird calls
#define HUMAN_FREQ_MAX   300      // Maximum fundamental frequency for human speech

// Audio detection thresholds
#define AUDIO_THRESHOLD_DB   10.0    // Minimum signal above noise floor (dB)
#define MIN_DETECTION_CONFIDENCE 0.3  // Minimum confidence to trigger recording
#define RECORDING_DURATION_MS 30000   // Recording duration (30 seconds)
#define PRE_TRIGGER_BUFFER_MS 2000    // Audio to include before trigger

// Audio quality assessment
#define MIN_SNR_DB        6.0   // Minimum signal-to-noise ratio
#define GOOD_SNR_DB      15.0   // Good quality SNR threshold
#define EXCELLENT_SNR_DB 25.0   // Excellent quality SNR threshold

// ============================================================================
// FREQUENCY DETECTION PARAMETERS
// ============================================================================

// AudioAnalyzeNoteFrequency confidence thresholds
#define NOTE_CONFIDENCE_HIGH      0.8   // High confidence in frequency detection
#define NOTE_CONFIDENCE_MEDIUM    0.5   // Medium confidence threshold
#define NOTE_CONFIDENCE_LOW       0.3   // Low confidence (probably noise)

// Tonal detection frequency bounds
#define TONAL_FREQ_MIN           500    // Minimum Hz for tonal bird sounds
#define TONAL_FREQ_MAX         10000    // Maximum Hz for tonal bird sounds

// Band-specific SNR thresholds
#define MIN_SNR_DB_LOW          10.0    // Low freq bands (more rumble noise)
#define MIN_SNR_DB_MID           6.0    // Mid freq bands (standard)
#define MIN_SNR_DB_HIGH          8.0    // High freq bands
#define MIN_SNR_DB_BROADBAND     6.0    // Overall broadband threshold

// FFT Configuration
#define FFT_SIZE         1024     // FFT bin size
#define SAMPLE_RATE      44100    // Audio sample rate (Hz)

// ============================================================================
// BIRD FREQUENCY BAND DEFINITIONS
// ============================================================================

// Low frequency band (owls, doves, crows)
#define BIRD_BAND_LOW_MIN        200     // Hz - Bottom of low bird band
#define BIRD_BAND_LOW_MAX       2000     // Hz - Top of low bird band

// Mid frequency band (most songbirds, robins, thrushes)
#define BIRD_BAND_MID_MIN       2000     // Hz - Bottom of mid bird band  
#define BIRD_BAND_MID_MAX       6000     // Hz - Top of mid bird band

// High frequency band (warblers, finches, high-pitched calls)
#define BIRD_BAND_HIGH_MIN      6000     // Hz - Bottom of high bird band
#define BIRD_BAND_HIGH_MAX     12000     // Hz - Top of high bird band

// Overall bird detection range
#define BIRD_FREQ_MIN_OVERALL    200     // Hz - Lowest bird frequency we detect
#define BIRD_FREQ_MAX_OVERALL  12000     // Hz - Highest bird frequency we detect

// ============================================================================
// POWER MANAGEMENT PARAMETERS
// ============================================================================

// Battery voltage thresholds (for 1S LiPo)
#define BATTERY_FULL_VOLTAGE    4.2   // Fully charged voltage
#define BATTERY_NOMINAL_VOLTAGE 3.7   // Nominal voltage
#define BATTERY_LOW_VOLTAGE     3.5   // Low battery warning
#define BATTERY_CRITICAL_VOLTAGE 3.3  // Critical - shutdown threshold

// FIXME: Voltage divider ratio
#define BATTERY_DIVIDER_RATIO   2.0   // Example: 10k/10k divider
#define SOLAR_DIVIDER_RATIO     3.0   // Example: 20k/10k divider

// Power states
#define SOLAR_CHARGING_THRESHOLD 4.5  // Voltage indicating solar charging
#define LOW_POWER_THRESHOLD     20    // Battery percentage to enter low power mode
#define CRITICAL_POWER_THRESHOLD 10   // Battery percentage for emergency shutdown

// ============================================================================
// GPS AND LOCATION PARAMETERS
// ============================================================================

// GPS timing
#define GPS_CHECK_INTERVAL_MS    86400000  // Daily GPS check (24 hours)
#define GPS_TIMEOUT_INITIAL_MS   60000     // Initial GPS fix timeout (60 seconds)
#define GPS_TIMEOUT_RETRY_MS     120000    // Retry timeout (120 seconds)
#define GPS_MAX_AGE_HOURS        72         // Maximum GPS age before data marked invalid

// Location detection
#define MOVEMENT_THRESHOLD_G     0.5       // Accelerometer threshold for movement
#define LOCATION_CHANGE_METERS   10.0      // Distance to trigger new location
#define MAX_LOCATIONS           10         // Maximum number of stored locations

// ============================================================================
// DATA STORAGE PARAMETERS
// ============================================================================

// SD Card thresholds
#define MIN_FREE_SPACE_MB       500        // Minimum free space before cleanup
#define CRITICAL_FREE_SPACE_MB  100        // Critical space - stop recording
#define CSV_BUFFER_SIZE         512        // Buffer size for CSV writing

// File naming
#define MAX_FILENAME_LENGTH     32         // Maximum filename length
#define AUDIO_FILE_PREFIX       "bird_"    // Prefix for audio files
#define LOG_FILE_NAME          "detections.csv"  // Main log file
#define SYSTEM_LOG_NAME        "system.log"      // System event log

// ============================================================================
// ENVIRONMENTAL MONITORING
// ============================================================================

// Sensor reading intervals
#define ENV_UPDATE_INTERVAL_MS  60000      // Environmental reading interval (1 minute)
#define ENV_LOG_INTERVAL_MS     300000     // Environmental logging interval (5 minutes)

// Environmental thresholds for bird activity correlation
#define TEMP_ACTIVITY_MIN       -10.0      // Minimum temperature for activity (°C)
#define TEMP_ACTIVITY_MAX       45.0       // Maximum temperature for activity (°C)
#define HUMIDITY_CONDENSATION   95.0       // Humidity level indicating condensation risk

// ============================================================================
// DISPLAY PARAMETERS
// ============================================================================

// Display timing
#define DISPLAY_UPDATE_INTERVAL_MS 5000    // Display refresh interval
#define DISPLAY_TIMEOUT_MS        30000    // Display timeout for menus
#define DISPLAY_SCROLL_DELAY_MS   200      // Delay for scrolling text

// Display brightness levels (0-255)
#define DISPLAY_BRIGHTNESS_LOW    10
#define DISPLAY_BRIGHTNESS_MEDIUM 50
#define DISPLAY_BRIGHTNESS_HIGH   128

// ============================================================================
// SYSTEM PARAMETERS
// ============================================================================

// Timing constants
#define MAIN_LOOP_DELAY_MS       10        // Main loop delay
#define BUTTON_DEBOUNCE_MS       100       // Button debounce time
#define LONG_PRESS_DURATION_MS   2000      // Long press detection time

// System limits
#define MAX_DAILY_DETECTIONS     1000      // Maximum detections per day
#define MAX_QUEUE_SIZE          10         // Maximum queued recordings
#define SERIAL_BAUD_RATE        115200     // Serial communication speed

// Debug modes
#define ENABLE_FAKE_DETECTIONS  false      // Generate fake detections for testing
#define FAKE_DETECTION_INTERVAL 30000      // Interval for fake detections (ms)

// ============================================================================
// DATA STRUCTURES
// ============================================================================

// Audio quality metrics
struct AudioQuality {
    float signal_to_noise_ratio;    // SNR in dB
    float peak_frequency;           // Dominant frequency in Hz
    bool has_clipping;              // Audio clipping detected
    uint8_t overall_quality;        // 1=poor, 2=fair, 3=good
};

// Environmental data
struct EnvironmentalData {
    float temperature;             // Temperature in Celsius
    float humidity;                // Relative humidity in %
    float pressure;                // Atmospheric pressure in hPa
    uint32_t timestamp;            // Reading timestamp
};

// Location data with confidence
struct LocationData {
    char id[8];                   // Location identifier (e.g., "LOC001")
    char name[32];                // User-friendly name
    float latitude;               // GPS latitude
    float longitude;              // GPS longitude
    uint8_t confidence;           // 0=no GPS, 1=old, 2=good
    bool movement_detected;       // Accelerometer triggered
    uint32_t hours_since_gps;     // Hours since last GPS fix
};

// Complete bird detection record
struct BirdDetection {
    uint32_t timestamp;            // Unix timestamp
    LocationData location;         // Location with confidence
    EnvironmentalData environment; // Environmental conditions
    AudioQuality audio_quality;    // Audio quality metrics
    char audio_filename[MAX_FILENAME_LENGTH];  // Recorded audio file
    bool needs_validation;        // Requires user validation
};

// ============================================================================
// ERROR CODES
// ============================================================================

enum ErrorCode {
    ERROR_NONE = 0,
    ERROR_SD_CARD_INIT,
    ERROR_SD_CARD_FULL,
    ERROR_AUDIO_INIT,
    ERROR_GPS_INIT,
    ERROR_BME280_INIT,
    ERROR_LIS3DH_INIT,
    ERROR_DISPLAY_INIT,
    ERROR_LOW_BATTERY,
    ERROR_FILE_WRITE,
    ERROR_MEMORY_FULL
};

// ============================================================================
// UTILITY MACROS
// ============================================================================

// Convert between units
#define C_TO_F(c) ((c * 9.0/5.0) + 32.0)  // Celsius to Fahrenheit
#define F_TO_C(f) ((f - 32.0) * 5.0/9.0)  // Fahrenheit to Celsius
#define HPA_TO_INHG(hpa) (hpa * 0.02953)  // hPa to inches of mercury

// Constrain values
#define CONSTRAIN(val, min, max) ((val)<(min)?(min):((val)>(max)?(max):(val)))

// Map values
#define MAP_FLOAT(x, in_min, in_max, out_min, out_max) \
    ((x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min)

#endif // CONFIG_H