#include <stdint.h>
#ifndef AUDIO_DETECTOR_H
#define AUDIO_DETECTOR_H

#include <Arduino.h>
#include <Audio.h>
#include <SD.h>
#include "config.h"

// Multi-band SNR
struct MultiSNR {
  float low_band_snr;                       // 200 - 2000Hz (doves, owls, crows)
  float mid_band_snr;                       // 2000 - 6000Hz (most songbirds)
  float high_band_snr;                      // 6000 - 12000Hz (warblers, finches)
  float broadband_snr;                      // 200 - 12000Hz (everything)
  float peak_frequency;                     // Dominant frequency detected
  float frequeny_confidence;                // Confidence in frequency detection
  bool is_valid;                            // Whether calculation succeeded
};

class AudioDetector
{
  public:
    AudioDetector();
    bool begin();

    // Detection
    bool checkForBird();                      // Quick check for activity
    AudioQuality analyzeAudioQuality();       // Detailed quality stats

    // Recording
    void startRecording();                    // Start a 30-second recording
    void stopRecording();
    bool saveRecording(const char* filename); // Save to SD card
    bool isRecording() { return is_recording; }

    // Quality Metrics
    AudioQuality getLastQuality() { return last_quality; }
    float getCurrentAmplitude();
    float getCurrentRMS();
    float getDominantFrequency();
    float getFrequencyConfidence();
    MultiSNR calculateMultiBandSNR();

    // Filters
    void configureFilters();                  // Sets up the filter parameters
    void enableFiltering(bool enable);
    void adjustFilterParameters(float highpass_freq, float notch1_freq, float notch2_freq);

    // Testing
    void enableFakeDetections(bool enable);

    // Mixer Controls
    void setFilterMix(float filtered_gain, float raw_gain);
    void setInputGain(float gain);

  private:
    // Audio Objects - Core
    AudioInputI2S* audio_input;
    AudioAnalyzeFFT1024* fft;
    AudioAnalyzePeak* peak_detector;
    AudioAnalyzeRMS* rms_detector;
    AudioAnalyzeNoteFrequency* note_freq;
    AudioRecordQueue* recorder;

    // Mixer for flexible signal routing
    AudioMixer4* analysis_mixer;              // Mix filtered/raw before analysis

    // Audio Objects - Filters
    AudioFilterBiquad* highpass_filter;       // Remove rumble, footsteps, wind < 120Hz
    AudioFilterBiquad* notch_filter_600;      // Supress voice formant "chestiness" 600Hz
    AudioFilterBiquad* notch_filter_2000;     // Supress voice intelligibility 2000Hz
    AudioFilterBiquad* highshelf_filter;      // Boost bird frequencies > 4kHz

    // TODO: Voice Activity Detector components
    AudioAnalyzeFFT1024* vad_fft;              // For speech detection (not implemented)
    AudioEffectGain* adaptive_gain;            // Dynamic gain control (not implemented)

    // Recording State
    bool is_recording;
    bool is_initialized;
    bool filtering_enabled;
    uint32_t recording_start_time;
    File recording_file;

    // Detection State
    float noise_floor_rms;
    float noise_floor_peak;
    uint32_t last_detection_time;
    uint32_t last_noise_update;
    bool fake_detections_enabled;

    // Quality Metrics
    AudioQuality last_quality;

    // Voice Detection State
    bool voice_detected;
    float speech_energy_average;

    // Helpers
    void updateNoiseFloor();
    void calibrateDetector();
    bool detectHumanSpeech();
    float calculateSNRFromRMS();
    float calculateSNRFromPeak();
};