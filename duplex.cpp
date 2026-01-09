/******************************************/
/*
  duplex.cpp
  by Gary P. Scavone, 2006-2019

  modified by Faustino Tchouwa and Christoph Windmüller
*/
/******************************************/


/* Important changes:
   - relative_path, write_input_path, write_output_path variables to set the path of the audio files
   - implementation of somefunc_2025.cpp functions
   - inout() and openStream() modifications for output only stream to avoid duplex channel = 0 issue
*/

/* Notes:
   - set main parameters in the main()
*/

/* To-do-list:
   - get the processing time for inout() and processBuffer()
   - change back to the non-blocking double buffer for input (also output if needed) for the report
   - try smaller buffer sizes (256, 128, 64 samples) and smaller number of harmonics
   - what is an optimal buffer size and fft size, should they differ?
   - try simple buffer and compare the performance with double buffer
   - implement circular buffer for input (also output if needed) after the report is done for double buffer
*/


#include "RtAudio.h"
#include <iostream>
#include <cstdlib>
#include <cstring>
#include <algorithm>
#include <atomic>
#include <thread>
#include <chrono>
#include <cmath>
#include <iomanip>

#ifndef PI
#define PI 3.14159265358979323846
#endif


// Declaration of the functions from somefunc_2025.cpp
extern int write_buff_dump(double *buff, const int n_buff, double *buff_dump, const int n_buff_dump, int *ind_dump);
extern double get_process_time();
extern int fft(double *x, double *y, const int m);
extern int ifft(double *x, double *y, const int m);
extern int fftr(double *x, double *y, const int m);
extern int ifftr(double *x, double *y, const int l);
extern int get_nextpow2(int n);
extern int checkm(const int m);
extern double *hanning(double *w, const int leng);
extern char *getmem(int leng, unsigned size);
extern double *dgetmem(int leng);


typedef double MY_TYPE;
#define FORMAT RTAUDIO_FLOAT64


// struct for passing data to callback function inout() and processing function processBuffer()
struct inoutData{
  MY_TYPE* pointer_audioFile;
  size_t playbackIdx_audioFile;
  size_t sizeInFrames_audioFile;

  MY_TYPE* pointer_bufferIn[2]; // double buffer for input
  std::atomic<bool> bufferInReady[2]; // flag to indicate if buffer is ready for processing
  std::atomic<int> writeIdx_bufferIn; // 0 or 1 to indicate which buffer is being written to
  std::atomic<int> readIdx_bufferIn; // 0 or 1 to indicate which buffer is being read from
  MY_TYPE* pointer_bufferOut; // buffer for output, may not necceessary later
  MY_TYPE* pointer_circBufferIn;
  int writeIdx_circBufferIn;
  int readIdx_circBufferIn;
  MY_TYPE* pointer_circBufferAnalysis;
  int samplesSinceLastProcess;
  size_t sizeInFrames_bufferInout;
  size_t sizeInBytes_bufferInout;

  size_t f_s;

  MY_TYPE* pointer_bufferDump_1;
  int writeIdx_bufferDump_1;
  MY_TYPE* pointer_bufferDump_2;
  int writeIdx_bufferDump_2;
  MY_TYPE* pointer_bufferDump_3;
  int writeIdx_bufferDump_3;
  MY_TYPE* pointer_bufferDump_4;
  int writeIdx_bufferDump_4;
  MY_TYPE* pointer_bufferDump_5;
  int writeIdx_bufferDump_5;
  MY_TYPE* pointer_bufferDump_6;
  int writeIdx_bufferDump_6;
  // MY_TYPE* pointer_bufferDump_7;
  // int writeIdx_bufferDump_7;
  size_t sizeInFrames_bufferDump;

  MY_TYPE* pointer_acf;
  MY_TYPE* pointer_f0;
  MY_TYPE* last_f0;

  int n_fft;
  MY_TYPE* pointer_x_real_forFFT;
  MY_TYPE* pointer_x_imag_forFFT;
  MY_TYPE* pointer_hanningWindow_forFFT;
  MY_TYPE* pointer_amplitude_DFT;
  MY_TYPE* pointer_harmonicAmplitudes;
  MY_TYPE* pointer_phase;
  MY_TYPE f0_lastBuffer;
};


// Normalized half autocorrelation function (only positive lags)
// (normalization by acf[0] is important for f0 detection, because the thresholds
// get independent of female/male voice, quiet/loud voice, buffer size)
void acf(const double* signal, int signal_size, double* acf){
  double mean = 0.0;
  for (int i=0; i<signal_size; i++) mean += signal[i];
  mean /= signal_size;

  // Computation of ACF (with removal of mean)
  // signal_size is equal to acf size because we compute only positive lags (0 to signal_size-1)
  for (int i=0; i<signal_size; i++){
    acf[i] = 0;
    for (int j=i; j<signal_size; j++){
      acf[i] += (signal[j]-mean) * (signal[j-i]-mean);
    }
  }

  // Normalization
  double energy = acf[0];
  if (energy > 0.0){
    for (int i = 0; i<signal_size; i++){
      acf[i] /= energy;
    }
  }
}


// Get estimated fundamental frequency f0 through detecting the second highest peak in ACF
double get_f0(const double* acf, int acf_size, int fs, double* last_f0){
    double best_idx = INFINITY; // set to inf to make sure f0 = 0 if no peak is found
    double best_acf = 0.39; // threshold to avoid f0 detection on noise or too small peaks

    bool COR_active = true; // set to true to activate control of range (COR)
    int imin, imax;
    // No control of range
    if (!COR_active){
      imin = 1; // min lag index for peak search
      imax = acf_size - 1; // max lag index for peak search
    }
    // Control of range (chosen vocal range: 70 Hz to 500 Hz)
    if (COR_active){
      imin = std::min(88, acf_size - 1); // max f0 of approx. 500 Hz
      imax = std::min(630, acf_size - 1); // min f0 of approx. 70 Hz
    }

    bool zero_crossing_1 = false;
    bool zero_crossing_2 = false;
    for (int i = 1; i < imax; i++){
      // Waiting for two zero crossings before starting peak detection
      if ((acf[i-1] >= 0) && (acf[i] < 0)) zero_crossing_1 = true;
      if (!zero_crossing_1) continue;
      if ((acf[i-1] <= 0) && (acf[i] > 0)) zero_crossing_2 = true;
      if (!zero_crossing_2) continue;

      if (i < imin) continue; // skip peak detection until imin is reached
      bool is_peak = (acf[i] > acf[i-1]) && (acf[i] > acf[i+1]);
      if (!is_peak) continue;

      if (acf[i] > 1.05 * best_acf){ // avoid small variations around best_acf
          best_acf = acf[i];
          best_idx = i;
      }
    }

    if (best_idx == INFINITY) return 0.0; // no f0 detection
    // T_0 = best_idx * Ts, with Ts = 1/fs
    double f0 = (double) fs / best_idx; // in Hz

    // Ignore f0 jumps of more than 100 Hz (if COR is active)
    if (COR_active){
      if ((abs(f0 - *last_f0) > 100.0) && (*last_f0 != 0.0)){
      f0 = 0.0;
      }
      else {
        *last_f0 = f0;
      }
    }

    return f0;
}


void usage( void ) {
  // Error function in case of incorrect command-line
  // argument specifications
  std::cout << "\nuseage: duplex N fs <iDevice> <oDevice> <iChannelOffset> <oChannelOffset>\n";
  std::cout << "    N = number of channels of audio file (1 channel),\n";
  std::cout << "    fs = sampling frequency of audio file (44100 Hz),\n";
  std::cout << "    iDevice = optional input device index to use (default = 0),\n";
  std::cout << "    oDevice = optional output device index to use (default = 0),\n";
  std::cout << "    iChannelOffset = optional input channel offset (default = 0),\n";
  std::cout << "    oChannelOffset = optional output channel offset (default = 0).\n\n";
  exit( 0 );
}

unsigned int getDeviceIndex( std::vector<std::string> deviceNames, bool isInput = false )
{
  unsigned int i;
  std::string keyHit;
  std::cout << '\n';
  for ( i=0; i<deviceNames.size(); i++ )
    std::cout << "  Device #" << i << ": " << deviceNames[i] << '\n';
  do {
    if ( isInput )
      std::cout << "\nChoose an input device #: ";
    else
      std::cout << "\nChoose an output device #: ";
    std::cin >> i;
  } while ( i >= deviceNames.size() );
  std::getline( std::cin, keyHit );  // used to clear out stdin
  return i;
}


// Processing of the inout buffer
void processBuffer(inoutData* d,  int r){

  // double T_start = get_process_time(); // start time of processBuffer process

  // Circular buffer management for analysis
  // int N = d->sizeInFrames_bufferInout; // size of analysis buffer
  //  for (int i = 0; i < N; i++){
  //   d->pointer_circBufferAnalysis[i] = d->pointer_circBufferIn[(d->writeIdx_circBufferIn - N + d->sizeInFrames_bufferInout + i) % d->sizeInFrames_bufferInout];
  //   // d->readIdx_circBufferIn = ...;
  // }


  // Compute ACF and f0 for the current input buffer
  // (replace with pointer_bufferIn[r] if changing to double buffer mode)
  // (replace with pointer_circBufferIn if changing to circular buffer mode)
  // Downsample for faster processing and low pass filtering before ACF?
  acf(d->pointer_bufferIn[r], d->sizeInFrames_bufferInout, d->pointer_acf);
  double f0 = get_f0(d->pointer_acf, d->sizeInFrames_bufferInout, d->f_s, d->last_f0);
  std::fill_n(d->pointer_f0, d->sizeInFrames_bufferInout, f0);

  if (f0 > 0.0){
    // Prepare data for FFT computation (hanning window, zeropadding of real part, imag part = 0)
    // (replace with pointer_bufferIn[r][i] if changing to double buffer mode)
    // (replace with pointer_circBufferIn[i] if changing to circular buffer mode)
    for (int i = 0; i < d->sizeInFrames_bufferInout; i++){
      // Apply a hanning window to the signal:
      d->pointer_x_real_forFFT[i] = d->pointer_bufferIn[r][i] * d->pointer_hanningWindow_forFFT[i];
    }
    for (int i = d->sizeInFrames_bufferInout; i < d->n_fft; i++){
      d->pointer_x_real_forFFT[i] = 0.0;
    }
    for (int i = 0; i < d->n_fft; i++){
      d->pointer_x_imag_forFFT[i] = 0.0;
    }

    // Compute FFT
    fftr(d->pointer_x_real_forFFT, d->pointer_x_imag_forFFT, d->n_fft);

    // Compute amplitude of DFT (by taking sqrt(re^2 + im^2) and normalizing it)
    for (int i = 0; i < d->n_fft/2 + 1; i++){
      d->pointer_amplitude_DFT[i] = sqrt( (d->pointer_x_real_forFFT[i] * d->pointer_x_real_forFFT[i]) +
                                        (d->pointer_x_imag_forFFT[i] * d->pointer_x_imag_forFFT[i]) );
      if (i == 0 || i == d->n_fft/2){
        d->pointer_amplitude_DFT[i] /= (double) d->n_fft;
      }
      else {
        d->pointer_amplitude_DFT[i] /= ((double) d->n_fft * 0.5);
      }
    }

    // Autotune
    bool autotune_active = false; // set to true to activate autotune
    if (autotune_active){
      double N = 1; // for rounding to N semitones
      double f0_ST = 12 * std::log2(f0);
      f0_ST = round(f0_ST / N) * N;
      f0 = std::pow(2, f0_ST/12.0);
    }

    // Compute the frequency bin and amplitude of each harmonic based on the f0 estimation
    for (int i=0; i < d->n_fft/2 + 1; i++){
      d->pointer_harmonicAmplitudes[i] = 0.0; // reset to 0
    }
    int n_validHarmonics = (int)floor(((double) d->f_s/2) / f0);
    n_validHarmonics = std::min(n_validHarmonics, d->n_fft/2);
    for (int n = 1; n <= n_validHarmonics; n++){
      // frequency bin of the n-th harmonic frequency:
      int kn = (int)round((double) n * f0 * (double) d->n_fft / (double) d->f_s);
      d->pointer_harmonicAmplitudes[n-1] = d->pointer_amplitude_DFT[kn];
    }

    // // Additive synthesis
    // // Possible improvements: frequency interpolation, amplitude interpolation,
    // // lock the phase of all harmonics to f0?
    // for (int i = 0; i < d->sizeInFrames_bufferInout; i++){
    //   // double t = (double) i / (double) d->f_s;
    //   double y = 0.0;
    //   for (int n = 0; n <= n_validHarmonics - 1; n++){
    //     double f = (n + 1) * f0;
    //     double A = d->pointer_harmonicAmplitudes[n];
    //     // y += A * cos(2.0 * PI * f * t);

    //     // Using phase (avoids phase jumps at buffer endings)
    //     // (for additive synthesis without phase: comment out below, uncomment t and y above)
    //     d->pointer_phase[n] += 2.0 * PI * f / (double) d->f_s;
    //     if (d->pointer_phase[n] > 2*PI) d->pointer_phase[n] -= 2*PI;
    //     y += A * cos(d->pointer_phase[n]);
    //   }

    //   // Simple amplitude clipping, replace with min max normalization?
    //   if (y > 1.0) y = 1.0;
    //   if (y < -1.0) y = -1.0;

    //   d->pointer_bufferOut[i] = y;
    // }
  }
  else{
    // If no f0 detected, set everything to 0
    for (int i = 0; i < d->sizeInFrames_bufferInout; i++){
      // Test output=0 and noise output, does noise improve the sound?
      d->pointer_amplitude_DFT[i] = 0.0;
      d->pointer_harmonicAmplitudes[i] = 0.0;
      d->pointer_phase[i] = 0.0; // ? if phase reset wanted when f0=0 or what is good here?
      d->pointer_bufferOut[i] = 0.0;
    }
  }

  // // Update f0_lastBuffer
  // d->f0_lastBuffer = f0; // use it for frequency interpotlation to avoid IF jumps?

  // Write chosen signals in the 'bufferDump' buffers (change here to record other signals)
  // (replace with pointer_bufferIn[r] if changing to double buffer mode)
  // (replace with pointer_circBufferIn if changing to circular buffer mode)
  write_buff_dump(d->pointer_bufferIn[r], d->sizeInFrames_bufferInout, d->pointer_bufferDump_1,
    d->sizeInFrames_bufferDump, &d->writeIdx_bufferDump_1);
  write_buff_dump(d->pointer_acf, d->sizeInFrames_bufferInout, d->pointer_bufferDump_2,
    d->sizeInFrames_bufferDump, &d->writeIdx_bufferDump_2);
  write_buff_dump(d->pointer_f0, d->sizeInFrames_bufferInout, d->pointer_bufferDump_3,
    d->sizeInFrames_bufferDump, &d->writeIdx_bufferDump_3);
  write_buff_dump(d->pointer_amplitude_DFT, d->sizeInFrames_bufferInout, d->pointer_bufferDump_4,
  d->sizeInFrames_bufferDump, &d->writeIdx_bufferDump_4);
  write_buff_dump(d->pointer_harmonicAmplitudes, d->sizeInFrames_bufferInout, d->pointer_bufferDump_5,
  d->sizeInFrames_bufferDump, &d->writeIdx_bufferDump_5);
  write_buff_dump(d->pointer_bufferOut, d->sizeInFrames_bufferInout, d->pointer_bufferDump_6,
  d->sizeInFrames_bufferDump, &d->writeIdx_bufferDump_6);
  // write_buff_dump(d->pointer_bufferOut, d->sizeInFrames_bufferInout, d->pointer_bufferDump_7,
  // d->sizeInFrames_bufferDump, &d->writeIdx_bufferDump_7);

  // Print message when recording is finished
   if (d->writeIdx_bufferDump_3 == d->sizeInFrames_bufferDump){
      std::cout << "\nFinished recording the signals.\n\n";
      d->writeIdx_bufferDump_3++; // to avoid multiple prints
  }



  // double T_end = get_process_time(); // end time of processBuffer process

  // // Time of processBuffer process
  // double T_processBuffer = T_end - T_start;
  // std::cout << "Time needed for processing each buffer: " << std::fixed << std::setprecision(6) << T_processBuffer << " seconds." << std::endl;
}


double streamTimePrintIncrement = 1.0; // seconds
double streamTimePrintTime = 1.0; // seconds

int inout( void *outputBuffer, void * /*inputBuffer*/, unsigned int /*nBufferFrames*/,
           double streamTime, RtAudioStreamStatus status, void *data)
{                                     // inputBuffer not needed for output only stream

  // double T_start = get_process_time(); // start time of inout process
  
  if ( status ) std::cout << "Stream over/underflow detected." << std::endl;

  
  inoutData* d = (inoutData*) data;
  
  // If using the input as output, we can do a simple buffer copy operation here
  // (number of input and output channels is equal)
  // Copy the input buffer to the output buffer
  // memcpy(outputBuffer, inputBuffer, d->sizeInBytes_bufferInout);

  // Cast the void* pointers to double* pointers
  double* outputB = (double*) outputBuffer;
  // double* inputB = (double*) inputBuffer;


  // // Managing input and output buffer (simple buffer implementation)
  // for (unsigned int i=0; i < d->sizeInFrames_bufferInout; i++){
  //   d->pointer_circBufferIn[i] = d->pointer_audioFile[d->playbackIdx_audioFile++];

  //   if (d->playbackIdx_audioFile >= d->sizeInFrames_audioFile){
  //     d->playbackIdx_audioFile = 0;
  //   }
  // }
  // processBuffer(d, 0); // not real time safe if processing time > buffer time

  // for (unsigned int i=0; i < d->sizeInFrames_bufferInout ; i++){
  //   outputB[i] = d->pointer_bufferOut[i];
  // }



  // Managing input and output buffer (NON-BLOCKING, double buffer implementation)
  int w = d->writeIdx_bufferIn.load(std::memory_order_relaxed);
  if (!d->bufferInReady[w].load(std::memory_order_acquire)){
    for (unsigned int i=0; i < d->sizeInFrames_bufferInout; i++){
      d->pointer_bufferIn[w][i] = d->pointer_audioFile[d->playbackIdx_audioFile++];
      outputB[i] = d->pointer_bufferOut[i];
      if (d->playbackIdx_audioFile >= d->sizeInFrames_audioFile){
        d->playbackIdx_audioFile = 0;
      }
    }
    d->bufferInReady[w].store(true, std::memory_order_release);
    d->writeIdx_bufferIn.store(1 - w, std::memory_order_relaxed);
  }
  else {
    for (unsigned int i = 0; i < d->sizeInFrames_bufferInout ; i++){
      outputB[i] = 0; // output 0 if no buffer can be written
    }
  }


  // // Managing input and output buffer (circular buffer implementation)
  // for (unsigned int i=0; i <= d->sizeInFrames_bufferInout - 1; i++){
  //   d->pointer_circBufferIn[d->writeIdx_circBufferIn] = d->pointer_audioFile[d->playbackIdx_audioFile++];
  //   d->writeIdx_circBufferIn = (d->writeIdx_circBufferIn + 1) % d->sizeInFrames_bufferInout;
  //   d->samplesSinceLastProcess++;

  //   outputB[i] = d->pointer_bufferOut[i];

  //   if (d->playbackIdx_audioFile >= d->sizeInFrames_audioFile){
  //     d->playbackIdx_audioFile = 0;
  //   }
  // }

  // double T_end = get_process_time(); // end time of inout process

  // Time of inout process
  // double T_inout = T_end - T_start;

  //  if ( streamTime >= streamTimePrintTime ){
  //   std::cout << "streamTime = " << streamTime << std::endl;

  //   // std::cout << "Time avaible for processing each buffer: " << std::fixed << std::setprecision(6) << T_inout << " seconds." << std::endl;
  //   // std::cout << "Needed time: " << std::fixed << std::setprecision(6) << T_inout << " seconds." << std::endl;

  //   streamTimePrintTime += streamTimePrintIncrement;
  // }

  return 0;
}


int main( int argc, char *argv[] )
{
  unsigned int channels, fs, bufferBytes, oDevice = 0, iDevice = 0, iOffset = 0, oOffset = 0;
  
  // Minimal command-line checking
  if (argc < 3 || argc > 7 ) usage();

  RtAudio adac;
  std::vector<unsigned int> deviceIds = adac.getDeviceIds();
  if ( deviceIds.size() < 1 ) {
    std::cout << "\nNo audio devices found!\n";
    exit( 1 );
  }
  channels = (unsigned int) atoi(argv[1]);
  fs = (unsigned int) atoi(argv[2]);

  if ( argc > 3 )
    iDevice = (unsigned int) atoi(argv[3]);
  if ( argc > 4 )
    oDevice = (unsigned int) atoi(argv[4]);
  if ( argc > 5 )
    iOffset = (unsigned int) atoi(argv[5]);
  if ( argc > 6 )
    oOffset = (unsigned int) atoi(argv[6]);

  // Let RtAudio print messages to stderr.
  adac.showWarnings(true);

  // Set the same number of channels for both input and output.
  RtAudio::StreamParameters iParams, oParams;
  iParams.nChannels = channels;
  iParams.firstChannel = iOffset;
  oParams.nChannels = channels;
  oParams.firstChannel = oOffset;

  if ( iDevice == 0 )
    iParams.deviceId = adac.getDefaultInputDevice();
  else {
    if ( iDevice >= deviceIds.size() )
      iDevice = getDeviceIndex( adac.getDeviceNames(), true );
    iParams.deviceId = deviceIds[iDevice];
  }
  if ( oDevice == 0 )
    oParams.deviceId = adac.getDefaultOutputDevice();
  else {
    if ( oDevice >= deviceIds.size() )
      oDevice = getDeviceIndex( adac.getDeviceNames() );
    oParams.deviceId = deviceIds[oDevice];
  }
  
  RtAudio::StreamOptions options;
  //options.flags |= RTAUDIO_NONINTERLEAVED;



  // Set main parameters

  unsigned int bufferFrames = 1024; // inout buffer size in frames
  // (bufferFrames: tradeoff between smooth f0 and missing short time events)
  int HOP = bufferFrames; // hop size for processing (0% overlap)
  bufferBytes = bufferFrames * channels * sizeof(MY_TYPE);
  double recordingTime = 1.6; // in seconds
  double stopTime = 2; // in seconds

  double T_inout_theoretical = (double) HOP / (double) fs; // time available for processing each buffer
  std::cout << "\nTheoretical time available for processing each buffer: " << T_inout_theoretical << " seconds.\n";



  // Reading the audio file

  // We know from the corresponding wav audio file that the bin audio file has:
  // - format: double, 64 bit, 8 bytes
  // - channels: 1, mono, -> number of frames = number of samples
  // - fs: 44100 Hz

  // const char* relativePath_audioFile = "../../../../audio_files/Tone_220Hz.bin";
  // const char* relativePath_audioFile = "../../../../audio_files/Sweep_80-450Hz.bin";
  const char* relativePath_audioFile = "../../../../audio_files/F01_a3_s100_v04.bin";
  // const char* relativePath_audioFile = "../../../../audio_files/F03_a1_s100_v03.bin";
  // const char* relativePath_audioFile = "../../../../audio_files/M04_a4_s100_v02.bin";
  // const char* relativePath_audioFile = "../../../../audio_files/M07_a1_s100_v03.bin";

  FILE *audioFile = fopen(relativePath_audioFile, "rb");
  if (audioFile == NULL) {
    perror("Error opening file");
    return 1;
  }
  fseek(audioFile, 0 ,SEEK_END);
  long sizeInBytes_audioFile = ftell(audioFile);
  fseek(audioFile, 0, SEEK_SET);
  double sizeInFrames_audioFile = (double) sizeInBytes_audioFile / (sizeof(double));
  MY_TYPE *audioFileRead = (MY_TYPE *)getmem(sizeInFrames_audioFile, sizeof(double));
  if (!audioFileRead) {
    fclose(audioFile);
  }
  size_t n_blocks = (size_t) sizeInFrames_audioFile;
  size_t n_blocksRead = fread(audioFileRead, sizeof(double), n_blocks, audioFile);
  fclose(audioFile);
  if (n_blocksRead != n_blocks) {
        free(audioFileRead);
    }

  inoutData* data = new inoutData();

  data->pointer_audioFile = audioFileRead;
  data->playbackIdx_audioFile = 0;
  data->sizeInFrames_audioFile = n_blocks;

  data->pointer_bufferIn[0] = (MY_TYPE *)getmem(bufferFrames, sizeof(MY_TYPE));
  data->pointer_bufferIn[1] = (MY_TYPE *)getmem(bufferFrames, sizeof(MY_TYPE));
  data->bufferInReady[0].store(false, std::memory_order_relaxed);
  data->bufferInReady[1].store(false, std::memory_order_relaxed);
  data->writeIdx_bufferIn = 0;
  data->readIdx_bufferIn = 0;
  data->pointer_bufferOut = (MY_TYPE *)getmem(bufferFrames, sizeof(MY_TYPE));
  data->pointer_circBufferIn = (MY_TYPE *)getmem(bufferFrames, sizeof(MY_TYPE));
  data->writeIdx_circBufferIn = 0;
  data->readIdx_circBufferIn = 0;
  data->pointer_circBufferAnalysis = (MY_TYPE *)getmem(bufferFrames, sizeof(MY_TYPE));
  data->samplesSinceLastProcess = 0;
  data->sizeInFrames_bufferInout = bufferFrames;
  data->sizeInBytes_bufferInout = bufferBytes;

  data->f_s = fs;
  
  size_t recordingFrames = (size_t) (double) fs * recordingTime; // in frames
  data->pointer_bufferDump_1 = (MY_TYPE *)getmem(recordingFrames, sizeof(MY_TYPE));
  data->writeIdx_bufferDump_1 = 0;
  data->pointer_bufferDump_2 = (MY_TYPE *)getmem(recordingFrames, sizeof(MY_TYPE));
  data->writeIdx_bufferDump_2 = 0;
  data->pointer_bufferDump_3 = (MY_TYPE *)getmem(recordingFrames, sizeof(MY_TYPE));
  data->writeIdx_bufferDump_3 = 0;
  data->pointer_bufferDump_4 = (MY_TYPE *)getmem(recordingFrames, sizeof(MY_TYPE));
  data->writeIdx_bufferDump_4 = 0;
  data->pointer_bufferDump_5 = (MY_TYPE *)getmem(recordingFrames, sizeof(MY_TYPE));
  data->writeIdx_bufferDump_5 = 0;
  data->pointer_bufferDump_6 = (MY_TYPE *)getmem(recordingFrames, sizeof(MY_TYPE));
  data->writeIdx_bufferDump_6 = 0;
  // data->pointer_bufferDump_7 = (MY_TYPE *)getmem(recordingFrames, sizeof(MY_TYPE));
  // data->writeIdx_bufferDump_7 = 0;
  data->sizeInFrames_bufferDump = recordingFrames;

  data->pointer_acf = (MY_TYPE *)getmem(bufferFrames, sizeof(MY_TYPE));
  data->pointer_f0 = (MY_TYPE *)getmem(bufferFrames, sizeof(MY_TYPE));
  data->last_f0 = (MY_TYPE *)getmem(1, sizeof(MY_TYPE));
  *(data->last_f0) = 0.0;

  int n_fft = get_nextpow2(bufferFrames);
  data->n_fft = n_fft;
  data->pointer_x_real_forFFT = (MY_TYPE *)getmem(n_fft, sizeof(MY_TYPE));
  data->pointer_x_imag_forFFT = (MY_TYPE *)getmem(n_fft, sizeof(MY_TYPE));
  data->pointer_hanningWindow_forFFT = (MY_TYPE *)getmem(bufferFrames, sizeof(MY_TYPE));
  data->pointer_amplitude_DFT = (MY_TYPE *)getmem(bufferFrames, sizeof(MY_TYPE));
  data->pointer_harmonicAmplitudes = (MY_TYPE *)getmem(bufferFrames, sizeof(MY_TYPE));
  data->pointer_phase = (MY_TYPE *)getmem(n_fft/2 + 1, sizeof(MY_TYPE));
  data->f0_lastBuffer = 0;


  // Compute the hanning window in advance
  hanning(data->pointer_hanningWindow_forFFT, bufferFrames);

  // Compute FFT once to preallocate the sin table memory before starting the audio stream
  // n_fft stays the same during the whole execution, so the sin table is computed only once here
  fftr(data->pointer_x_real_forFFT, data->pointer_x_imag_forFFT, n_fft);



  if ( adac.openStream( &oParams, NULL /*&iParams*/, FORMAT, fs, &bufferFrames, &inout, (void *)data, &options ) ) {
    goto cleanup; // openStream &iParams changed to NULL to get output only stream to
  }               // avoid the duplex channel = 0 issue

  if ( adac.isStreamOpen() == false ) goto cleanup;

  // Test RtAudio functionality for reporting latency.
  std::cout << "\nStream latency = " << adac.getStreamLatency() << " frames" << std::endl;

  if ( adac.startStream() ) goto cleanup;


  // Just usable if a main loop is not used (blocks the main loop):
  // char input;
  // std::cout << "\nRunning ... press <enter> to quit (bufferFrames = " << bufferFrames << ").\n\n";
  // std::cin.get(input); // it blocks everything until user presses enter, main loop cannot run



  std::cout << "\nRunning... for the next " << stopTime << " seconds.\n(bufferFrames = " << bufferFrames << ")\n\n";


  // Main loop to process buffers when they are ready (double buffer implementation)
  auto startTime = std::chrono::high_resolution_clock::now();
  std::atomic<bool> stopRequested(false); // flag to indicate when to stop
  while (!stopRequested.load(std::memory_order_acquire)) { // check if stop is requested
    int r = data->readIdx_bufferIn.load(std::memory_order_relaxed);
    if (data->bufferInReady[r].load(std::memory_order_acquire)) { // check if new buffer is ready
      processBuffer(data, r); // process the buffer
      data->bufferInReady[r].store(false, std::memory_order_release); // mark buffer as processed
      data->readIdx_bufferIn.store(1 - r, std::memory_order_relaxed);
    }
    std::this_thread::sleep_for(std::chrono::microseconds(200)); // sleep to avoid busy waiting

    auto now = std::chrono::high_resolution_clock::now(); // check elapsed time
    double elapsed = std::chrono::duration<double>(now - startTime).count();
    if (elapsed >= stopTime) stopRequested.store(true); // request stop after stopTime seconds
  }

  
  // // Main loop to process buffers when they are ready (circular buffer implementation)
  // auto startTime = std::chrono::high_resolution_clock::now();
  // std::atomic<bool> stopRequested(false); // flag to indicate when to stop
  // while (!stopRequested.load(std::memory_order_acquire)) { // check if stop is requested
  //   if (data->samplesSinceLastProcess >= HOP) { // check if new buffer is ready
  //     processBuffer(data, 0); // process the buffer
  //     data->samplesSinceLastProcess -= HOP;
  //   }
  //   else {
  //     std::this_thread::sleep_for(std::chrono::microseconds(200)); // sleep to avoid busy waiting
  //   }
  //   auto now = std::chrono::high_resolution_clock::now(); // check elapsed time
  //   double elapsed = std::chrono::duration<double>(now - startTime).count();
  //   if (elapsed >= stopTime) stopRequested.store(true); // request stop after stopTime seconds
  // }



  // Stop the stream
  if ( adac.isStreamRunning() ) adac.stopStream();

 cleanup:
  if ( adac.isStreamOpen() ) adac.closeStream();



  // Writing the recorded signals to binary files
  const char* relativePath_writtenFile_1 = "../../../../python/writtenFile_1.bin";
  FILE *writtenFile_1 = fopen(relativePath_writtenFile_1, "wb");
  size_t n_blocksWritten_1 = fwrite(data->pointer_bufferDump_1, sizeof(double), data->writeIdx_bufferDump_1, writtenFile_1);
  // std::cout<<"Number of written blocks (signal 1): "<< n_blocksWritten_1 <<"\n";
  fclose(writtenFile_1);

  const char* relativePath_writtenFile_2 = "../../../../python/writtenFile_2.bin";
  FILE *writtenFile_2 = fopen(relativePath_writtenFile_2, "wb");
  size_t n_blocksWritten_2 = fwrite(data->pointer_bufferDump_2, sizeof(double), data->writeIdx_bufferDump_2, writtenFile_2);
  // std::cout<<"Number of written blocks (signal 2): "<< n_blocksWritten_2 <<"\n";
  fclose(writtenFile_2);

  const char* relativePath_writtenFile_3 = "../../../../python/writtenFile_3.bin";
  FILE *writtenFile_3 = fopen(relativePath_writtenFile_3, "wb");
  size_t n_blocksWritten_3 = fwrite(data->pointer_bufferDump_3, sizeof(double), data->writeIdx_bufferDump_3, writtenFile_3);
  // std::cout<<"Number of written blocks (signal 3): "<< n_blocksWritten_3 <<"\n";
  fclose(writtenFile_3);

  const char* relativePath_writtenFile_4 = "../../../../python/writtenFile_4.bin";
  FILE *writtenFile_4 = fopen(relativePath_writtenFile_4, "wb");
  size_t n_blocksWritten_4 = fwrite(data->pointer_bufferDump_4, sizeof(double), data->writeIdx_bufferDump_4, writtenFile_4);
  // std::cout<<"Number of written blocks (signal 4): "<< n_blocksWritten_4 <<"\n";
  fclose(writtenFile_4);

  const char* relativePath_writtenFile_5 = "../../../../python/writtenFile_5.bin";
  FILE *writtenFile_5 = fopen(relativePath_writtenFile_5, "wb");
  size_t n_blocksWritten_5 = fwrite(data->pointer_bufferDump_5, sizeof(double), data->writeIdx_bufferDump_5, writtenFile_5);
  // std::cout<<"Number of written blocks (signal 5): "<< n_blocksWritten_5 <<"\n";
  fclose(writtenFile_5);
  
  const char* relativePath_writtenFile_6 = "../../../../python/writtenFile_6.bin";
  FILE *writtenFile_6 = fopen(relativePath_writtenFile_6, "wb");
  size_t n_blocksWritten_6 = fwrite(data->pointer_bufferDump_6, sizeof(double), data->writeIdx_bufferDump_6, writtenFile_6);
  // std::cout<<"Number of written blocks (signal 6): "<< n_blocksWritten_6 <<"\n";
  fclose(writtenFile_6);

  // const char* relativePath_writtenFile_7 = "../../../../python/writtenFile_7.bin";
  // FILE *writtenFile_7 = fopen(relativePath_writtenFile_7, "wb");
  // size_t n_blocksWritten_7 = fwrite(data->pointer_bufferDump_7, sizeof(double), data->writeIdx_bufferDump_7, writtenFile_7);
  // // std::cout<<"Number of written blocks (signal 7): "<< n_blocksWritten_7 <<"\n";
  // fclose(writtenFile_7);

  // Free allocated memory
  free(data->pointer_audioFile);
  free(data->pointer_bufferIn[0]);
  free(data->pointer_bufferIn[1]);
  free(data->pointer_bufferOut);
  free(data->pointer_circBufferIn);
  free(data->pointer_circBufferAnalysis);
  free(data->pointer_bufferDump_1);
  free(data->pointer_bufferDump_2);
  free(data->pointer_bufferDump_3);
  free(data->pointer_bufferDump_4);
  free(data->pointer_bufferDump_5);
  free(data->pointer_bufferDump_6);
  // free(data->pointer_bufferDump_7);
  free(data->pointer_acf);
  free(data->pointer_f0);
  free(data->last_f0);
  free(data->pointer_x_real_forFFT);
  free(data->pointer_x_imag_forFFT);
  free(data->pointer_hanningWindow_forFFT);
  free(data->pointer_amplitude_DFT);
  free(data->pointer_harmonicAmplitudes);
  free(data->pointer_phase);
  delete data;

  std::cout << "\nStopped running on purpose.\n";
  std::cout << "(recordingTime = " << recordingTime << " seconds)\n";
  std::cout << "(stopTime = " << stopTime << " seconds)\n";

  return 0;
}