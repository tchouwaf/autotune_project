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


#include "RtAudio.h"
#include <iostream>
#include <cstdlib>
#include <cstring>
#include <algorithm>
#include <atomic>
#include <thread>
#include <chrono>
#include <cmath>

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
  size_t sizeInFrames_bufferDump;

  MY_TYPE* pointer_acf;
  MY_TYPE* pointer_f0;

  int n_fft;
  MY_TYPE* pointer_x_real_forFFT;
  MY_TYPE* pointer_x_imag_forFFT;
  MY_TYPE* pointer_hanningWindow_forFFT;
  MY_TYPE* pointer_amplitude_DFT;
  MY_TYPE* pointer_frequency_DFT;
  MY_TYPE* pointer_harmonicAmplitudes;
  MY_TYPE* pointer_phase;
  MY_TYPE f0_lastBuffer;
};


// Half autocorrelation function (only positive lags)
void acf(double* signal, int signal_size, double* acf){
  // signal_size is equal to acf size because we compute only positive lags (0 to signal_size-1)
  for(int i=0; i<signal_size; i++){
    acf[i] = 0;
    for(int j=i; j<signal_size; j++){
      acf[i]+=signal[j]*signal[j-i];
    }
  }
}


// Get estimated fundamental frequency f0 through detecting the second highest peak in ACF
double get_f0(const double* acf, int acf_size, int fs){
    double best_idx = INFINITY; // set to inf to make sure f0 = 0 if no peak is found
    double best_acf = 0.25; // threshold to avoid f0 detection on noise or too small peaks

    // for (int i = 1; i < acf_size - 1; i++){
            // set i=1 to skip the maximum at lag 0 and to avoid out-of-bounds access
    int imin = 80;
    int imax = std::min(275, acf_size - 2); // avoid out-of-bounds access if buffer size changed
    for (int i = imin; i < imax; i++){
        // look at a specific range of i to avoid missdetecting f0 at
        // high and low frequencies (chosen vocal range: 160 Hz < f < 550 Hz, for fs = 44100 Hz)
        bool is_peak = (acf[i] > acf[i-1]) && (acf[i] > acf[i+1]);

        if (!is_peak) continue;

        if (acf[i] > best_acf){
            best_acf = acf[i];
            best_idx = i;
        }
    }
    if (best_idx == INFINITY) return 0.0; // no f0 detected
    // T_0 = best_idx * Ts, with Ts = 1/fs
    double f0 = (double) fs / best_idx; // in Hz
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

double streamTimePrintIncrement = 1.0; // seconds
double streamTimePrintTime = 1.0; // seconds


int inout( void *outputBuffer, void * /*inputBuffer*/, unsigned int /*nBufferFrames*/,
           double streamTime, RtAudioStreamStatus status, void *data)
{                                     // inputBuffer not needed for output only stream

  if ( status ) std::cout << "Stream over/underflow detected." << std::endl;

  if ( streamTime >= streamTimePrintTime ) {
    std::cout << "streamTime = " << streamTime << std::endl;
    streamTimePrintTime += streamTimePrintIncrement;
  }
  
  inoutData* d = (inoutData*) data;
  
  // If using the input as output, we can do a simple buffer copy operation here
  // (number of input and output channels is equal)
  // Copy the input buffer to the output buffer
  // memcpy(outputBuffer, inputBuffer, d->sizeInBytes_bufferInout);

  // Cast the void* pointers to double* pointers
  double* outputB = (double*) outputBuffer;
  // double* inputB = (double*) inputBuffer;


  // // Managing input and output buffer (NON-BLOCKING, double buffer implementation)
  // int w = d->writeIdx_bufferIn.load(std::memory_order_relaxed);
  // if (!d->bufferInReady[w].load(std::memory_order_acquire)){
  //   for(unsigned int i=0; i < d->sizeInFrames_bufferInout; i++){
  //     d->pointer_bufferIn[w][i] = d->pointer_audioFile[d->playbackIdx_audioFile++];
  //     outputB[i] = d->pointer_bufferOut[i];
  //     if (d->playbackIdx_audioFile >= d->sizeInFrames_audioFile){
  //       d->playbackIdx_audioFile = 0;
  //     }
  //   }
  //   d->bufferInReady[w].store(true, std::memory_order_release);
  //   d->writeIdx_bufferIn.store(1 - w, std::memory_order_relaxed);
  // }
  // else {
  //   for (unsigned int i = 0; i < d->sizeInFrames_bufferInOut ; i++){
  //     outputB[i] = 0; // output 0 if no buffer can be written
  //   }
  // }


  // // Managing input and output buffer (circular buffer implementation)
  for(unsigned int i=0; i <= d->sizeInFrames_bufferInout - 1; i++){
    d->pointer_circBufferIn[d->writeIdx_circBufferIn] = d->pointer_audioFile[d->playbackIdx_audioFile++];
    d->writeIdx_circBufferIn = (d->writeIdx_circBufferIn + 1) % d->sizeInFrames_bufferInout;

    outputB[i] = d->pointer_bufferOut[i];

    if (d->playbackIdx_audioFile >= d->sizeInFrames_audioFile){
      d->playbackIdx_audioFile = 0;
    }
  }

  return 0;
}


// Processing of the inout buffer
void processBuffer(inoutData* d,  int r){

  int N = 512; // size of analysis buffer
   for (int i = 0; i < N; i++){
    d->pointer_circBufferAnalysis[i] = d->pointer_circBufferIn[(d->writeIdx_circBufferIn - N + d->sizeInFrames_bufferInout + i) % d->sizeInFrames_bufferInout];
    // d->readIdx_circBufferIn = ...;
  }

  // Compute ACF and f0 for the current input buffer
  // (replace with pointer_bufferIn[r] if changing to double buffer mode)
  acf(d->pointer_circBufferAnalysis, d->sizeInFrames_bufferInout, d->pointer_acf);
  double f0 = get_f0(d->pointer_acf, d->sizeInFrames_bufferInout, d->f_s);
  std::fill_n(d->pointer_f0, d->sizeInFrames_bufferInout, f0);

  // Prepare data for FFT computation (zeropadding of real part and imag part = 0)
  // (replace with pointer_bufferIn[r][i] if changing to double buffer mode)
  for (int i = 0; i < d->sizeInFrames_bufferInout; i++){
    d->pointer_x_real_forFFT[i] = d->pointer_circBufferAnalysis[i];
  }
  for (int i = d->sizeInFrames_bufferInout; i < d->n_fft; i++){
    d->pointer_x_real_forFFT[i] = 0.0;
  }
  for (int i = 0; i < d->n_fft; i++){
    d->pointer_x_imag_forFFT[i] = 0.0;
  }

  // Apply a hanning window to the signal
  for (int i = 0; i < d->sizeInFrames_bufferInout; i++){
    d->pointer_x_real_forFFT[i] *= d->pointer_hanningWindow_forFFT[i];
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

  // Autotune (comment out for no autotune)
  // double N = 1; // for rounding to N semitones
  // double f0_ST = 12 * std::log2(f0);
  // f0_ST = round(f0_ST / N) * N;
  // f0 = std::pow(2, f0_ST/12.0);

  // Compute the frequency bin and amplitude of each harmonic based on the f0 estimation
  for (int i=0; i < d->n_fft/2 + 1; i++){
    d->pointer_harmonicAmplitudes[i] = 0.0; // reset to 0
  }
  int n_validHarmonics = (int)floor(((double) d->f_s/2) / f0);
  n_validHarmonics = std::min(n_validHarmonics, d->n_fft/2);
  if (f0 > 0.0){ // if no f0 detected, skip harmonic extraction
    for (int n = 1; n <= n_validHarmonics; n++){
    // frequency bin of the n-th harmonic frequency:
    int kn = (int)round((double) n * f0 * (double) d->n_fft / (double) d->f_s);
    d->pointer_harmonicAmplitudes[n-1] = d->pointer_amplitude_DFT[kn];
    }
  }

  // Additive synthesis
  // Possible improvements: frequency interpolation, amplitude interpolation,
  // lock the phase of all harmonics to f0?
  if (f0 > 0.0){ // if no f0 detected, skip additive synthesis
    for (int i = 0; i < d->sizeInFrames_bufferInout; i++){
      // double t = (double) i / (double) d->f_s;
      double y = 0.0;
      for (int n = 0; n <= n_validHarmonics - 1; n++){
          double f = (n + 1) * f0;
          double A = d->pointer_harmonicAmplitudes[n];
          // y += A * cos(2.0 * PI * f * t);

          // Using phase (avoids phase jumps at buffer endings)
          // (for additive synthesis without phase: comment out below, uncomment t and y above)
          d->pointer_phase[n] += 2.0 * PI * f / (double) d->f_s;
          if (d->pointer_phase[n] > 2*PI) d->pointer_phase[n] -= 2*PI;
          y += A * cos(d->pointer_phase[n]);
      }

      // Simple amplitude clipping 
      if (y > 1.0) y = 1.0;
      if (y < -1.0) y = -1.0;

      d->pointer_bufferOut[i] = y;
    }
  }
  else {
    for (int i = 0; i < d->sizeInFrames_bufferInout; i++){
      d->pointer_bufferOut[i] = 0.0;
    }
  }

  // Update f0_lastBuffer
  d->f0_lastBuffer = f0; // use it for frequency interpotlation to avoid IF jumps?

  // Write chosen signals in the 'bufferDump' buffers (change here to record other signals)
  // (replace with pointer_bufferIn[r] if changing to double buffer mode)
  write_buff_dump(d->pointer_circBufferAnalysis, d->sizeInFrames_bufferInout, d->pointer_bufferDump_1,
    d->sizeInFrames_bufferDump, &d->writeIdx_bufferDump_1);
  write_buff_dump(d->pointer_acf, d->sizeInFrames_bufferInout, d->pointer_bufferDump_2,
    d->sizeInFrames_bufferDump, &d->writeIdx_bufferDump_2);
  write_buff_dump(d->pointer_f0, d->sizeInFrames_bufferInout, d->pointer_bufferDump_3,
    d->sizeInFrames_bufferDump, &d->writeIdx_bufferDump_3);
  write_buff_dump(d->pointer_bufferOut, d->sizeInFrames_bufferInout, d->pointer_bufferDump_4,
  d->sizeInFrames_bufferDump, &d->writeIdx_bufferDump_4);

  // Print message when recording is finished
  if (d->writeIdx_bufferDump_1 == d->sizeInFrames_bufferDump){
      std::cout << "\nFinished recording signal 1.";
      d->writeIdx_bufferDump_1++; // to avoid multiple prints
  }
  if (d->writeIdx_bufferDump_2 == d->sizeInFrames_bufferDump){
      std::cout << "\nFinished recording signal 2.";
      d->writeIdx_bufferDump_2++; // to avoid multiple prints
  }
  if (d->writeIdx_bufferDump_3 == d->sizeInFrames_bufferDump){
      std::cout << "\nFinished recording signal 3.";
      d->writeIdx_bufferDump_3++; // to avoid multiple prints
  }
   if (d->writeIdx_bufferDump_4 == d->sizeInFrames_bufferDump){
      std::cout << "\nFinished recording signal 4.\n\n";
      d->writeIdx_bufferDump_4++; // to avoid multiple prints
  }
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

  unsigned int bufferFrames = 2048; // inout buffer size in frames
  // (bufferFrames: tradeoff between smooth f0 and missing short time events)
  bufferBytes = bufferFrames * channels * sizeof(MY_TYPE);
  size_t recordingTime = 6; // in seconds
  double stopTime = 8; // in seconds



  // Reading the audio file

  // We know from the corresponding wav audio file that the bin audio file has:
  // - format: double, 64 bit, 8 bytes
  // - channels: 1, mono, -> number of frames = number of samples
  // - fs: 44100 Hz

  const char* relativePath_audioFile = "../../../../audio_files/F01_a3_s100_v04.bin";
  // const char* relativePath_audioFile = "/net/etudiant/home/tchouwaf/Documents/BE_tstr_2025_v1/BE_tstr_2025_v1/audio_files/F01_a3_s100_v04.bin";
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
  data->sizeInFrames_bufferInout = bufferFrames;
  data->sizeInBytes_bufferInout = bufferBytes;

  data->f_s = fs;
  
  size_t recordingFrames = (size_t) fs * recordingTime; // in frames
  data->pointer_bufferDump_1 = (MY_TYPE *)getmem(recordingFrames, sizeof(MY_TYPE));
  data->writeIdx_bufferDump_1 = 0;
  data->pointer_bufferDump_2 = (MY_TYPE *)getmem(recordingFrames, sizeof(MY_TYPE));
  data->writeIdx_bufferDump_2 = 0;
  data->pointer_bufferDump_3 = (MY_TYPE *)getmem(recordingFrames, sizeof(MY_TYPE));
  data->writeIdx_bufferDump_3 = 0;
  data->pointer_bufferDump_4 = (MY_TYPE *)getmem(recordingFrames, sizeof(MY_TYPE));
  data->writeIdx_bufferDump_4 = 0;
  data->sizeInFrames_bufferDump = recordingFrames;

  data->pointer_acf = (MY_TYPE *)getmem(bufferFrames, sizeof(MY_TYPE));
  data->pointer_f0 = (MY_TYPE *)getmem(bufferFrames, sizeof(MY_TYPE));

  int n_fft = get_nextpow2(bufferFrames);
  data->n_fft = n_fft;
  data->pointer_x_real_forFFT = (MY_TYPE *)getmem(n_fft, sizeof(MY_TYPE));
  data->pointer_x_imag_forFFT = (MY_TYPE *)getmem(n_fft, sizeof(MY_TYPE));
  data->pointer_hanningWindow_forFFT = (MY_TYPE *)getmem(bufferFrames, sizeof(MY_TYPE));
  data->pointer_amplitude_DFT = (MY_TYPE *)getmem(n_fft/2 + 1, sizeof(MY_TYPE));
  data->pointer_frequency_DFT = (MY_TYPE *)getmem(n_fft/2 + 1, sizeof(MY_TYPE));
  data->pointer_harmonicAmplitudes = (MY_TYPE *)getmem(n_fft/2 + 1, sizeof(MY_TYPE));
  data->pointer_phase = (MY_TYPE *)getmem(n_fft/2 + 1, sizeof(MY_TYPE));
  data->f0_lastBuffer = 0;


  // Compute the hanning window in advance
  hanning(data->pointer_hanningWindow_forFFT, bufferFrames);

  // Compute FFT once to preallocate the sin table memory before starting the audio stream
  // n_fft stays the same during the whole execution, so the sin table is computed only once here
  fftr(data->pointer_x_real_forFFT, data->pointer_x_imag_forFFT, n_fft);

  // Compute the frequency bins of DFT in advance
  for (int i = 0; i < n_fft/2 + 1; i++){
    data->pointer_frequency_DFT[i] = (double) i * (double) fs / (double) n_fft;
  }



  if ( adac.openStream( &oParams, NULL /*&iParams*/, FORMAT, fs, &bufferFrames, &inout, (void *)data, &options ) ) {
    goto cleanup; // openStream &iParams changed to NULL to get output only stream to
  }               // avoid the duplex channel = 0 issue

  if ( adac.isStreamOpen() == false ) goto cleanup;

  // Test RtAudio functionality for reporting latency.
  std::cout << "\nStream latency = " << adac.getStreamLatency() << " frames" << std::endl;

  if ( adac.startStream() ) goto cleanup;

  // char input;
  // std::cout << "\nRunning ... press <enter> to quit (bufferFrames = " << bufferFrames << ").\n";
  // std::cin.get(input); // it blocks everything until user presses enter, main loop cannot run

  std::cout << "\nRunning... for the next " << stopTime << " seconds.\n(bufferFrames = " << bufferFrames << ")\n\n";



  // // Main loop to process buffers when they are ready (double buffer implementation)
  // auto startTime = std::chrono::high_resolution_clock::now();
  // std::atomic<bool> stopRequested(false); // flag to indicate when to stop
  // while (!stopRequested.load(std::memory_order_acquire)) { // check if stop is requested
  //   int r = data->readIdx_bufferIn.load(std::memory_order_relaxed);
  //   if (data->bufferInReady[r].load(std::memory_order_acquire)) { // check if new buffer is ready
  //     processBuffer(data, r); // process the buffer
  //     data->bufferInReady[r].store(false, std::memory_order_release); // mark buffer as processed
  //     data->readIdx_bufferIn.store(1 - r, std::memory_order_relaxed);
  //   }
  //   std::this_thread::sleep_for(std::chrono::microseconds(200)); // sleep to avoid busy waiting

  //   auto now = std::chrono::high_resolution_clock::now(); // check elapsed time
  //   double elapsed = std::chrono::duration<double>(now - startTime).count();
  //   if (elapsed >= stopTime) stopRequested.store(true); // request stop after stopTime seconds
  // }

  
  // Main loop to process buffers when they are ready
  auto startTime = std::chrono::high_resolution_clock::now();
  std::atomic<bool> stopRequested(false); // flag to indicate when to stop
  while (!stopRequested.load(std::memory_order_acquire)) { // check if stop is requested
    
    processBuffer(data, 0); // process the buffer

    std::this_thread::sleep_for(std::chrono::microseconds(200)); // sleep to avoid busy waiting

    auto now = std::chrono::high_resolution_clock::now(); // check elapsed time
    double elapsed = std::chrono::duration<double>(now - startTime).count();
    if (elapsed >= stopTime) stopRequested.store(true); // request stop after stopTime seconds
  }



  // Stop the stream
  if ( adac.isStreamRunning() ) adac.stopStream();

 cleanup:
  if ( adac.isStreamOpen() ) adac.closeStream();



  // Writing the recorded signals to binary files
  const char* relativePath_writtenFile_1 = "../../../../python/writtenFile_1.bin";
  // const char* relativePath_writtenFile_1 = "/net/etudiant/home/tchouwaf/Documents/BE_tstr_2025_v1/BE_tstr_2025_v1/audio_files/writtenFile_1.bin";
  FILE *writtenFile_1 = fopen(relativePath_writtenFile_1, "wb");
  size_t n_blocksWritten_1 = fwrite(data->pointer_bufferDump_1, sizeof(double), data->writeIdx_bufferDump_1, writtenFile_1);
  // std::cout<<"Number of written blocks (signal 1): "<< n_blocksWritten_1 <<"\n";
  fclose(writtenFile_1);

  const char* relativePath_writtenFile_2 = "../../../../python/writtenFile_2.bin";
  // const char* relativePath_writtenFile_2 = "/net/etudiant/home/tchouwaf/Documents/BE_tstr_2025_v1/BE_tstr_2025_v1/audio_files/writtenFile_2.bin";
  FILE *writtenFile_2 = fopen(relativePath_writtenFile_2, "wb");
  size_t n_blocksWritten_2 = fwrite(data->pointer_bufferDump_2, sizeof(double), data->writeIdx_bufferDump_2, writtenFile_2);
  // std::cout<<"Number of written blocks (signal 2): "<< n_blocksWritten_2 <<"\n";
  fclose(writtenFile_2);

  const char* relativePath_writtenFile_3 = "../../../../python/writtenFile_3.bin";
  // const char* relativePath_writtenFile_3 = "/net/etudiant/home/tchouwaf/Documents/BE_tstr_2025_v1/BE_tstr_2025_v1/audio_files/writtenFile_3.bin";
  FILE *writtenFile_3 = fopen(relativePath_writtenFile_3, "wb");
  size_t n_blocksWritten_3 = fwrite(data->pointer_bufferDump_3, sizeof(double), data->writeIdx_bufferDump_3, writtenFile_3);
  // std::cout<<"Number of written blocks (signal 3): "<< n_blocksWritten_3 <<"\n";
  fclose(writtenFile_3);

  const char* relativePath_writtenFile_4 = "../../../../python/writtenFile_4.bin";
  // const char* relativePath_writtenFile_4 = "/net/etudiant/home/tchouwaf/Documents/BE_tstr_2025_v1/BE_tstr_2025_v1/audio_files/writtenFile_4.bin";
  FILE *writtenFile_4 = fopen(relativePath_writtenFile_4, "wb");
  size_t n_blocksWritten_4 = fwrite(data->pointer_bufferDump_4, sizeof(double), data->writeIdx_bufferDump_4, writtenFile_4);
  // std::cout<<"Number of written blocks (signal 4): "<< n_blocksWritten_4 <<"\n";
  fclose(writtenFile_4);

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
  free(data->pointer_acf);
  free(data->pointer_f0);
  free(data->pointer_x_real_forFFT);
  free(data->pointer_x_imag_forFFT);
  free(data->pointer_hanningWindow_forFFT);
  free(data->pointer_amplitude_DFT);
  free(data->pointer_frequency_DFT);
  free(data->pointer_harmonicAmplitudes);
  delete data;

  std::cout << "\nStopped running on purpose.\n";
  std::cout << "(recordingTime = " << recordingTime << " seconds)\n";
  std::cout << "(stopTime = " << stopTime << " seconds)\n";

  return 0;
}