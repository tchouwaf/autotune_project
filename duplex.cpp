/******************************************/
/*
  duplex.cpp
  by Gary P. Scavone, 2006-2019.

  This program opens a duplex stream and passes
  input directly through to the output.
*/
/******************************************/

#include "RtAudio.h"
#include <iostream>
#include <cstdlib>
#include <cstring>
#include <algorithm>

typedef double MY_TYPE;
#define FORMAT RTAUDIO_FLOAT64

struct InOutData{
  unsigned int data_size;
  double buffer_size;
  double* buffer;
  size_t index;

  double* dump_input;
  size_t dumpInput_idx;
  
  double* dump_output;
  size_t dumpOutput_idx;

  size_t dump_size;
};

int write_buff_dump(double* buff, const int n_buff, double* buff_dump, const int n_buff_dump, int* ind_dump) {

  int i = 0;
  for (i = 0; i < n_buff; i++) 
  {
    if (*ind_dump < n_buff_dump) 
    {
      buff_dump[*ind_dump] = buff[i];
      (*ind_dump)++;
    } else 
    {
      break;
    }
  }

  return i;
}

double* acf(double* signal, int signal_size){
  int acf_size = 2*signal_size-1;
  double* acf_function = (double*)calloc(acf_size, sizeof(double));

  for(int i=0; i<acf_size; i++){
    for(int j=i; j<signal_size; j++){
      acf_function[i]+=signal[j]*signal[j-i];
    }
  }
  return acf_function;
}

double get_max(double* acf_function, int acf_size){
  double max_1 = acf_function[0];
  int idx_max_1 = 0;

  double max_2 = acf_function[1];
  int idx_max_2 = 1;

  for(int i=1; i<acf_size-1; i++){
    if ((acf_function[i]-acf_function[i-1]>0) && (acf_function[i+1]-acf_function[i])<0){

      if((acf_function[i]>=max_2) && (acf_function[i]>=max_1)){
        max_2 = max_1;
        idx_max_2 = idx_max_1;
        max_1 = acf_function[i];
        idx_max_1 = i;
      }

      if((acf_function[i]>=max_2) && (acf_function[i]<max_1)){
        max_2 = acf_function[i];
        idx_max_2 = i;
      }
    } 
  }

  int T_0 = std::abs(idx_max_2-idx_max_1);
  if(T_0 == 0){
    return 0;
  }

  return 1/T_0; //Multiply it by the sampling frequency
}

void usage( void ) {
  // Error function in case of incorrect command-line
  // argument specifications
  std::cout << "\nuseage: duplex N fs <iDevice> <oDevice> <iChannelOffset> <oChannelOffset>\n";
  std::cout << "    where N = number of channels,\n";
  std::cout << "    fs = the sample rate,\n";
  std::cout << "    iDevice = optional input device index to use (default = 0),\n";
  std::cout << "    oDevice = optional output device index to use (default = 0),\n";
  std::cout << "    iChannelOffset = an optional input channel offset (default = 0),\n";
  std::cout << "    and oChannelOffset = optional output channel offset (default = 0).\n\n";
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

int inout( void *outputBuffer, void *inputBuffer, unsigned int /*nBufferFrames*/,
           double streamTime, RtAudioStreamStatus status, void *data)
{
  // Since the number of input and output channels is equal, we can do
  // a simple buffer copy operation here.
  if ( status ) std::cout << "Stream over/underflow detected." << std::endl;

  if ( streamTime >= streamTimePrintTime ) {
    std::cout << "streamTime = " << streamTime << std::endl;
    streamTimePrintTime += streamTimePrintIncrement;
  }
 
  // memcpy( outputBuffer, inputBuffer, *bytes );
  double* outputB = (double*) outputBuffer;
  double* inputB = (double*) inputBuffer;

  InOutData* my_data = (InOutData*) data;
  double n_samples = my_data->buffer_size;

  unsigned int bytes = my_data->data_size;
  unsigned int len = bytes/sizeof(double);

  // if(my_data->dumpInput_idx < my_data->dump_size){
  //   my_data->dump_input[my_data->dumpInput_idx++] = inputB[0];
  // }

  // if(my_data->dumpOutput_idx < my_data->dump_size){
  //   my_data->dump_output[my_data->dumpOutput_idx++] = outputB[0];
  // }

  for(unsigned int i=0; i<len; i++){
    if(my_data->index >= n_samples){
      my_data->index = 0;
    }
    inputB[i] = my_data->buffer[my_data->index++];
  }
  
  for(unsigned int i=0; i< len; i++){
    outputB[i] = inputB[i];
  }

  int dump__input_size = (int)(my_data->dumpInput_idx);
  int the_size_input = write_buff_dump(inputB, my_data->buffer_size, my_data->dump_input, my_data->dump_size, &dump__input_size);

  int dump__output_size = (int)(my_data->dumpOutput_idx);
  int the_size_output = write_buff_dump(outputB, my_data->buffer_size, my_data->dump_output, my_data->dump_size, &dump__output_size);

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
  adac.showWarnings( true );

  // Set the same number of channels for both input and output.
  unsigned int bufferFrames = 512;
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

  bufferBytes = bufferFrames * channels * sizeof( MY_TYPE );
  InOutData data;
  data.data_size = bufferBytes;


  ////// Reading our audio files ///////

  const char* relative_path = "/net/etudiant/home/tchouwaf/Documents/BE_tstr_2025_v1/BE_tstr_2025_v1/audio_files/F01_a3_s100_v04.bin";
  FILE *audio_file = fopen(relative_path, "rb");

  if (audio_file == NULL) {
    perror("Error opening file");
    return 1;
  }

  fseek(audio_file, 0 ,SEEK_END);
  long size = ftell(audio_file);
  fseek(audio_file, 0, SEEK_SET);
  
  double nSamples = (double) size/sizeof(double);

  double *buffer = (double *)malloc(nSamples*sizeof(double));
  if (!buffer) {
    fclose(audio_file);
  }
  
  size_t readed_blocks = fread(buffer, sizeof(double), nSamples, audio_file);
  fclose(audio_file);
  
  if (readed_blocks != nSamples) {
        free(buffer);
    }

  if ( adac.openStream( &oParams, &iParams, FORMAT, fs, &bufferFrames, &inout, (void *)&data, &options ) ) {
    goto cleanup;
  }

  data.dump_input = (double *)calloc(nSamples, sizeof(double));
  data.dumpInput_idx = 0;

  data.dump_output = (double *)calloc(nSamples, sizeof(double));
  data.dumpOutput_idx = 0;

  data.dump_size = nSamples;

  data.buffer_size = nSamples;
  data.buffer = buffer;


  if ( adac.isStreamOpen() == false ) goto cleanup;

  // Test RtAudio functionality for reporting latency.
  std::cout << "\nStream latency = " << adac.getStreamLatency() << " frames" << std::endl;

  if ( adac.startStream() ) goto cleanup;

  char input;
  std::cout << "\nRunning ... press <enter> to quit (buffer frames = " << bufferFrames << ").\n";
  std::cin.get(input);

  // Stop the stream.
  if ( adac.isStreamRunning() )
    adac.stopStream();

 cleanup:
  if ( adac.isStreamOpen() ) adac.closeStream();

  const char* write_input_path = "/net/etudiant/home/tchouwaf/Documents/BE_tstr_2025_v1/BE_tstr_2025_v1/audio_files/write_input.bin";
  FILE *audio_input_writed = fopen(write_input_path, "wb");
  size_t written_input = fwrite(data.dump_input, sizeof(double), data.dumpInput_idx, audio_input_writed);
  fclose(audio_input_writed);

  free(data.dump_input);

  const char* write_output_path = "/net/etudiant/home/tchouwaf/Documents/BE_tstr_2025_v1/BE_tstr_2025_v1/audio_files/write_output.bin";
  FILE *audio_ouput_writed = fopen(write_output_path, "wb");
  size_t written_output = fwrite(data.dump_output, sizeof(double), data.dumpOutput_idx, audio_ouput_writed);
  fclose(audio_ouput_writed);

  free(data.dump_output);

  return 0;
}
