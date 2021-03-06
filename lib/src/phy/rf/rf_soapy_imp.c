/**
 *
 * \section COPYRIGHT
 *
 * Copyright 2013-2015 Software Radio Systems Limited
 *
 * \section LICENSE
 *
 * This file is part of the srsLTE library.
 *
 * srsLTE is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Affero General Public License as
 * published by the Free Software Foundation, either version 3 of
 * the License, or (at your option) any later version.
 *
 * srsLTE is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Affero General Public License for more details.
 *
 * A copy of the GNU Affero General Public License can be found in
 * the LICENSE file in the top-level directory of this distribution
 * and at http://www.gnu.org/licenses/.
 *
 */


#include <sys/time.h>
#include <string.h>
#include <unistd.h>
#include <pthread.h>

#include "srslte/srslte.h"
#include "rf_soapy_imp.h"
#include "srslte/phy/rf/rf.h"

#include <SoapySDR/Device.h>
#include <SoapySDR/Formats.h>

typedef struct rf_soapy_handler{
    char *devname;
    SoapySDRKwargs args;
    SoapySDRDevice *device;
    SoapySDRRange *ranges;
    SoapySDRStream *rxStream;
    SoapySDRStream *txStream;
    bool tx_stream_active;
    bool rx_stream_active;
    int tx_antnum;  //0,1,2
    int rx_antnum;  //0,1,2
    size_t *tx_ants; //{0},{1},{0,1}
    size_t *rx_ants; //{1},{1},{0,1}
    struct rf_soapy_handler * next; //apply for multi-device
} rf_soapy_handler_t;


cf_t zero_mem[64*1024];


int soapy_error(void *h)
{
  return 0;
}


void rf_soapy_get_freq_range(void *h)
{
  printf("rf_soapy_get_freq_range not supported\n");
}


void rf_soapy_suppress_handler(const char *x)
{
  printf("rf_soapy_suppress_handler not supported\n");
    // not supported
}


void rf_soapy_msg_handler(const char *msg)
{
  printf("rf_soapy_msg_handler not supported\n");
    // not supported
}


void rf_soapy_suppress_stdout(void *h)
{
  printf("rf_soapy_suppress_stdout not supported\n");
    // not supported
}


void rf_soapy_register_error_handler(void *notused, srslte_rf_error_handler_t new_handler)
{
  printf("rf_soapy_register_error_handler not supported\n");
    // not supported
}


char* rf_soapy_devname(void* h)
{
  rf_soapy_handler_t *handler = (rf_soapy_handler_t*) h;
  return handler->devname; 
}


bool rf_soapy_rx_wait_lo_locked(void *h)
{
  rf_soapy_handler_t *handler = (rf_soapy_handler_t*)h;
  char *ret = SoapySDRDevice_readChannelSensor(handler->device, SOAPY_SDR_RX, 0, "lo_locked");
  if (ret != NULL) {
    return (strcmp(ret, "true") == 0 ? true : false);
  }
  return true;
}


void rf_soapy_set_tx_cal(void *h, srslte_rf_cal_t *cal)
{
  printf("rf_soapy_set_tx_cal not supported\n");
  printf("TODO: implement rf_soapy_rx_wait_lo_locked()\n");
  // not supported
}


void rf_soapy_set_rx_cal(void *h, srslte_rf_cal_t *cal)
{
  printf("rf_soapy_set_rx_cal not supported\n");
  printf("TODO: implement rf_soapy_set_rx_cal()\n");
}


int rf_soapy_start_rx_stream(void *h, bool now) //multi device
{
  rf_soapy_handler_t *handler = (rf_soapy_handler_t*) h;
  while (handler!=NULL){
    if(handler->rx_stream_active == false){
      if(SoapySDRDevice_activateStream(handler->device, handler->rxStream, 0, 0, 0) != 0)
        return SRSLTE_ERROR;
      handler->rx_stream_active = true;
    }
    handler = handler->next;
 }
  return SRSLTE_SUCCESS;
}


int rf_soapy_start_tx_stream(void *h)
{
  rf_soapy_handler_t *handler = (rf_soapy_handler_t*) h;
  if(handler->tx_stream_active == false){
    if(SoapySDRDevice_activateStream(handler->device, handler->txStream, 0, 0, 0) != 0)
      return SRSLTE_ERROR;
    handler->tx_stream_active = true;  
   }
  return SRSLTE_SUCCESS;
}


int rf_soapy_stop_rx_stream(void *h) //multi device
{
  rf_soapy_handler_t *handler = (rf_soapy_handler_t*) h;
  while(handler!=NULL){
    if (SoapySDRDevice_deactivateStream(handler->device, handler->rxStream, 0, 0) != 0)
      return SRSLTE_ERROR;
    handler->rx_stream_active = false;
    handler = handler->next;
  }
  return SRSLTE_SUCCESS;
}


int rf_soapy_stop_tx_stream(void *h)
{
  rf_soapy_handler_t *handler = (rf_soapy_handler_t*) h;
  if(SoapySDRDevice_deactivateStream(handler->device, handler->txStream, 0, 0) != 0)
    return SRSLTE_ERROR;

  handler->tx_stream_active = false;
  return SRSLTE_SUCCESS;
}


void rf_soapy_flush_buffer(void *h) // multi device
{
  int n;
  cf_t tmp1[1024];
  cf_t tmp2[1024];
  void *data[2] = {tmp1, tmp2};
  do {
    n = rf_soapy_recv_with_time_multi(h, data, 1024, 0, NULL, NULL);
  } while (n > 0);
}


bool rf_soapy_has_rssi(void *h)
{
  printf("rf_soapy_has_rssi not supported\n");
  // TODO: implement rf_soapy_has_rssi()
  return false;
}


float rf_soapy_get_rssi(void *h)
{
  printf("rf_soapy_get_rssi not supported\n");
  printf("TODO: implement rf_soapy_get_rssi()\n");
  return 0.0;
}


//add multi-channel support, use nof_rx_antennas to find devices
int rf_soapy_open_multi(char *args, void **h, uint32_t nof_rx_antennas)
{
  size_t length;
  const SoapySDRKwargs *soapy_args = SoapySDRDevice_enumerate(NULL, &length);
  
  if (length == 0) {
    printf("No Soapy devices found.\n");
    return SRSLTE_ERROR;
  }
  char* devlist[16];
  for (size_t i = 0; i < length; i++) {
    printf("Soapy has found device #%d: ", (int)i);
    for (size_t j = 0; j < soapy_args[i].size; j++) {
      printf("%s=%s, ", soapy_args[i].keys[j], soapy_args[i].vals[j]);
      if(!strcmp(soapy_args[i].keys[j],"serial")){  //devlist <= serial
        devlist[i]=soapy_args[i].vals[j];
      }
    }
    printf("\n");
  }
  
  printf("Soapy device list:\n");
  for (size_t i = 0; i < length; i++) {
    printf("  %s\n",devlist[i]);
  }
  
  // create handler
  int devnum = 0;
  *h = NULL;
  rf_soapy_handler_t * last_handler = NULL;
  while (devnum*2<nof_rx_antennas){
    printf("********************************\n");
    printf("Initializing device %d: %s\n",devnum,devlist[devnum]);
    SoapySDRDevice *sdr = SoapySDRDevice_make(&(soapy_args[devnum]));
    if (sdr == NULL) {
      printf("Failed to create Soapy object\n");
      return SRSLTE_ERROR;
    }

    rf_soapy_handler_t *handler = (rf_soapy_handler_t*) malloc(sizeof(rf_soapy_handler_t));
    bzero(handler, sizeof(rf_soapy_handler_t));
    if (*h==NULL)  *h = handler;  else  last_handler->next = handler;
    last_handler = handler;

    handler->device = sdr;
    handler->devname = devlist[devnum];
    handler->next = NULL;
    handler->tx_stream_active = false;
    handler->rx_stream_active = false;

    handler->tx_ants = (size_t*) malloc(sizeof(2*sizeof(size_t)));
    handler->tx_ants[0]=0; handler->tx_ants[1]=1; 
    handler->rx_ants = (size_t*) malloc(sizeof(2*sizeof(size_t)));
    handler->rx_ants[0]=0; handler->rx_ants[1]=1; 
    handler->tx_antnum = (nof_rx_antennas>2?2:1);
    handler->rx_antnum = (nof_rx_antennas>2?2:1);
    
    int rx_max_ant = SoapySDRDevice_getNumChannels(handler->device,SOAPY_SDR_RX); 
    if(rx_max_ant > 0){     
      printf("Setting up RX stream with max %d ant\n",rx_max_ant);
      if(SoapySDRDevice_setupStream(handler->device, &(handler->rxStream), SOAPY_SDR_RX, SOAPY_SDR_CF32, handler->rx_ants, handler->rx_antnum, NULL) != 0) {
        printf("Rx setupStream fail: %s\n", SoapySDRDevice_lastError());
        return SRSLTE_ERROR;
      } 
    }

    int tx_max_ant = SoapySDRDevice_getNumChannels(handler->device,SOAPY_SDR_TX);
    if(tx_max_ant > 0){
      printf("Setting up TX stream with max %d ant\n",tx_max_ant);
      if (SoapySDRDevice_setupStream(handler->device, &(handler->txStream), SOAPY_SDR_TX, SOAPY_SDR_CF32, NULL, 0, NULL) != 0) {
        printf("Tx setupStream fail: %s\n", SoapySDRDevice_lastError());
        return SRSLTE_ERROR;
      }     
    }

    // list device sensors
    size_t sensor_length;
    char** sensors;
    sensors = SoapySDRDevice_listSensors(handler->device, &sensor_length);
    printf("Available device sensors: \n");
    for(int i = 0; i < sensor_length; i++) {
      printf(" - %s\n", sensors[i]);
    }

    // list channel sensors
    sensors = SoapySDRDevice_listChannelSensors(handler->device, SOAPY_SDR_RX, 0, &sensor_length);
    printf("Available sensors for RX channel 0: \n");
    for(int i = 0; i < sensor_length; i++) {
      printf(" - %s\n", sensors[i]);
    }
    devnum++;
  }
  printf("********************************\n");
  return SRSLTE_SUCCESS;
}


int rf_soapy_open(char *args, void **h) //usually for cell_search
{
  return rf_soapy_open_multi(args, h, 1);
}


int rf_soapy_close(void *h) //multi device
{
  rf_soapy_handler_t *handler = (rf_soapy_handler_t*) h;
  while (handler!=NULL){
    if (handler->tx_stream_active) {
      rf_soapy_stop_tx_stream(handler);
      SoapySDRDevice_closeStream(handler->device, handler->txStream);
    }
    
    if (handler->rx_stream_active) {
      rf_soapy_stop_rx_stream(handler);
      SoapySDRDevice_closeStream(handler->device, handler->rxStream);
    }
    SoapySDRDevice_unmake(handler->device);
    rf_soapy_handler_t * last_handler = handler;
    handler = handler->next;
    free(last_handler);
  }
  return SRSLTE_SUCCESS;
}

void rf_soapy_set_master_clock_rate(void *h, double rate) //multi device
{
  // Allow the soapy to automatically set the appropriate clock rate
  rf_soapy_handler_t *handler = (rf_soapy_handler_t*) h;
  while (handler!=NULL){
    if (SoapySDRDevice_setMasterClockRate(handler->device, rate) != 0){
      printf("setMasterClock fail\n");
    }
    else{
      printf("setMasterClock succeed: %f\n",SoapySDRDevice_getMasterClockRate(handler->device));
    }
    handler = handler->next;
  }
}


bool rf_soapy_is_master_clock_dynamic(void *h)
{
  printf("TODO: implement rf_soapy_is_master_clock_dynamic()\n");
  return false;
}


double rf_soapy_set_rx_srate(void *h, double rate) //multi device
{
  rf_soapy_handler_t *handler = (rf_soapy_handler_t*) h;
  while (handler!=NULL){
    for (int i=0;i<handler->rx_antnum;i++){ 
      if (SoapySDRDevice_setSampleRate(handler->device, SOAPY_SDR_RX, handler->rx_ants[i], rate) != 0) {
        printf("setSampleRate Rx fail: %s\n", SoapySDRDevice_lastError());
        return SRSLTE_ERROR;
      }
    }
    handler = handler->next;
  }
  handler = (rf_soapy_handler_t*) h;
  return SoapySDRDevice_getSampleRate(handler->device, SOAPY_SDR_RX, handler->rx_ants[0]);
}

double rf_soapy_set_tx_srate(void *h, double rate)
{
  rf_soapy_handler_t *handler = (rf_soapy_handler_t*) h;
  if (SoapySDRDevice_setSampleRate(handler->device, SOAPY_SDR_TX, 0, rate) != 0) {
    printf("setSampleRate Tx fail: %s\n", SoapySDRDevice_lastError());
    return SRSLTE_ERROR;
  }
  return SoapySDRDevice_getSampleRate(handler->device, SOAPY_SDR_TX,0);
}

double * rf_soapy_get_iqbalance(rf_soapy_handler_t* handler, int direction, int channel)
{
  char angle_buf[32];
  char scale_buf[32];
  char * devidx = handler->devname+strlen(handler->devname)-2;
  char * strdir = (direction==SOAPY_SDR_RX ? "rx":"tx");
  sprintf(scale_buf,"\"%s_%s_%d_scale\"",strdir,devidx,channel);
  sprintf(angle_buf,"\"%s_%s_%d_angle\"",strdir,devidx,channel);
  //printf("(%s)\n(%s)\n",angle_buf,scale_buf);
  double angle_iq = 0;
  double scale_iq = 1;
  static double iqbalance[2]; //ibalance, qbalance
  FILE * fd;
  char buf[1024];
  char para_name[128];
  char para_value_buf[128];
  double para_value;
  char * filename = "../../../utils/iqbalance.json";  //relative path
  fd=fopen(filename,"r");
  if (fd==NULL) printf("WARNING: can not find iqbalance file\n");
  while(fgets(buf,sizeof(buf),fd)!=NULL){
    if (strlen(buf)<=2) continue;
    sscanf(buf,"%[^:]:%[^,]",para_name,para_value_buf);
    para_value_buf[strlen(para_value_buf)-1]='\0';
    para_value = atof(para_value_buf+1);
    if (!strcmp(para_name,angle_buf)) angle_iq = para_value;
    if (!strcmp(para_name,scale_buf)) scale_iq = para_value;
  }
  fclose(fd);

  iqbalance[0] = cos(angle_iq)*scale_iq;
  iqbalance[1] = sin(angle_iq)*scale_iq;
  return iqbalance;
}

double rf_soapy_set_rx_gain(void *h, double gain) // multi device
{
  rf_soapy_handler_t *handler = (rf_soapy_handler_t*) h;

  while (handler!=NULL){
    for (int i=0;i<handler->rx_antnum;i++){ 
      //*******************
      //ref:IQBalance
      //int SoapySDRDevice_getIQBalance(const SoapySDRDevice *device, const int direction, const size_t channel, double *balanceI, double *balanceQ)
      //int SoapySDRDevice_setIQBalance(SoapySDRDevice *device, const int direction, const size_t channel, const double balanceI, const double balanceQ)
      //*******************
    
      double* iqbalance = rf_soapy_get_iqbalance(handler,SOAPY_SDR_RX,handler->rx_ants[i]);
      printf("setIQBalance as %f+1i*%f\n",iqbalance[0],iqbalance[1]);
      if (SoapySDRDevice_setIQBalance(handler->device, SOAPY_SDR_RX, handler->rx_ants[i],iqbalance[0],iqbalance[1])!=0)
      {
        printf("setIQBalance fail\n");
      }
      else{
        double reti,retq;
        int ret = SoapySDRDevice_getIQBalance(handler->device, SOAPY_SDR_RX, handler->rx_ants[i], &reti, &retq);
        printf("getIQBalance %d: %f+1i*%f\n",ret,reti,retq);
      }
      
      if (SoapySDRDevice_setGain(handler->device, SOAPY_SDR_RX, handler->rx_ants[i], gain) != 0)
      {
        printf("setGain fail: %s\n", SoapySDRDevice_lastError());
        return SRSLTE_ERROR;
      }
    }
    handler = handler->next;
  }
  double retgain = rf_soapy_get_rx_gain(h);
  printf("setGain succeed: %f\n",retgain);
  return retgain;
}


double rf_soapy_set_tx_gain(void *h, double gain)
{
  rf_soapy_handler_t *handler = (rf_soapy_handler_t*) h;
  if (SoapySDRDevice_setGain(handler->device, SOAPY_SDR_TX, 0, gain) != 0)
  {
    printf("setGain fail: %s\n", SoapySDRDevice_lastError());
    return SRSLTE_ERROR;
  }
  return rf_soapy_get_tx_gain(h);
}


double rf_soapy_get_rx_gain(void *h)
{
  rf_soapy_handler_t *handler = (rf_soapy_handler_t*) h;
  double retgain = SoapySDRDevice_getGain(handler->device,SOAPY_SDR_RX, handler->rx_ants[0]);
  if (handler->rx_antnum>1){
    double retgain2 = SoapySDRDevice_getGain(handler->device,SOAPY_SDR_RX, handler->rx_ants[1]);
    if (retgain!=retgain2) printf("warning: gains of two channels are different %f and %f\n", retgain, retgain2);
  }
  return retgain;
}


double rf_soapy_get_tx_gain(void *h)
{
  rf_soapy_handler_t *handler = (rf_soapy_handler_t*) h;
  return SoapySDRDevice_getGain(handler->device,SOAPY_SDR_TX,0);
}


double rf_soapy_set_rx_freq(void *h, double freq) //multi device
{
  rf_soapy_handler_t *handler = (rf_soapy_handler_t*) h;
  double retfreq;
  while(handler!=NULL){
    for (int i=0;i<handler->rx_antnum;i++){ 
      if (SoapySDRDevice_setFrequency(handler->device, SOAPY_SDR_RX, handler->rx_ants[i], freq, NULL) != 0)
      {
        printf("setFrequency fail: %s\n", SoapySDRDevice_lastError());
        return SRSLTE_ERROR;
      }
    }

    // Todo: expose antenna setting
    for (int i=0;i<handler->rx_antnum;i++){ 
      if (SoapySDRDevice_setAntenna(handler->device, SOAPY_SDR_RX, handler->rx_ants[i], "TRX") != 0) { //default: "LNAH"
        fprintf(stderr, "Failed to set Rx antenna.\n");
      }
    }

    for (int i=0;i<handler->rx_antnum;i++){
      char * ant = SoapySDRDevice_getAntenna(handler->device, SOAPY_SDR_RX, handler->rx_ants[i]);
      printf("Rx antenna %d set to %s\n",i,ant);
    }

    for (int i=0;i<handler->rx_antnum;i++){
      retfreq = SoapySDRDevice_getFrequency(handler->device, SOAPY_SDR_RX, handler->rx_ants[i]);
      printf("Rx freq %d set to %f\n",i,retfreq);
    }

    handler = handler->next;
  }
  return retfreq;
}

double rf_soapy_set_tx_freq(void *h, double freq)
{
  rf_soapy_handler_t *handler = (rf_soapy_handler_t*) h;
  if (SoapySDRDevice_setFrequency(handler->device, SOAPY_SDR_TX, 0, freq, NULL) != 0)
  {
    printf("setFrequency fail: %s\n", SoapySDRDevice_lastError());
    return SRSLTE_ERROR;
  }

  // Todo: expose antenna name in arguments
  if (SoapySDRDevice_setAntenna(handler->device, SOAPY_SDR_TX, 0, "TRX") != 0) { //default: BAND1
    fprintf(stderr, "Failed to set Tx antenna.\n");
  }

  char *ant = SoapySDRDevice_getAntenna(handler->device, SOAPY_SDR_TX, 0);
  printf("Tx antenna set to %s\n", ant);

  return SoapySDRDevice_getFrequency(handler->device, SOAPY_SDR_TX, 0);
}


void rf_soapy_get_time(void *h, time_t *secs, double *frac_secs)
{
  printf("Todo: implement rf_soapy_get_time()\n");
}


//add multi-channel support
int  rf_soapy_recv_with_time_multi(void *h,
                                   void *data[SRSLTE_MAX_PORTS],
                                   uint32_t nsamples,
                                   bool blocking,
                                   time_t *secs,
                                   double *frac_secs)
{
  rf_soapy_handler_t *handler = (rf_soapy_handler_t*) h;
  int devnum=0;
  int n;
  while(handler!=NULL){
    int flags; //flags set by receive operation
    int num_channels = handler->rx_antnum; // temp
    const long timeoutUs = 1000000; // arbitrarily chosen
    
    int trials = 0;
    int ret = 0;
    long long timeNs; //timestamp for receive buffer
    n = 0;
    do {
      size_t rx_samples = nsamples;
      
      if (rx_samples > nsamples - n){
        rx_samples = nsamples - n;
      }
      void *buffs_ptr[4];
      for (int i=0; i<num_channels; i++){
        cf_t *data_c = (cf_t*) data[i+devnum*2];
        buffs_ptr[i] = &data_c[n];
      }
      ret = SoapySDRDevice_readStream(handler->device, handler->rxStream, buffs_ptr, rx_samples, &flags, &timeNs, timeoutUs);
      if(ret < 0) {
        // continue when getting overflows
        if (ret == SOAPY_SDR_OVERFLOW) {
          fprintf(stderr, "O");
          fflush(stderr);
          continue;
        } else {
          return SRSLTE_ERROR;
        }
      }

      // update rx time
      if (secs != NULL && frac_secs != NULL) {
        *secs = timeNs / 1e9;
        *frac_secs = (timeNs % 1000000000)/1e9;
        //printf("rx_time: secs=%d, frac_secs=%lf timeNs=%lld\n", *secs, *frac_secs, timeNs);
      }

      n += ret;
      trials++;
    } while (n < nsamples && trials < 100);
    
    handler = handler->next;
    devnum++;
  }
  return n;
}


int rf_soapy_recv_with_time(void *h,
                            void *data,
                            uint32_t nsamples,
                            bool blocking,
                            time_t *secs,
                            double *frac_secs)
{
  return rf_soapy_recv_with_time_multi(h, &data, nsamples, blocking, secs, frac_secs);
}


int rf_soapy_send_timed(void *h,
                      void *data,
                      int nsamples,
                      time_t secs,
                      double frac_secs,
                      bool has_time_spec,
                      bool blocking,
                      bool is_start_of_burst,
                      bool is_end_of_burst)
{
  void *_data[SRSLTE_MAX_PORTS]= {data, zero_mem, zero_mem, zero_mem};
  return rf_soapy_send_timed_multi(h, _data, nsamples, secs, frac_secs, has_time_spec, blocking, is_start_of_burst, is_end_of_burst);
}


// Todo: Check correct handling of flags, use RF metrics API, fix timed transmissions
int rf_soapy_send_timed_multi(void *h,
                            void *data[SRSLTE_MAX_PORTS],
                            int nsamples,
                            time_t secs,
                            double frac_secs,
                            bool has_time_spec,
                            bool blocking,
                            bool is_start_of_burst,
                            bool is_end_of_burst)
{
  rf_soapy_handler_t *handler = (rf_soapy_handler_t *) h;
  int flags = 0;
  const long timeoutUs = 2000; // arbitrarily chosen
  long long timeNs = 0;
  int trials = 0;
  int ret = 0;
  int n = 0;


  if (!handler->tx_stream_active) {
    rf_soapy_start_tx_stream(h);
  }

  if (is_start_of_burst && is_end_of_burst) {
    flags |= SOAPY_SDR_ONE_PACKET;
  }

  if (is_end_of_burst) {
    flags |= SOAPY_SDR_END_BURST;
  }

  if (has_time_spec) {
    flags |= SOAPY_SDR_HAS_TIME;
    timeNs = secs * 1000000000;
    timeNs = timeNs + (frac_secs * 1000000000);
    //printf("time_spec: secs=%d, frac_secs=%lf timeNs=%lld\n", secs, frac_secs, timeNs);
  }

  do {
    size_t tx_samples = nsamples;
    if (tx_samples > nsamples - n) {
      tx_samples = nsamples - n;
    }

    ret = SoapySDRDevice_writeStream(handler->device, handler->txStream, (const void *)data, tx_samples, &flags, timeNs, timeoutUs);
    if (ret == SOAPY_SDR_TIMEOUT) {
      printf("L");
      continue;
    }
    if (ret == SOAPY_SDR_OVERFLOW) {
      printf("O");
      continue;
    }
    if (ret == SOAPY_SDR_UNDERFLOW) {
      printf("U");
      continue;
    }
    if (ret < 0) {
      fprintf(stderr, "Error during writeStream\n");
      exit(-1);
      return SRSLTE_ERROR;
    }

    n += ret;
    trials++;
  } while (n < nsamples && trials < 100);

  if (n != nsamples) {
    fprintf(stderr, "Couldn't write all samples.\n");
    return SRSLTE_ERROR;
  }

  return ret;
}
