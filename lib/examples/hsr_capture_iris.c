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

#include <stdio.h>
#include <signal.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include <unistd.h>
#include <math.h>
#include <time.h>

#include <stdbool.h>

#include "srslte/srslte.h"
#include "srslte/phy/rf/rf.h"
#include "srslte/phy/rf/rf_utils.h"

#define MHZ             1000000
#define SAMP_FREQ       1920000
#define FLEN            9600
#define FLEN_PERIOD     0.005
#define MAX_EARFCN 1000
char * path="/media/soar/My Passport/data/tmp";
//char * path="data/8.18.1";
char * output_file_name = NULL;

int band = -1;
int earfcn_start=-1, earfcn_end = -1;

cell_search_cfg_t cell_detect_config = {
  SRSLTE_DEFAULT_MAX_FRAMES_PBCH,
  SRSLTE_DEFAULT_MAX_FRAMES_PSS, 
  SRSLTE_DEFAULT_NOF_VALID_PSS_FRAMES,
  0
};

struct cells {
  srslte_cell_t cell;
  float freq; 
  int dl_earfcn;
  float power;
};
struct cells results[1024]; 
uint32_t n_found_cells=0;

static bool keep_running = true;
char *rf_args="";
float rf_gain=70.0, rf_freq=-1.0, rf_rate=0.96e6;
int nof_prb = 6;
int nof_samples = -1;
int nof_frames = -1;
int N_id_2 = -1; 
uint32_t nof_rx_antennas = 1;
bool save_datainfo = true;

void int_handler(int dummy) {
  keep_running = false;
}

void usage(char *prog) {
  printf("Usage: %s [agrnv] -l N_id_2 -f rx_frequency_hz -o output_file\n", prog);
  printf("\t-a RF args [Default %s]\n", rf_args);
  printf("\t-g RF Gain [Default %.2f dB]\n", rf_gain);
  printf("\t-p nof_prb [Default %d]\n", nof_prb);
  printf("\t-s earfcn_start [Default All]\n");
  printf("\t-e earfcn_end [Default All]\n");
  printf("\t-n nof_frames [Default %d]\n", nof_frames);
  printf("\t-A nof_rx_antennas [Default %d]\n", nof_rx_antennas);
  printf("\t-v verbose\n");
}

void parse_args(int argc, char **argv) {
  int opt;
  while ((opt = getopt(argc, argv, "agpnrvfolAbse?")) != -1) {
    switch (opt) {
    case 'o':
      output_file_name = argv[optind];
      save_datainfo = false;
      break;
    case 'a':
      rf_args = argv[optind];
      break;
    case 'r':
      rf_rate = atof(argv[optind]);
    case 'g':
      rf_gain = atof(argv[optind]);
      break;
    case 'p':
      nof_prb = atoi(argv[optind]);
      break;
    case 'f':
      rf_freq = atof(argv[optind]);
      break;
    case 'n':
      nof_frames = atoi(argv[optind]);
      break;
    case 'l':
      N_id_2 = atoi(argv[optind]);
      break;
    case 'A':
      nof_rx_antennas = (uint32_t) atoi(argv[optind]);
      break;
    case 'v':
      srslte_verbose++;
      break;
    case 's':
      earfcn_start = atoi(argv[optind]);
      break;
    case 'e':
      earfcn_end = atoi(argv[optind]);
      break;
    case 'b':
      band = atoi(argv[optind]);
      break;
    case '?':
    default:
      usage(argv[0]);
      exit(-1);
    }
  }
}


int srslte_rf_recv_wrapper(void *h, cf_t *data[SRSLTE_MAX_PORTS], uint32_t nsamples, srslte_timestamp_t *t) {
  DEBUG(" ----  Receive %d samples  ---- \n", nsamples);
  void *ptr[SRSLTE_MAX_PORTS];
  for (int i=0;i<SRSLTE_MAX_PORTS;i++) {
    ptr[i] = data[i];
  }
  return srslte_rf_recv_with_time_multi((srslte_rf_t*) h, ptr, nsamples, 1, NULL, NULL);
}

bool go_exit = false; 

void sig_int_handler(int signo)
{
  printf("SIGINT received. Exiting...\n");
  if (signo == SIGINT) {
    go_exit = true;
  }
}

double srslte_rf_set_rx_gain_wrapper(void *h, double f) {
  return srslte_rf_set_rx_gain((srslte_rf_t*) h, f);
}

void cell_search(){
  int n; 
  srslte_rf_t rf;
  srslte_ue_cellsearch_t cs; 
  srslte_ue_cellsearch_result_t found_cells[3]; 
  int nof_freqs; 
  srslte_earfcn_t channels[MAX_EARFCN];
  uint32_t freq;
  n_found_cells=0;

  printf("Opening RF device...\n");
  if (srslte_rf_open(&rf, rf_args)) {
    fprintf(stderr, "Error opening rf\n");
    exit(-1);
  }  
  if (!cell_detect_config.init_agc) {
    srslte_rf_set_rx_gain(&rf, rf_gain);
  } else {
    printf("Starting AGC thread...\n");
    if (srslte_rf_start_gain_thread(&rf, false)) {
      fprintf(stderr, "Error opening rf\n");
      exit(-1);
    }
    srslte_rf_set_rx_gain(&rf, 50);      
  }

  srslte_rf_set_master_clock_rate(&rf, 80e6); //80MHz for iris
  //srslte_rf_set_master_clock_rate(&rf, 30.72e6);        

  // Supress RF messages
  srslte_rf_suppress_stdout(&rf);
  
  nof_freqs = srslte_band_get_fd_band(band, channels, earfcn_start, earfcn_end, MAX_EARFCN);
  if (nof_freqs < 0) {
    fprintf(stderr, "Error getting EARFCN list\n");
    exit(-1);
  }

  sigset_t sigset;
  sigemptyset(&sigset);
  sigaddset(&sigset, SIGINT);
  sigprocmask(SIG_UNBLOCK, &sigset, NULL);
  signal(SIGINT, sig_int_handler);

  if (srslte_ue_cellsearch_init_multi(&cs, cell_detect_config.max_frames_pss, srslte_rf_recv_wrapper, 1, (void*) &rf)) {
    fprintf(stderr, "Error initiating UE cell detect\n");
    exit(-1);
  }

  if (cell_detect_config.max_frames_pss) {
    srslte_ue_cellsearch_set_nof_valid_frames(&cs, cell_detect_config.nof_valid_pss_frames);
  }
  if (cell_detect_config.init_agc) {
    srslte_ue_sync_start_agc(&cs.ue_sync, srslte_rf_set_rx_gain_wrapper, cell_detect_config.init_agc);
  }

  for (freq=0;freq<nof_freqs && !go_exit;freq++) {
  
    /* set rf_freq */
    srslte_rf_set_rx_freq(&rf, (double) channels[freq].fd * MHZ);
    srslte_rf_rx_wait_lo_locked(&rf);
    INFO("Set rf_freq to %.3f MHz\n", (double) channels[freq].fd * MHZ/1000000);
    
    printf("[%3d/%d]: EARFCN %d Freq. %.2f MHz looking for PSS.\n", freq, nof_freqs,
                      channels[freq].id, channels[freq].fd);fflush(stdout);
    
    if (SRSLTE_VERBOSE_ISINFO()) {
      printf("\n");
    }
      
    bzero(found_cells, 3*sizeof(srslte_ue_cellsearch_result_t));

    INFO("Setting sampling frequency %.2f MHz for PSS search\n", SRSLTE_CS_SAMP_FREQ/1000000);
    srslte_rf_set_rx_srate(&rf, SRSLTE_CS_SAMP_FREQ);
    INFO("Starting receiver...\n");
    srslte_rf_start_rx_stream(&rf, false);
    
    n = srslte_ue_cellsearch_scan(&cs, found_cells, NULL); 
    if (n < 0) {
      fprintf(stderr, "Error searching cell\n");
      exit(-1);
    } else if (n > 0) {
      for (int i=0;i<3;i++) {
        if (found_cells[i].psr > 10.0) {
          srslte_cell_t cell;
          cell.id = found_cells[i].cell_id; 
          cell.cp = found_cells[i].cp; 
          int ret = rf_mib_decoder(&rf, 1, &cell_detect_config, &cell, NULL);
          if (ret < 0) {
            fprintf(stderr, "Error decoding MIB\n");
            exit(-1);
          }
          if (ret == SRSLTE_UE_MIB_FOUND) {
            printf("Found CELL ID %d. %d PRB, %d ports, %f dbm\n", 
                 cell.id, 
                 cell.nof_prb, 
                 cell.nof_ports,
                 10*log10(found_cells[i].peak)-rf_gain);
            if (cell.nof_ports > 0) {
              memcpy(&results[n_found_cells].cell, &cell, sizeof(srslte_cell_t));
              results[n_found_cells].freq = channels[freq].fd; 
              results[n_found_cells].dl_earfcn = channels[freq].id;
              results[n_found_cells].power = found_cells[i].peak;
              n_found_cells++;
            }
            if (10*log10(results[n_found_cells-1].power)-rf_gain>-100){
              go_exit=true;
            }
            break;
          }          
        }
      }
    }    
  }

  
  srslte_ue_cellsearch_free(&cs);
  srslte_rf_close(&rf);
}

int main(int argc, char **argv) {
    parse_args(argc, argv);
    //srslte_use_standard_symbol_size(true);
    //cell_search
    cell_search();
    if (n_found_cells==0){
      printf("No cell found\n");
      exit(0);
    }
    int maxcell = 0;
    float maxpower = results[0].power;
    for (int i=0;i<n_found_cells;i++){
      if (results[i].power>maxpower){
        maxpower = results[i].power;
        maxcell = i;
      }
    }
    
    //start receiving
    go_exit=false;
    cf_t *buffer[SRSLTE_MAX_PORTS];
    int sample_count, n;
    srslte_rf_t rf;
    srslte_filesink_t sink[SRSLTE_MAX_PORTS];
    uint32_t buflen;

    
    
    int real_nof_prb = results[maxcell].cell.nof_prb;
    /*
    //usrp cannot support two rx with prb=100, thus change prb=100 to prb=75 if two rx
    if (results[maxcell].cell.nof_prb>75 && nof_rx_antennas==2){
      printf("Downsample from prb of %d to 75\n",results[maxcell].cell.nof_prb);
      real_nof_prb = 75;
    }
    */
    int subframe_sz = srslte_symbol_sz(real_nof_prb)*15;
    rf_rate = subframe_sz*1000;
    buflen = subframe_sz*5;  //length of 5 subframe     
    sample_count = 0;

    

    for (int i = 0; i < nof_rx_antennas; i++) {
      buffer[i] = malloc(sizeof(cf_t) * buflen);
      if (!buffer[i]) {
        perror("malloc");
        exit(-1);
      }
    }

    printf("Opening RF device...\n");
    if (srslte_rf_open_multi(&rf, rf_args, nof_rx_antennas)) {
      fprintf(stderr, "Error opening rf\n");
      exit(-1);
    }
    /* master_clock_rate must be divided by sampling rate*/
    srslte_rf_set_master_clock_rate(&rf, 80e6); //80MHz for iris
    /*
    if (real_nof_prb == 75 || real_nof_prb <= 15){
      srslte_rf_set_master_clock_rate(&rf, 30.72e6); 
    }
    else{
      srslte_rf_set_master_clock_rate(&rf, 23.04e6);  
    }
    */
    
    printf("Set RF rate as %.2f MHz\n",rf_rate*1e-6);
    float srate = srslte_rf_set_rx_srate(&rf, rf_rate); 
    if (srate != rf_rate) {
      if (srate < 10e6) {          
        srslte_rf_set_master_clock_rate(&rf, 4*rf_rate);        
      } else {
        srslte_rf_set_master_clock_rate(&rf, rf_rate);        
      }
      srate = srslte_rf_set_rx_srate(&rf, rf_rate);
      if (srate != rf_rate) {
        fprintf(stderr, "Errror setting samplign frequency %.2f MHz\n", rf_rate*1e-6);
        exit(-1);
      }
    }
    printf("Correctly RX rate: %.2f MHz\n", srate*1e-6);

    printf("Set RX gain: %.2f dB\n", srslte_rf_set_rx_gain(&rf, rf_gain));
    printf("Set RX freq: %.2f MHz\n", srslte_rf_set_rx_freq(&rf, results[maxcell].freq * MHZ)/1000000);
    
    srslte_rf_rx_wait_lo_locked(&rf);
    srslte_rf_start_rx_stream(&rf, false);
    
    
    char time_buf[100];
    if (save_datainfo){
      time_t cur_time;
      struct tm *t;
      time(&cur_time);
      t=gmtime(&cur_time);
      output_file_name = time_buf; 
      sprintf(output_file_name,"%s/%d.%d.%d-%d-%d",path,t->tm_mon+1,t->tm_mday,t->tm_hour+8,t->tm_min,t->tm_sec);
    }

    for (int i = 0; i < nof_rx_antennas; i++) {
      char file_name[100];
      sprintf(file_name, "%s-%d.bin", output_file_name,i);
      //file_name=output_file_name;
      if (srslte_filesink_init(&sink[i], file_name, SRSLTE_COMPLEX_FLOAT_BIN)!=0){
        printf("Open file failed\n");
        return 0;
      }
      printf("output_file_name_%d is:%s\n",i,file_name);
    }
    
    printf("start receiving\n");
    time_t start_time = time(NULL);
    nof_samples = nof_frames * subframe_sz * 10;
    while((sample_count < nof_samples || nof_samples == -1)
          && keep_running){
     
      n = srslte_rf_recv_with_time_multi(&rf, (void**) buffer, buflen, true, NULL, NULL);
      if (n < 0) {
        fprintf(stderr, "Error receiving samples\n");
        exit(-1);
      }
      else printf("Receive %d samples\n", n);
      
      for (int i = 0; i < nof_rx_antennas; i++) {
        srslte_filesink_write_multi(&sink[i], (void**) &buffer[i], buflen, 1);
      }
      sample_count += buflen;
      
    }
    time_t end_time = time(NULL);
    printf("stop recving in %f seconds\n",difftime(end_time,start_time));

    for (int i = 0; i < nof_rx_antennas; i++) {
      if (buffer[i]) {
        free(buffer[i]);
      }
    }

    if (save_datainfo){
      FILE* fd;
      char datainfo[100];
      sprintf(datainfo, "%s/datainfo.csv", path);
      printf("%s\n", datainfo);
      fd=fopen(datainfo,"a");
      fprintf(fd,"%s,%.1f,%d,%d,%d,%d,%.1f,%d\n",
             output_file_name,
             results[maxcell].freq,
             results[maxcell].dl_earfcn,
             results[maxcell].cell.id, 
             results[maxcell].cell.nof_prb, 
             results[maxcell].cell.nof_ports, 
             10*log10(results[maxcell].power)-rf_gain,
             nof_frames);
      fclose(fd);
    }

    for (int i = 0; i < nof_rx_antennas; i++) {
      srslte_filesink_free(&sink[i]);
    }
    srslte_rf_close(&rf);

    printf("\nFound %d cells\n", n_found_cells);
    for (int i=0;i<n_found_cells;i++) {
      printf("Found CELL %.1f MHz, EARFCN=%d, PHYID=%d, %d PRB, %d ports, PSS power=%.1f dBm\n", 
           results[i].freq,
           results[i].dl_earfcn,
           results[i].cell.id, 
           results[i].cell.nof_prb, 
           results[i].cell.nof_ports, 
           10*log10(results[i].power)-rf_gain);

    }
    printf("Ok - wrote %d samples of %d frames in %f seconds\n", sample_count, nof_frames, nof_frames/100.0);
    exit(0);
}
