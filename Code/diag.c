/*
HOMEWORK 1 - CONTROLLER POSIX

Code submitted by:
N46004251 - Luca Petracca
N46004302 - Gianluca Pepe
N46004416 - Alessandro Minutolo
*/

#include <signal.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <mqueue.h>
#include <fcntl.h>
#include <string.h>
#include <sched.h>
#include "rt-lib.h"
#include "parameters.h"

#define SLEEP_TIME_S 1 
#define DS_REQ_QUEUE_NAME   "/req_ds"
#define DS_RES_QUEUE_NAME	"/res_ds"

//Per la Cpu Affinity utilizziamo il comando taskset


int main(void){
	int start = 1;
	
	// Setto la priorità e lo scheduler del processo
	struct sched_param sp;
	sp.sched_priority = 20;
	sched_setscheduler(0, SCHED_FIFO, &sp);
	
	// descrittore della coda
	mqd_t req_ds;
	mqd_t res_ds;
	char message[50];

	struct mq_attr attr;

	attr.mq_flags = 0;				
	attr.mq_maxmsg = MAX_MESSAGES;	
	attr.mq_msgsize = MAX_MSG_SIZE; 
	attr.mq_curmsgs = 0;
	
	// Apriamo una coda in sola scrittura (O_WRONLY). La coda serve ad inviare messaggi al server.
	if ((req_ds = mq_open (DS_REQ_QUEUE_NAME, O_WRONLY | O_CREAT, QUEUE_PERMISSIONS, &attr)) == -1) {
		perror ("Client: req_ds (server)");
		exit (1);
	}

	// Apriamo una coda in sola lettura (O_RDONLY), se non esiste la creiamo (O_CREAT).
	if ((res_ds = mq_open (DS_RES_QUEUE_NAME, O_RDONLY | O_CREAT, QUEUE_PERMISSIONS, &attr)) == -1) {
		perror ("Diag: res_ds (Diag)");
		exit (1);
	}
	
	while(start){
		printf("\nPRESS 1 TO SEND DIAG REQUEST:\n");
		printf(">");
		scanf("%d",&start);
		// There should be a control for not numeric input
		if(start == 1){
			printf("Sending request...\n");
			if (mq_send (req_ds, message, strlen(message) + 1, 0) == -1) {
        			perror ("async_request: Not able to send message to ds");
			}
		}
		else{
			start = 0; 		
		}
		if(mq_receive(res_ds, message, MAX_MSG_SIZE, NULL) == -1){
			//perror ("Diag: Not able to open client queue");
			break;
		}
		else{
			printf ("Aperiodic Message received:\n %s", message);
		}
	}
	
	/* Clear */
	if (mq_close (req_ds) == -1) {
		perror ("diag: mq_close req_ds");
		exit (1);
	}	
	if (mq_close (res_ds) == -1) {
		perror ("Diag: mq_close res_ds");
		exit (1);
	}
	if (mq_unlink (DS_REQ_QUEUE_NAME) == -1) {
		perror ("Diag: mq_unlink Diag queue");
		exit (1);
	}	
	if (mq_unlink (DS_RES_QUEUE_NAME) == -1) {
		perror ("Diag: mq_unlink Diag queue");
		exit (1);
	}	
	return 0;
}
