/*
HOMEWORK 1 - CONTROLLER POSIX

Code submitted by:
N46004251 - Luca Petracca
N46004302 - Gianluca Pepe
N46004416 - Alessandro Minutolo
*/

//------------------- CONTROLLER.C ---------------------- 

#include <time.h>
#include <pthread.h>
#include <signal.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <mqueue.h>
#include <fcntl.h>
#include <string.h>
#include "rt-lib.h"
#include "parameters.h"

//Per la Cpu Affinity utilizziamo il comando taskset


//emulates the controller
static int attivo = 0;
static int keep_on_running = 1;

struct shared_int {
	int value;
	pthread_mutex_t lock;
};
static struct shared_int shared_avg_sensor;
static struct shared_int shared_control;

int buffer[BUF_SIZE];
int head = 0;

/*Nella replica abbiamo scelto di mantenere non condivise reference e buffer, in quanto il DS comunica solo con il controller.
Inoltre, sarà solo la replica a mantenere la reference di un controllore che smette di funzionare (tramite l'heartbeat), 
mentre non sarà vero il contrario: attivato il controllore, porterà il sensore a regime. Per la gestione della attivazione
e disattivazione della replica ci avvaliamo di una variabile di stato (attivo) impostata a 0 quando la replica non riceve heartbeat
per due cicli (quindi quando deve partire), a 1 viceversa. */

void * acquire_filter_loop(void * par) {
	
	periodic_thread *th = (periodic_thread *) par;
	start_periodic_timer(th,TICK_TIME);

	// Messaggio da prelevare dal driver
	char message [MAX_MSG_SIZE];

	/* Coda */
	struct mq_attr attr;

	attr.mq_flags = 0;				
	attr.mq_maxmsg = MAX_MESSAGES;	
	attr.mq_msgsize = MAX_MSG_SIZE; 
	attr.mq_curmsgs = 0;
	
	// Apriamo la coda sensor del plant in lettura 
	mqd_t sensor_qd;
	if ((sensor_qd = mq_open (SENSOR_QUEUE_NAME, O_RDONLY | O_CREAT, QUEUE_PERMISSIONS,&attr)) == -1) {
		perror ("acquire filter loop: mq_open (sensor)");
		exit (1);
	}
	unsigned int sum = 0;
	int cnt = BUF_SIZE;
	while (keep_on_running)
	{
		wait_next_activation(th);

		// PRELIEVO DATI dalla coda del PLANT
		if (mq_receive(sensor_qd, message,MAX_MSG_SIZE,NULL) == -1){
			perror ("acquire filter loop: mq_receive (actuator)");	
			break;						//DEBUG
		}
		else{ 
			buffer[head] = atoi(message);
			sum += buffer[head];
			head = (head+1)%BUF_SIZE;
			cnt--;

			//printf("\t\t\t\tbuffer[%d]=%d, sum=%d\n",head,buffer[head],sum); //DEBUG

			// calcolo media sulle ultime BUF_SIZE letture
			if (cnt == 0) {
				cnt = BUF_SIZE;
				pthread_mutex_lock(&shared_avg_sensor.lock);
				shared_avg_sensor.value = sum/BUF_SIZE;
				//printf("\t\t\t\tavg_sensor.value=%d\n",shared_avg_sensor.value); //DEBUG
				pthread_mutex_unlock(&shared_avg_sensor.lock);
				sum = 0;
			}	
		}
	}		

	/* Clear */
    if (mq_close (sensor_qd) == -1) {
        perror ("acquire filter loop: mq_close sehsor_qd");
        exit (1);
    }

	return 0;
}


void * control_loop(void * par) {

	periodic_thread *th = (periodic_thread *) par;
	start_periodic_timer(th,TICK_TIME);
	
	// Messaggio da prelevare dal reference
	char message [MAX_MSG_SIZE];
	
	/* Coda */
	struct mq_attr attr;

	attr.mq_flags = 0;				
	attr.mq_maxmsg = MAX_MESSAGES;	
	attr.mq_msgsize = MAX_MSG_SIZE; 
	attr.mq_curmsgs = 0;
	
	// Apriamo la coda per il reference, in lettura e non bloccante
	mqd_t reference_qd;
	if ((reference_qd = mq_open (REFERENCE_QUEUE_NAME, O_RDONLY | O_CREAT | O_NONBLOCK, QUEUE_PERMISSIONS,&attr)) == -1) {
		perror ("control loop: mq_open (reference)");
		exit (1);
	}
	unsigned int reference = 110;
	unsigned int plant_state = 0;
	int error = 0;
	unsigned int control_action = 0;
	while (keep_on_running)
	{
		wait_next_activation(th);

		// legge il plant state 
		pthread_mutex_lock(&shared_avg_sensor.lock);
		plant_state = shared_avg_sensor.value;
		pthread_mutex_unlock(&shared_avg_sensor.lock);

		// riceve la reference (in modo non bloccante)
		/*Attraverso l'uso di una variabile di stato, la variabile attivo, controllo se sono attivo (== 0) e solo in quel caso
		mi metto in ascolto di una reference. */
		if(attivo==0){
		if (mq_receive(reference_qd, message,MAX_MSG_SIZE,NULL) == -1){
			printf ("No reference ...\n");							//DEBUG
		}
		else{
			printf ("Reference received: %s.\n",message);			//DEBUG
			reference = atoi(message);
		}
		}

		// calcolo della legge di controllo
		error = reference - plant_state;

		if (error > 0) control_action = 1;
		else if (error < 0) control_action = 2;
		else control_action = 3;

		// aggiorna la control action
		pthread_mutex_lock(&shared_control.lock);
		shared_control.value = control_action;
		pthread_mutex_unlock(&shared_control.lock);
	}
	/* Clear */
    if (mq_close (reference_qd) == -1) {
        perror ("control loop: mq_close reference_qd");
        exit (1);
    }
	return 0;
}

void * actuator_loop(void * par) {

	periodic_thread *th = (periodic_thread *) par;
	start_periodic_timer(th,TICK_TIME);

	// Messaggio da prelevare dal driver
	char message [MAX_MSG_SIZE];

	/* Coda */
	struct mq_attr attr;

	attr.mq_flags = 0;				
	attr.mq_maxmsg = MAX_MESSAGES;	
	attr.mq_msgsize = MAX_MSG_SIZE; 
	attr.mq_curmsgs = 0;
	
	// Apriamo la coda actuator del plant in scrittura 
	mqd_t actuator_qd;
	if ((actuator_qd = mq_open (ACTUATOR_QUEUE_NAME, O_WRONLY|O_CREAT, QUEUE_PERMISSIONS,&attr)) == -1) {
		perror ("actuator  loop: mq_open (actuator)");
		exit (1);
	}	

	unsigned int control_action = 0;
	unsigned int control = 0;
	while (keep_on_running)
	{
		wait_next_activation(th);
		//Solo se sono in stato attivo attuo le modifiche
		if(attivo==0){
		// prelievo della control action
		pthread_mutex_lock(&shared_control.lock);
		control_action = shared_control.value;
		pthread_mutex_unlock(&shared_control.lock);
		
		switch (control_action) {
			case 1: control = 1; break;
			case 2:	control = -1; break;
			case 3:	control = 0; break;
			default: control = 0;
		}
		printf("Control: %d\n",control); //DEBUG
		sprintf (message, "%d", control);
		//invio del controllo al driver del plant
		if (mq_send (actuator_qd, message, strlen (message) + 1, 0) == -1) {
		    perror ("Sensor driver: Not able to send message to controller");
		    continue;
		}}
	}
	/* Clear */
    if (mq_close (actuator_qd) == -1) {
        perror ("Actuator loop: mq_close actuator_qd");
        exit (1);
    }
	return 0;
}

void * watchdog(void * par) { 

	periodic_thread *th = (periodic_thread *) par;
	start_periodic_timer(th,TICK_TIME);

	char message [MAX_MSG_SIZE];

	/* Coda */
	struct mq_attr attr;

	attr.mq_flags = 0;				
	attr.mq_maxmsg = MAX_MESSAGES;	
	attr.mq_msgsize = MAX_MSG_SIZE; 
	attr.mq_curmsgs = 0;
	
	// Apriamo la coda per gli heartbeat del controllore 
	mqd_t wd_qd;
	if ((wd_qd = mq_open (WDOG_QUEUE_NAME, O_RDONLY|O_CREAT, QUEUE_PERMISSIONS,&attr)) == -1) {
		perror ("watchdog  loop: mq_open (watchdog)");
		exit (1);
	}
	
	while(keep_on_running){
		
		wait_next_activation(th);
		start_periodic_timer(th,TICK_TIME*BUF_SIZE*2);
		//Per non far andare in crash il controllore dobbiamo utilizzare una recive temporizzata e non una bloccante, altrimenti ci troveremmo perennemente in uno stato attivo
		if(mq_timedreceive(wd_qd,message,MAX_MSG_SIZE,NULL,&(th->r))==-1){
			printf("REPLICA ACTIVE\n");
			//Per la logica di attivazione ci siamo limitati ad assegnare 0 alla variabile di stato attivo. 
			attivo = 0;
		} else { 
			printf("REPLICA NOT ACTIVE\n");
			//Per la logica di disattivazione ci siamo limitati ad assegnare 1 alla variabile di stato attivo.
			attivo = 1;
		}
	}
	
	if (mq_close (wd_qd) == -1) {
        	perror ("Wdog loop: mq_close wd_qd");
        	exit (1);
    	}
}

int main(void)
{
	printf("The replica is STARTED! [press 'q' to stop]\n");
 	
	pthread_t acquire_filter_thread;
    	pthread_t control_thread;
    	pthread_t actuator_thread;
    	pthread_t watchdog_thread;

	pthread_mutex_init(&shared_avg_sensor.lock, NULL);
	pthread_mutex_init(&shared_control.lock, NULL);

	pthread_attr_t myattr;
	struct sched_param myparam;

	pthread_attr_init(&myattr);
	pthread_attr_setschedpolicy(&myattr, SCHED_FIFO);
	pthread_attr_setinheritsched(&myattr, PTHREAD_EXPLICIT_SCHED); 

	// ACQUIRE FILTER THREAD
	periodic_thread acquire_filter_th;
	acquire_filter_th.period = TICK_TIME;
	acquire_filter_th.priority = 50;

	myparam.sched_priority = acquire_filter_th.priority;
	pthread_attr_setschedparam(&myattr, &myparam); 
	pthread_create(&acquire_filter_thread,&myattr,acquire_filter_loop,(void*)&acquire_filter_th);

	// CONTROL THREAD
	periodic_thread control_th;
	control_th.period = TICK_TIME*BUF_SIZE;
	control_th.priority = 45;

	myparam.sched_priority = control_th.priority;
	pthread_attr_setschedparam(&myattr, &myparam); 
	pthread_create(&control_thread,&myattr,control_loop,(void*)&control_th);

	// ACTUATOR THREAD
	periodic_thread actuator_th;
	actuator_th.period = TICK_TIME*BUF_SIZE;
	actuator_th.priority = 45;

	pthread_attr_setschedparam(&myattr, &myparam); 
	pthread_create(&actuator_thread,&myattr,actuator_loop,(void*)&actuator_th);

	// WATCHDOG THREAD
	periodic_thread wdog_th;
	wdog_th.period = TICK_TIME*BUF_SIZE*2;
	wdog_th.priority = 40;
	
	myparam.sched_priority = wdog_th.priority;
	pthread_attr_setschedparam(&myattr, &myparam);
	pthread_create(&watchdog_thread,&myattr,watchdog,(void*)&wdog_th);

	pthread_attr_destroy(&myattr);
		
	/* Wait user exit commands*/
	while (1) {
   		if (getchar() == 'q') break;
  	}
	keep_on_running = 0;

		if (mq_unlink (REFERENCE_QUEUE_NAME) == -1){
		perror( "Main: mq_unlink reference queue");
		exit(1);
		}
    
    	if (mq_unlink (WDOG_QUEUE_NAME) == -1) {
        perror ("Main: mq_unlink wdog queue");
        exit (1);
    	}

 	printf("The replica is STOPPED\n");
	return 0;
}




