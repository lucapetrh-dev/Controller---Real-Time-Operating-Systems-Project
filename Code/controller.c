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

//emulates the controller
//Per la Cpu Affinity utilizziamo il comando taskset

static int keep_on_running = 1;

struct shared_int {
	int value;
	pthread_mutex_t lock;
};
static struct shared_int shared_avg_sensor;
static struct shared_int shared_control;

/*Definisco anche la reference e il buffer come variabili condivise in quanto dovranno essere prelevate anche dal DS.*/
struct shared_buffer{
	int b[BUF_SIZE];
	pthread_mutex_t lock;
};
static struct shared_int shared_reference; 
static struct shared_buffer shared_buffer;
int head = 0;


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
			pthread_mutex_lock(&shared_buffer.lock);
			shared_buffer.b[head] = atoi(message);
			sum += shared_buffer.b[head];
			pthread_mutex_unlock(&shared_buffer.lock);
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
	
	//Heartbeat da inviare al Watchdog
	char heartbeat [MAX_MSG_SIZE];
	
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
	//Acquisisco il controllo della variabile reference e setto valore di default a 110
	pthread_mutex_lock(&shared_reference.lock);
	shared_reference.value = 110;
	pthread_mutex_unlock(&shared_reference.lock);
	//unsigned int reference = 110; Di default non era variabile condivisa, nel controllre la rendiamo condivisa per il DS

	unsigned int plant_state = 0;
	int error = 0;
	unsigned int control_action = 0;
	
	//Definisco la coda del watchdog
		mqd_t wd_qd;
	
	if ((wd_qd = mq_open (WDOG_QUEUE_NAME, O_WRONLY|O_CREAT, QUEUE_PERMISSIONS,&attr)) == -1) {
		perror ("actuator  loop: mq_open (actuator)");
		exit (1);
	}	
		
	while (keep_on_running)
	{
		wait_next_activation(th);

		// legge il plant state 
		pthread_mutex_lock(&shared_avg_sensor.lock);
		plant_state = shared_avg_sensor.value;
		pthread_mutex_unlock(&shared_avg_sensor.lock);

		// riceve la reference (in modo non bloccante)
		if (mq_receive(reference_qd, message,MAX_MSG_SIZE,NULL) == -1){
			printf ("No reference ...\n");							//DEBUG
		}
		else{
			printf ("Reference received: %s.\n",message);			//DEBUG
			//reference = atoi(message);
			pthread_mutex_lock(&shared_reference.lock);
			shared_reference.value = atoi(message);
			pthread_mutex_unlock(&shared_reference.lock);
		}
		
		//Attraverso l'heartbeat inviamo l'ultima reference impostata in modo che la replica possa ripartire dall'ultima reference
		strcpy(heartbeat,message);
		//sprintf (heartbeat, "%s", message);
		//Impostiamo il periodo di 2 cicli
		start_periodic_timer(th,TICK_TIME*BUF_SIZE*2);
		
		//Timed send mi permette di inviare l'heartbeat ogni 2 cicli
		if (mq_timedsend(wd_qd,heartbeat,MAX_MSG_SIZE,0,&(th->r)) == -1){
			printf ("Non inviato...\n");
		}
		else{
			printf ("Inviato correttamente: %s.\n",heartbeat);
		}

		// calcolo della legge di controllo
		//error = reference - plant_state;
		pthread_mutex_lock(&shared_reference.lock);
		error = shared_reference.value - plant_state;
		pthread_mutex_unlock(&shared_reference.lock);

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
    
    if (mq_close (wd_qd) == -1) {
        	perror ("Wdog loop: mq_close wd_qd");
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
		}
	}
	/* Clear */
    if (mq_close (actuator_qd) == -1) {
        perror ("Actuator loop: mq_close actuator_qd");
        exit (1);
    }
	return 0;
}

/***************************** DS ************************************/


int diag_fun(int wcet, mqd_t *res_ds, char * message,int priority){
	
	//Invia un messaggio al processo diag per fare la print
	//Definisco 4 stringhe "ausiliari" per fare la sprintf, in quanto dobbiamo convertire i valori interi in stringhe e poi concatenare
	char avg_sensor_string[MAX_MSG_SIZE];
	char control_string[MAX_MSG_SIZE];
	char buffer_string[MAX_MESSAGES];
	char reference_string[MAX_MSG_SIZE];
	//Sprintf: interi e buffer -> stringa
	sprintf(avg_sensor_string, "%d", shared_avg_sensor.value);
	sprintf(control_string, "%d", shared_control.value);
	int index = 0;
	for(int i=0; i<BUF_SIZE; i++){
		index += sprintf(&buffer_string[index], "%d", shared_buffer.b[i]);
	}
	sprintf(reference_string, "%d", shared_reference.value);
	
	/*Svuoto il messaggio all'inizio e concateno ogni stringa convertita con la sprintf al messaggio che invierò poi alla coda res_ds.
	Abbiamo visto anche da altri colleghi soluzioni più smart di questa ma per onestà intellettuale abbiamo lasciato la nostra, che
	seppur hard coded funziona. "If it's stupid but it works, it isn't stupid". */
	strcpy(message, "");
	strcat(message, avg_sensor_string);
	strcat(message, " ");
	strcat(message, control_string);
	strcat(message, " ");
	strcat(message, buffer_string);
	strcat(message, " ");
	strcat(message, reference_string);
	
	//Invio alla coda delle risposte del diag il messaggio finale ottenuto dalla concat di tutti i valori prelevati dal DS.
	//printf("\n\nDeferrable server: %s\n\n",message);
	if (mq_send (*res_ds, message, strlen (message) + 1, priority) == -1) {
		perror ("diag_fun: Not able to send message to diag");
		return -1;
	}
	busy_sleep(wcet);
	return 0;
}

void* ds(void* parameter){

	printf("\n\nSTART DS\n\n");

	periodic_thread *th = (periodic_thread *) parameter;
	start_periodic_timer(th,0);

	static mqd_t req_ds;
	static mqd_t res_ds;
	
	// Messaggio da inviare al diag
	char message [] = " ";
	// Messaggio che verrà ricevuto da diag 
	char in_buffer [MAX_MSG_SIZE];
		
	/* Queues */
	struct mq_attr attr;

	attr.mq_flags = 0;				
	attr.mq_maxmsg = MAX_MESSAGES;	
	attr.mq_msgsize = MAX_MSG_SIZE; 
	attr.mq_curmsgs = 0;
	
	// Apriamo una coda in sola lettura (O_RDONLY), se non esiste la creiamo (O_CREAT). Inoltre la coda sarà non bloccante (O_NONBLOCK)
	// La coda riceverà le richieste aperiodiche di diagnostica
	if ((req_ds = mq_open (DS_REQ_QUEUE_NAME, O_RDONLY | O_CREAT | O_NONBLOCK, QUEUE_PERMISSIONS, &attr)) == -1) {
		perror ("Server: mq_open (server)");
		exit (1);
	}
	
	// Apriamo una coda in sola scrittura (O_WRONLY)
	// La coda serve ad inviare messaggi al diag per la stampa a video
	if ((res_ds = mq_open (DS_RES_QUEUE_NAME, O_WRONLY | O_CREAT, QUEUE_PERMISSIONS, &attr)) == -1) {
		perror ("Client: mq_open (server)");
		exit (1);
	}

	//Definisco un timer per gestire la capacità del DS nella timed receive.
	struct timespec timer;
	clock_gettime(CLOCK_REALTIME, &timer);
	timer.tv_sec+=TICK_TIME/2;

	while (1) {
		wait_next_activation(th);
		//printf("[%d] DEFERRABLE SERVER with period:%d\t priority:%d\t wcet:%d\n",th->index,th->period,th->priority,th->wcet);	//DEBUG

		if (mq_timedreceive(req_ds, in_buffer,MAX_MSG_SIZE,NULL, &timer) == -1){
			//printf ("No message ...\n");							//DEBUG
		}
		else{
			printf ("Deferrable Server: message received:\n");			//DEBUG
			diag_fun(th->wcet,&res_ds,message,th->priority);
		}
	}

	if (mq_close (res_ds) == -1) {
		perror ("rt_ds: mq_close res_ds");
		exit (1);
	}

	if (mq_close (req_ds) == -1) {
		perror ("rt_ds: mq_close req_ds");
		exit (1);
	}
		
}


int main(void)
{
	printf("The controller is STARTED! [press 'q' to stop]\n");
 	
	pthread_t acquire_filter_thread;
    pthread_t control_thread;
    pthread_t actuator_thread;

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

	pthread_attr_destroy(&myattr);

	// DS THREAD
	periodic_thread th_ds;
	pthread_t thread_ds;
	th_ds.wcet = 1*TICK_TIME; 
	th_ds.period = TICK_TIME/2;
	th_ds.priority = 5;	
	myparam.sched_priority = th_ds.priority;
	pthread_attr_setschedparam(&myattr, &myparam); 	
	pthread_create(&thread_ds, &myattr, ds, &th_ds);
	pthread_attr_destroy(&myattr);
	
	
	/* Wait user exit commands*/
	while (1) {
   		if (getchar() == 'q') break;
  	}
	keep_on_running = 0;

	/* Clean */
  	pthread_kill(thread_ds,0);

	if (mq_unlink (DS_REQ_QUEUE_NAME) == -1) {
		perror ("rt_ds: mq_unlink ds req queue");
		exit (1);
	}

	if (mq_unlink (DS_RES_QUEUE_NAME) == -1) {
		perror ("rt_ds: mq_unlink ds res queue");
		exit (1);
	}

 	printf("The controller is STOPPED\n");
	return 0;
}




