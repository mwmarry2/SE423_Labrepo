#include <stdio.h>
#include <stdlib.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <string.h>
#include <unistd.h>
#include <semaphore.h>
#include <sys/mman.h>
#include <signal.h>
#include <pthread.h>
#include <sched.h> // for sched_yield()
#include <assert.h>
#include <sys/resource.h>
#include <math.h>
#include <dirent.h>
#include <sys/time.h>
#include <errno.h>
#include <stdint.h>
#include <sys/signal.h>
#include <termios.h>
#include <ctype.h>

int gs_quit = 0;
int gs_exit = 0;

float vref = 1.0;
float turn = 0.0;
float refRightWall = 0.0;
float leftTurnStartThreshold = 0.0;
float LeftTurnStopThreshold = 0.0;
float KPFrontWall = 0.0;
float KPRightWall = 0.0;
float FrontTurnVelocity = 0.0;
float ForwardVelocityRW = 0.0;
float TurnCommandSaturation = 0.0;
int sem_count_send = 0; //for sem_getvalue
#define CMDNUM_FROM_FLOATS 11
#define RECVFROM_LINUXCMDAPP_SEM_MUTEX_NAME "/sem-LINUXCMDApp-recvfrom"
#define RECVFROM_LINUXCMDAPP_SHARED_MEM_NAME "/sharedmem-LINUXCMDApp-recvfrom"

typedef union {
    char data_char[4*CMDNUM_FROM_FLOATS];
    float data_flts[CMDNUM_FROM_FLOATS];
} int_FromCMD_union;

struct shared_memory_recvfrom_LINUXCMDApp
{
  int_FromCMD_union new_FromCMD;
};

struct shared_memory_recvfrom_LINUXCMDApp *shared_mem_ptr_recvfrom_LINUXCMDApp;
sem_t *recvfrom_LINUXCMDApp_mutex_sem;
int recvfrom_LINUXCMDApp_fd_shm;


int mygetch(void)
{
	struct termios oldt,
	newt;
	int ch;
	tcgetattr( STDIN_FILENO, &oldt );
	newt = oldt;
	newt.c_lflag &= ~( ICANON | ECHO );
	tcsetattr( STDIN_FILENO, TCSANOW, &newt );
	ch = getchar();
	tcsetattr( STDIN_FILENO, TCSANOW, &oldt );
	return ch;
}


// Print system error and exit
void error (char *msg)
{
    perror (msg);
    exit (1);
}

/*  
* gs_killapp()
*   ends application safely
*
*/
void gs_killapp(int s)
{
	printf("\nTerminating\n");
	gs_quit = 1;
	gs_exit = 1;
	return;
}

/*
* main()
*   process command line input
*/
int main (int argc, char **argv)
{
	int i = 0;

	char buffer[200];  // used by fgets to read character string typed by user.
	char mychar;
	float tempfloat = 0;
	
	fflush(stdout);
	 
  	
    //create the semaphore for recv 
    if ((recvfrom_LINUXCMDApp_mutex_sem = sem_open(RECVFROM_LINUXCMDAPP_SEM_MUTEX_NAME, 0, 0, 0)) == SEM_FAILED)
        error("Error recv LINUXCMDApp sem_open");

    // create shared memory for recv
    if ((recvfrom_LINUXCMDApp_fd_shm = shm_open(RECVFROM_LINUXCMDAPP_SHARED_MEM_NAME, O_RDWR, 0)) == -1)
        error("Error shm_open LINUXCMDApp");

    //map the memory to virtual address
    if ((shared_mem_ptr_recvfrom_LINUXCMDApp = mmap(NULL, sizeof(struct shared_memory_recvfrom_LINUXCMDApp), PROT_READ | PROT_WRITE, MAP_SHARED,
                                recvfrom_LINUXCMDApp_fd_shm, 0)) == MAP_FAILED)
        error("Error mmap LINUXCMDApp");

	printf("Setting signal handler...\n");
	signal(SIGKILL, gs_killapp);
	signal(SIGINT, gs_killapp);
	printf("...OK\n");
	printf(".\n");
	while (!gs_exit) {
		sched_yield();

		printf("\n\n");
		printf("Menu of Selections\n");
		printf("DO NOT PRESS CTRL-C when in this menu selection\n");
		printf("e - Exit Application\n");
		printf("s - enter Desired Velocity Setpoint and RightWall speed (ft/s)\n");
		printf("q - increment Left\n");
		printf("p - increment Right\n");
		printf("w - Right Wall Reference\n");
		printf("u - Left Turn Start Threshold\n");
		printf("i - Left Turn Stop Threshold\n");
		printf("y - KP Right Wall\n");
		printf("o - KP Front Wall\n");
		printf("k - Front Turn Velocity\n");
		printf("j - Forward Velocity\n");
		printf("h - Turn Command Saturation\n");
		printf("l - List All Parameters\n");

		mychar = (char) mygetch();
		
		switch (mychar) {
		case 'q':
			if (turn > 0.0) {
				turn = 0.0;
			} else {
				turn = turn - 0.2;
			}
			printf("turn =%.3f\n",turn);
			break;
		case 'p':                                
			if (turn < 0.0) {
				turn = 0.0;
			} else {
				turn = turn + 0.2;
			}
			printf("turn =%.3f\n",turn);
			break;
		case 'e':
			gs_exit = 1;
			break;
		case 's':
			printf("Enter Desired Velocity (vref) and Right Wall velocity\n");
			fgets(buffer,190,stdin); 
			buffer[strlen(buffer)-1] = '\0';  // get ride of '\n' in returned string
			if (buffer[0] != '\0') {
				if (sscanf(buffer,"%f",&tempfloat) != 0) {  // check that it was a number entered
					vref = tempfloat;
					printf("DVel = %.3f\n",vref);
				}  else {
					printf("Error: Non numerical value typed\n");
				}
			} else {
				printf("Error: vref not changed\n");
			}
			
			break;

		case 'w':
			printf("Enter Right Wall Reference Value\n");
			fgets(buffer,190,stdin); 
			buffer[strlen(buffer)-1] = '\0';  // get ride of '\n' in returned string
			if (buffer[0] != '\0') {
				if (sscanf(buffer,"%f",&tempfloat) != 0) {  // check that it was a number entered
					refRightWall = tempfloat;
					printf("refRightWall = %.3f\n",refRightWall);
				}  else {
					printf("Error: Non numerical value typed\n");
				}
			} else {
				printf("Error: refRightWall not changed\n");
			}
			
			break;
		case 'y':
			printf("Enter Desired Velocity (KPRightWall) and Right Wall velocity\n");
			fgets(buffer,190,stdin); 
			buffer[strlen(buffer)-1] = '\0';  // get ride of '\n' in returned string
			if (buffer[0] != '\0') {
				if (sscanf(buffer,"%f",&tempfloat) != 0) {  // check that it was a number entered
					KPRightWall = tempfloat;
					printf("KPRightWall = %.3f\n",KPRightWall);
				}  else {
					printf("Error: Non numerical value typed\n");
				}
			} else {
				printf("Error: KPRightWall not changed\n");
			}
			
			break;	
		case 'u':
			printf("Enter Desired Velocity (leftTurnStartThreshold) and Right Wall velocity\n");
			fgets(buffer,190,stdin); 
			buffer[strlen(buffer)-1] = '\0';  // get ride of '\n' in returned string
			if (buffer[0] != '\0') {
				if (sscanf(buffer,"%f",&tempfloat) != 0) {  // check that it was a number entered
					leftTurnStartThreshold = tempfloat;
					printf("leftTurnStartThreshold = %.3f\n",leftTurnStartThreshold);
				}  else {
					printf("Error: Non numerical value typed\n");
				}
			} else {
				printf("Error: leftTurnStartThreshold not changed\n");
			}
			
			break;	
		case 'i':
			printf("Enter Desired Velocity (LeftTurnStopThreshold) and Right Wall velocity\n");
			fgets(buffer,190,stdin); 
			buffer[strlen(buffer)-1] = '\0';  // get ride of '\n' in returned string
			if (buffer[0] != '\0') {
				if (sscanf(buffer,"%f",&tempfloat) != 0) {  // check that it was a number entered
					LeftTurnStopThreshold = tempfloat;
					printf("LeftTurnStopThreshold = %.3f\n",LeftTurnStopThreshold);
				}  else {
					printf("Error: Non numerical value typed\n");
				}
			} else {
				printf("Error: LeftTurnStopThreshold not changed\n");
			}
			
			break;	
		case 'o':
			printf("Enter Desired Velocity (KPFrontWall) and Right Wall velocity\n");
			fgets(buffer,190,stdin); 
			buffer[strlen(buffer)-1] = '\0';  // get ride of '\n' in returned string
			if (buffer[0] != '\0') {
				if (sscanf(buffer,"%f",&tempfloat) != 0) {  // check that it was a number entered
					KPFrontWall = tempfloat;
					printf("KPFrontWall = %.3f\n",KPFrontWall);
				}  else {
					printf("Error: Non numerical value typed\n");
				}
			} else {
				printf("Error: KPFrontWall not changed\n");
			}
			
			break;	
		case 'k':
			printf("Enter Desired Velocity (FrontTurnVelocity) and Right Wall velocity\n");
			fgets(buffer,190,stdin); 
			buffer[strlen(buffer)-1] = '\0';  // get ride of '\n' in returned string
			if (buffer[0] != '\0') {
				if (sscanf(buffer,"%f",&tempfloat) != 0) {  // check that it was a number entered
					FrontTurnVelocity = tempfloat;
					printf("FrontTurnVelocity = %.3f\n",FrontTurnVelocity);
				}  else {
					printf("Error: Non numerical value typed\n");
				}
			} else {
				printf("Error: FrontTurnVelocity not changed\n");
			}
			
			break;		
		case 'j':
			printf("Enter Desired Velocity (ForwardVelocityRW) and Right Wall velocity\n");
			fgets(buffer,190,stdin); 
			buffer[strlen(buffer)-1] = '\0';  // get ride of '\n' in returned string
			if (buffer[0] != '\0') {
				if (sscanf(buffer,"%f",&tempfloat) != 0) {  // check that it was a number entered
					ForwardVelocityRW = tempfloat;
					printf("ForwardVelocityRW = %.3f\n",ForwardVelocityRW);
				}  else {
					printf("Error: Non numerical value typed\n");
				}
			} else {
				printf("Error: ForwardVelocityRW not changed\n");
			}
			
			break;
		case 'h':
			printf("Enter Desired TurnCommandSaturation\n");
			fgets(buffer,190,stdin); 
			buffer[strlen(buffer)-1] = '\0';  // get ride of '\n' in returned string
			if (buffer[0] != '\0') {
				if (sscanf(buffer,"%f",&tempfloat) != 0) {  // check that it was a number entered
					TurnCommandSaturation = tempfloat;
					printf("TurnCommandSaturation = %.3f\n",TurnCommandSaturation);
				}  else {
					printf("Error: Non numerical value typed\n");
				}
			} else {
				printf("Error: TurnCommandSaturation not changed\n");
			}
			
			break;				
		case 'l':
			printf("\n");
			printf("turn = %.3f\n",turn);
			printf("vref = %.3f\n",vref);
			printf("refRightWall= %.3f\n",refRightWall);
			printf("leftTurnStartThreshold= %.3f\n",leftTurnStartThreshold);
			printf("LeftTurnStopThreshold= %.3f\n",LeftTurnStopThreshold);
			printf("KPFrontWall= %.3f\n",KPFrontWall);
			printf("KPRightWall= %.3f\n",KPRightWall);
			printf("FrontTurnVelocity= %.3f\n",FrontTurnVelocity);
			printf("ForwardVelocityRW= %.3f\n",ForwardVelocityRW);
			printf("TurnCommandSaturation= %.3f\n",TurnCommandSaturation);
			break;	
		default:
			
			break;
		}
		
		shared_mem_ptr_recvfrom_LINUXCMDApp->new_FromCMD.data_flts[0] = vref;
        shared_mem_ptr_recvfrom_LINUXCMDApp->new_FromCMD.data_flts[1] = turn;
        shared_mem_ptr_recvfrom_LINUXCMDApp->new_FromCMD.data_flts[2] = refRightWall;
        shared_mem_ptr_recvfrom_LINUXCMDApp->new_FromCMD.data_flts[3] = leftTurnStartThreshold;
        shared_mem_ptr_recvfrom_LINUXCMDApp->new_FromCMD.data_flts[4] = LeftTurnStopThreshold;
        shared_mem_ptr_recvfrom_LINUXCMDApp->new_FromCMD.data_flts[5] = KPFrontWall;
        shared_mem_ptr_recvfrom_LINUXCMDApp->new_FromCMD.data_flts[6] = KPRightWall;
        shared_mem_ptr_recvfrom_LINUXCMDApp->new_FromCMD.data_flts[7] = FrontTurnVelocity;
        shared_mem_ptr_recvfrom_LINUXCMDApp->new_FromCMD.data_flts[8] = ForwardVelocityRW;
        shared_mem_ptr_recvfrom_LINUXCMDApp->new_FromCMD.data_flts[9] = TurnCommandSaturation;
        shared_mem_ptr_recvfrom_LINUXCMDApp->new_FromCMD.data_flts[10] = 0;

		if (sem_getvalue(recvfrom_LINUXCMDApp_mutex_sem,  &sem_count_send) == 0) {
			if (sem_post(recvfrom_LINUXCMDApp_mutex_sem) == -1){
				printf("Error LINUXCMDApp sem_post: recvfrom_mutex");
			}
		}
		
	}

}


