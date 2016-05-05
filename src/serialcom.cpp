/*******************************************************************************
* \file serialcom.c
* \brief  Implementa modulo basico de acesso � porta serial.
// Observa�oes:
//    - 
*******************************************************************************/
#include <stdio.h>
#include <math.h>
#include <unistd.h> /* for libc5 */
#include <sys/io.h> /* for glibc */
#include <fcntl.h>           /* For O_* constants */
#include <sys/stat.h>        /* For mode constants */
#include <semaphore.h>
#include <time.h>
#include <sys/time.h>

#include "serialcom.h" 

/*! \def SERIALCOM_COMPORTADDRESS_1 
* Endereco base da porta serial COM1. Uso interno. */
#define SERIALCOM_COMPORTADDRESS_1 0x3F8
/*! \def SERIALCOM_COMPORTADDRESS_2 
* Endereco base da porta serial COM2. Uso interno. */
#define SERIALCOM_COMPORTADDRESS_2 0x2F8
/*! \def SERIALCOM_COMPORTADDRESS_3 
* Endereco base da porta serial COM3. Uso interno. */
#define SERIALCOM_COMPORTADDRESS_3 0x3E8
/*! \def SERIALCOM_COMPORTADDRESS_4 
* Endereco base da porta serial COM4. Uso interno. */
#define SERIALCOM_COMPORTADDRESS_4 0x2E8

/*! \var SEM *pComPortSemaphores[4] 
* Vetor de ponteiros para semaforos. Cada elemento desse vetor eh um ponteiro para o semaforo associado aa porta X, com X = 1, 2, 3 ou 4. Os semaforos de cada porta sao iniciados na chamada aa funcao serialcom_init(). Para uso interno pelas funcoes serialcom_semwait() e serialcom_semsignal(). */
sem_t *pComPortSemaphores[4] = {NULL, NULL, NULL, NULL}; 

// funcao de uso interno apenas
inline void serialcom_delayus(double timeus)
{
	struct timeval     timereset;
	struct timeval     time;
 
 	gettimeofday(&timereset, NULL);
	do  {
		usleep(timeus/4.0);
		gettimeofday(&time, NULL);
	} while (((time.tv_sec - timereset.tv_sec)*1e6 + (time.tv_usec - timereset.tv_usec)) < timeus);
}

/************* Rotinas Genericas de Manipulacao da porta serial com acesso externo *****************/
/*! \fn int serialcom_init(PSERIALPORTCONFIG pSerialPortConfig, int ComPortNumber, unsigned long int ComPortBPS)
* Funcao que inicia a porta serial ComPortNumber com a taxa dada em BPS por ComPortBPS. Essa funcao dever ser chamada por cada thread 
* que tenha acesso  porta serial ComPortNumber. Seus argumentos de chamada so:
* \param pSerialPortConfig Ponteiro para estrutura SERIALPORCONFIG que guarda informaes de configurao da porta serial no 
* contexto do thread de chamada. Mesmo que uma dada porta serial seja utilizada por diversos threads, cada thread dever ter a sua 
* estrutura SERIALPORCONFIG.
* \param ComPortNumber Numero da porta serial, no intervalo de 1 a 4.
* \param ComPortBPS Taxa de comunicao em BPS, no intervalo de 2 a 115200.
* \return SERIALCOM_SUCCESS : Porta iniciada com sucesso. 
* \return SERIALCOM_ERROR_INCORRECTPORTNUMBER : Erro, corresponde a um ComPortNumber invlido.
* \return SERIALCOM_ERROR_MAXBPSPRECISION : Erro, a taxa ComPortBPS no pode ser realizada com erro inferior a. SERIALCOM_MAXBPSPRECISION. 
* \return SERIALCOM_ERROR_IOPL : Erro, corresponde a uma tentativa de executar o programa sem que se tenha acesso privilegiado de administrador a portas de E/S. 
*/
int serialcom_init(PSERIALPORTCONFIG pSerialPortConfig, int ComPortNumber, char *pComPortDevice, unsigned long int ComPortBPS)
{
	#if SERIALCOM_USE_RS485
	int status;
	#endif
	char semname[50];
	int baudrate;

	strcpy(pSerialPortConfig->pComPortDevice, pComPortDevice);
	
	pSerialPortConfig->fd = open(pComPortDevice, O_RDWR ); 
	if (pSerialPortConfig->fd <0) {return(SERIALCOM_ERROR_INVALIDDEVICE); }

	tcgetattr(pSerialPortConfig->fd,&pSerialPortConfig->oldtio); /* save current port settings */

	switch(ComPortBPS){
		case 50:
			baudrate = B50;
			break;
		case 75:
			baudrate = B75;
			break;
		case 110:
			baudrate = B110;
			break;
		case 134:
			baudrate = B134;
			break;
		case 150:
			baudrate = B150;
			break;
		case 200:
			baudrate = B200;
			break;
		case 300:
			baudrate = B300;
			break;
		case 600:
			baudrate = B600;
			break;
		case 1200:
			baudrate = B1200;
			break;
		case 1800:
			baudrate = B1800;
			break;
		case 2400:
			baudrate = B2400;
			break;
		case 4800:
			baudrate = B4800;
			break;
		case 9600:
			baudrate = B9600;
			break;
		case 19200:
			baudrate = B19200;
			break;
		case 38400:
			baudrate = B38400;
			break;
		case 57600:
			baudrate = B57600;
			break;
		case 115200:
			baudrate = B115200;
			break;
		case 230400:
			baudrate = B230400;
			break;
		case 460800:
			baudrate = B460800;
			break;
		case 500000:
			baudrate = B500000;
			break;
		case 576000:
			baudrate = B576000;
			break;
		case 921600:
			baudrate = B921600;
			break;
		case 1000000:
			baudrate = B1000000;
			break;
		case 1152000:
			baudrate = B1152000;
			break;
		case 1500000:
			baudrate = B1500000;
			break;
		default:
			return SERIALCOM_ERROR_INVALIDBAUDRATE;
			break;
	}
	
	bzero((void*)&pSerialPortConfig->newtio, sizeof(pSerialPortConfig->newtio));
	pSerialPortConfig->newtio.c_cflag = baudrate | CS8 | CLOCAL | CREAD;
	pSerialPortConfig->newtio.c_iflag = IGNPAR;
	pSerialPortConfig->newtio.c_oflag = 0;

	/* set input mode (non-canonical, no echo,...) */
	pSerialPortConfig->newtio.c_lflag = 0;
	 
	pSerialPortConfig->newtio.c_cc[VTIME]    = 0;   /* inter-character timer unused */
	pSerialPortConfig->newtio.c_cc[VMIN]     = 1;   /* blocking read until 1 chars received */

	tcflush(pSerialPortConfig->fd, TCIFLUSH);
	tcsetattr(pSerialPortConfig->fd,TCSANOW,&pSerialPortConfig->newtio);

//		printf("\n speed set to %X\n\n", cfgetispeed (&pSerialPortConfig->newtio)); exit(1);
	
	pSerialPortConfig->ComPortBPS = ComPortBPS;
	pSerialPortConfig->ComPortNumber = ComPortNumber;
	pSerialPortConfig->FramePeriodUS = (1e7)/(((float)(ComPortBPS)));

	fcntl(pSerialPortConfig->fd, F_SETFL, FNDELAY); // read functin returns imediatly (non-blocking mode)

	#if SERIALCOM_USE_RS485
	ioctl(pSerialPortConfig->fd, TIOCMGET, &status); /* get the serial port status */
	status |= TIOCM_RTS;
	ioctl(pSerialPortConfig->fd, TIOCMSET, &status); //setar RTS = 0.
	#endif

	sprintf(semname,"CoSem%i",ComPortNumber);
	pComPortSemaphores[ComPortNumber-1] = sem_open(semname,O_CREAT,S_IRUSR|S_IWUSR,1);
//	printf("\n Semaforo %s: %X\n",semname, pComPortSemaphores[ComPortNumber-1]);

	return SERIALCOM_SUCCESS;
}

int serialcom_close(PSERIALPORTCONFIG pSerialPortConfig)
{
	sem_close(pComPortSemaphores[pSerialPortConfig->ComPortNumber-1]);

	return SERIALCOM_SUCCESS;
}

/*! \fn void serialcom_semwait(PSERIALPORTCONFIG pSerialPortConfig)
* Funcao que aguarda semforo para acessar a porta descrita por pSerialPortConfig. Juntamente com serialcom_semsignal, pode-se 
* garantir o acesso exclusivo de um thread  porta serial. 
* \param pSerialPortConfig Ponteiro para estrutura SERIALPORTCONFIG que guarda informaes de configurao da porta serial no 
* contexto do thread de chamada. Mesmo que uma dada porta serial seja utilizada por diversos threads, cada thread dever ter a sua 
* estrutura SERIALPORTCONFIG..
* \warning Se uma determinada porta somente  gerenciada por um s thread, no h necessidade de se usar essas funes de semforo. As 
* funes de semforo tm somente utilizade em situaes em que mais de um thread pode acessar a porta serial X, com X = 1, 2, 3 ou 4.
* \warning Aps concluir o acesso  porta serial cedido por essa funcao, deve-se chamar serialcom_semsignal para liberar o semforo
*/
void serialcom_semwait(PSERIALPORTCONFIG pSerialPortConfig)
{
	sem_wait(pComPortSemaphores[pSerialPortConfig->ComPortNumber-1]);
}

/*! \fn void serialcom_semsignal(PSERIALPORTCONFIG pSerialPortConfig)
* Funcao que libera semforo que foi previamente cedido por serialcom_semwait para acessar a porta descrita por pSerialPortConfig.  
* \param pSerialPortConfig Ponteiro para estrutura SERIALPORTCONFIG que guarda informaes de configurao da porta serial no 
* contexto do thread de chamada. Mesmo que uma dada porta serial seja utilizada por diversos threads, cada thread dever ter a sua 
* estrutura SERIALPORTCONFIG..
* \warning Se uma determinada porta somente  gerenciada por um s thread, no h necessidade de se usar essas funes de semforo. As 
* funes de semforo tm somente utilizade em situaes em que mais de um thread pode acessar a porta serial X, com X = 1, 2, 3 ou 4.
* \warning Aps concluir o acesso  porta serial cedido por essa funcao, deve-se chamar serialcom_semsignal para liberar o semforo
*/
void serialcom_semsignal(PSERIALPORTCONFIG pSerialPortConfig)
{
	sem_post(pComPortSemaphores[pSerialPortConfig->ComPortNumber-1]);
}

/*! \fn int serialcom_sendbyte(PSERIALPORTCONFIG pSerialPortConfig, unsigned char *pData)
* Funcao que envia um byte apontado por pData pela porta serial descrita por pSerialPortConfig.   
* \param pSerialPortConfig Ponteiro para estrutura SERIALPORTCONFIG que guarda informaes de configurao da porta serial no 
* contexto do thread de chamada. Mesmo que uma dada porta serial seja utilizada por diversos threads, cada thread dever ter a sua 
* estrutura SERIALPORTCONFIG. Se SERIALCOM_USE_RS485 = 1, ento o sinal RTS ser colocado em nvel lgico 1 enquanto durar o frame do 
* byte enviado, permitindo assim ativar o driver externo de uma porta com conversor RS-485. Nessa situao, essa funcao somente retorna 
* quando o byte tiver sido enviado. Caso contrrio, a funcao somente escrever no buffer de sada o byte apontado por pData, retornando em * seguida.
* \param pData Ponteiro para o byte que ser enviado.
* \return SERIALCOM_SUCCESS : Dado escrito no registro de sada com sucesso. Entretanto, isso significa apenas que uma transmisso est em curso. Para se certificar de que o dado foi efetivamente transmitido, deve-se fazer uso da funcao serialcom_status()
* \return SERIALCOM_ERROR_MAXWAITENDOFTRANSMISSION : Situao de erro em que a funcao ficou aguardando por um perodo de at 5 frames para disponibilizao do registro de sada da porta
* \warning Essa funcao fica bloqueada enquanto o ltimo byte escrito no buffer de sada ainda no tiver sido enviado.
*/
int serialcom_sendbyte(PSERIALPORTCONFIG pSerialPortConfig, unsigned char *pData)
{

	#if SERIALCOM_USE_RS485
	int status;
	#endif

	#if SERIALCOM_USE_RS485
	ioctl(pSerialPortConfig->fd, TIOCMGET, &status); /* get the serial port status */
	status &= ~TIOCM_RTS;
	ioctl(pSerialPortConfig->fd, TIOCMSET, &status); //setar RTS = 1.
	#endif
	
	if (write(pSerialPortConfig->fd, pData, 1) < 0) return SERIALCOM_ERROR_MAXWAITENDOFTRANSMISSION;
	
	#if SERIALCOM_USE_RS485
	ioctl(pSerialPortConfig->fd, TIOCMGET, &status); /* get the serial port status */
	status |= TIOCM_RTS;
	ioctl(pSerialPortConfig->fd, TIOCMSET, &status); //setar RTS = 0.
	#endif

	return SERIALCOM_SUCCESS;
}

/*! \fn serialcom_receivebyte(PSERIALPORTCONFIG pSerialPortConfig, unsigned char *pData, double MaximaEsperaUS)
* Funcao que aguarda um byte chegar pela porta serial descrita por pSerialPortConfig por um tempo mximo dado por MaximaEsperaUS, dado em 
* microsegundos. Se um dado chegar dentro do perodo dado por MaximaEsperaUS, o mesmo ser colocado na varivel apontada por pData. 
* \param pSerialPortConfig Ponteiro para estrutura SERIALPORTCONFIG que guarda informaes de configurao da porta serial no 
* contexto do thread de chamada.
* \param pData Ponteiro para o byte recebido.
* \param MaximaEsperaUS Tempo mximo de espera pela chegada de um byte pela porta. Se MaximaEsperaUS 
* \return SERIALCOM_SUCCESS : Operao realizada com sucesso. Um byte foi recebido pela porta serial e se encontra disponvel na varivel 
* apontada por pData.
* \return SERIALCOM_ERROR_MAXWAITFORRECEPTION : Nenhum bayte chegou dentro do tempo estipulado por MaximaEsperaUS
* \warning Essa funcao fica bloqueada por at MaximaEsperaUS enquanto um byte no chegar.
*/
int serialcom_receivebyte(PSERIALPORTCONFIG pSerialPortConfig, unsigned char *pData, double MaximaEsperaUS)
{
	double	ElapsedTime;
	int 	nbytesreceived;
	struct 	timeval timereset;
	struct 	timeval time;
 
	if(MaximaEsperaUS<0) MaximaEsperaUS = 0; // Espera minima de 0 us.

	ElapsedTime = 0.0;
	gettimeofday(&timereset, NULL);
	while(1)
	{
		nbytesreceived = read(pSerialPortConfig->fd,pData,1);
		if(nbytesreceived==1){
//			printf("\n FILE = %s, LINE = %i",__FILE__,__LINE__);
			return(SERIALCOM_SUCCESS);
		}
//		serialcom_delayus(0.2*pSerialPortConfig->FramePeriodUS);
		gettimeofday(&time, NULL);
		ElapsedTime = ((time.tv_sec - timereset.tv_sec)*1e6 + (time.tv_usec - timereset.tv_usec));
//			printf("\n ElapsedTime = %f",ElapsedTime);
		if(ElapsedTime >= MaximaEsperaUS){
//			printf("\n FILE = %s, LINE = %i",__FILE__,__LINE__);
//			printf("\n ElapsedTime = %f",ElapsedTime);
	//		printf("\n Porta %i, Retornando estado %i", pSerialPortConfig->ComPortNumber, SERIALCOM_ERROR_MAXWAITFORRECEPTION);
			return(SERIALCOM_ERROR_MAXWAITFORRECEPTION);  // Dado nao chegou no tempo estipulado.
		}
	}
}

/*! \fn int serialcom_status(PSERIALPORTCONFIG pSerialPortConfig)
* Funcao que l o registro de status da porta serial descrita por pSerialPortConfig. 
* \param pSerialPortConfig Ponteiro para estrutura SERIALPORTCONFIG que guarda informaes de configurao da porta serial no 
* contexto do thread de chamada.
* \return O valor de retorno tem os bits setados conforme que o dado foi efetivamente enviadoos eventos que ocorreram com a porta serial, que podem ser testados usando um 
* teste lgico E bit a bit com as seguintes mscaras: 
* \return SERIALCOM_STATUSMASK_ERROR_RX_FIFO  
* \return SERIALCOM_STATUSMASK_EMPTY_DH_REGISTERS 
* \return SERIALCOM_STATUSMASK_EMPTY_TX_REGISTER
* \return SERIALCOM_STATUSMASK_BREAK_INTERRUPT
* \return SERIALCOM_STATUSMASK_FRAMING_ERROR
* \return SERIALCOM_STATUSMASK_PARITY_ERROR
* \return SERIALCOM_STATUSMASK_OVERRUN_ERROR
* \return SERIALCOM_STATUSMASK_RX_DATA_READY
* \return As mscaras acima correspondem a eventos que so detalhados em http://www.beyondlogic.org/serial/serial.htm 
*/
	// undefined

/************* Rotinas Genericas de Manipulacao da porta serial com acesso interno *****************/

/*! \mainpage 
A biblioteca serialcom foi concebida para dar funcionalidade de comunicao serial para processos LINUX com a extenso de tempo real RTAI. Ela  disponibilizada na forma de cdigo fonte nos arquivos serialcom.c e serialcom.h. Essa biblioteca foi concebida para ser compatvel com processos com multiplos threads, e permite ainda que vrios threads acessem a mesma porta serial. No caso, esto implementadas funes para COM1, COM2, COM3 e COM4. E ainda, a biblioteca implementa funes de controle de acesso por semforo, o que permite que uma mesma porta serial possa ser acessada por um s thread por vez. Dependendendo do tipo de protocolo, o uso de semforos se faz necessrio. 

O projeto acompanha um exemplo no diretrio test. Para compilar o exemplo, basta fazer make. O resultado  o arquivo eval_serialcom. Antes de executar esse arquivo  necessrio pelo menos uma vez aps ter iniciado o sistema carregar os mdulos do RTAI. Para isso, basta executar o script loadmods.

*/


