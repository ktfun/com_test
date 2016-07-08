#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <errno.h>
#include <termios.h>

typedef unsigned char			BYTE;
typedef unsigned short			WORD;
typedef signed int				LONG;
typedef unsigned int			DWORD;

typedef struct tagCOMM_ATTR
 {
	DWORD	baudrate;		
	BYTE	databits;
	BYTE	parity;
	BYTE	stopbits;	
	BYTE	reserved;
} COMM_ATTR;


enum comm_stopbits_t 
{
	COMM_ONESTOPBIT,
	COMM_ONE5STOPBITS,
	COMM_TWOSTOPBITS,
};

enum comm_parity_t 
{
	COMM_NOPARITY,
	COMM_ODDPARITY,
	COMM_EVENPARITY,
	COMM_MARK,
	COMM_SPACE,
};


#define COMM_PURGE_TXABORT			0x0001	
#define COMM_PURGE_RXABORT			0x0002	
#define COMM_PURGE_TXCLEAR			0x0004	
#define COMM_PURGE_RXCLEAR			0x0008	


#define dprintfbin(buf, size) 					\
	do {											\
		int i; 										\
		for (i = 0; i < size - 1; i++){ 				\
			if (0 == i % 16){ 						\
				if (0 != i)							\
					printf("\n");					\
				printf("0x%04x: ", i); 			\
			}										\
			printf("%02x ", ((char*)buf)[i]);		\
		}											\
		printf("%02x\n", ((char*)buf)[i]); 			\
	} while(0)

#define MAX(a, b)		(a > b? a: b)

#define ERR_PRINT		perror
#define LIBDVR_PRINT		printf

static int fd_com = -1;
char *DEV_COM = NULL;


int CommOpen(void)
{	
	if (fd_com < 0)
	{
		fd_com = open(DEV_COM, O_RDWR);
		if (fd_com < 0)
		{
			ERR_PRINT("open com dev\n");
			return -1;
		}
	}
	//LIBDVR_PRINT("CommOpen Successful\n");
	return fd_com;
}


int CommDestory(void)
{
	int ret = -1;

	if(fd_com > 0)
	{
		printf("close start!\n");
		ret = close(fd_com);
		printf("close end!\n");
		fd_com = -1;		
	}

	return ret;
}



int SetAttribute(COMM_ATTR *pattr)
{
	int dev_fd = fd_com;
	struct termios opt;
	COMM_ATTR *attr = pattr;
	
	if (dev_fd < 0)
	{
		return -1;
	}

	memset(&opt, 0, sizeof(struct termios));
	tcgetattr(dev_fd, &opt);
	cfmakeraw(&opt);			

	
	printf("set baudrate %d\n", attr->baudrate);
	switch (attr->baudrate)
	{
		case 50:
			cfsetispeed(&opt, B50);
			cfsetospeed(&opt, B50);
			break;
		case 75:
			cfsetispeed(&opt, B75);
			cfsetospeed(&opt, B75);
			break;
		case 110:
			cfsetispeed(&opt, B110);
			cfsetospeed(&opt, B110);
			break;
		case 134:
			cfsetispeed(&opt, B134);
			cfsetospeed(&opt, B134);
			break;
		case 150:
			cfsetispeed(&opt, B150);
			cfsetospeed(&opt, B150);
			break;
		case 200:
			cfsetispeed(&opt, B200);
			cfsetospeed(&opt, B200);
			break;
		case 300:
			cfsetispeed(&opt, B300);
			cfsetospeed(&opt, B300);
			break;
		case 600:
			cfsetispeed(&opt, B600);
			cfsetospeed(&opt, B600);
			break;
		case 1200:
			cfsetispeed(&opt, B1200);
			cfsetospeed(&opt, B1200);
			break;
		case 1800:
			cfsetispeed(&opt, B1800);
			cfsetospeed(&opt, B1800);
			break;
		case 2400:
			cfsetispeed(&opt, B2400);
			cfsetospeed(&opt, B2400);
			break;
		case 4800:
			cfsetispeed(&opt, B4800);
			cfsetospeed(&opt, B4800);
			break;
		case 9600:
			cfsetispeed(&opt, B9600);
			cfsetospeed(&opt, B9600);
			break;
		case 19200:
			cfsetispeed(&opt, B19200);
			cfsetospeed(&opt, B19200);
			break;
		case 38400:
			cfsetispeed(&opt, B38400);
			cfsetospeed(&opt, B38400);
			break;
		case 57600:
			cfsetispeed(&opt, B57600);
			cfsetospeed(&opt, B57600);
			break;
		case 115200:
			cfsetispeed(&opt, B115200);
			cfsetospeed(&opt, B115200);
			break;
		case 230400:
			cfsetispeed(&opt, B230400);
			cfsetospeed(&opt, B230400);
			break;
		case 460800:
			cfsetispeed(&opt, B460800);
			cfsetospeed(&opt, B460800);
			break;
		case 500000:
			cfsetispeed(&opt, B500000);
			cfsetospeed(&opt, B500000);
			break;
		case 576000:
			cfsetispeed(&opt, B576000);
			cfsetospeed(&opt, B576000);
			break;
		case 921600:
			cfsetispeed(&opt, B921600);
			cfsetospeed(&opt, B921600);
			break;
		case 1000000:
			cfsetispeed(&opt, B1000000);
			cfsetospeed(&opt, B1000000);
			break;
		case 1152000:
			cfsetispeed(&opt, B1152000);
			cfsetospeed(&opt, B1152000);
			break;
		case 1500000:
			cfsetispeed(&opt, B1500000);
			cfsetospeed(&opt, B1500000);
			break;
		case 2000000:
			cfsetispeed(&opt, B2000000);
			cfsetospeed(&opt, B2000000);
			break;
		case 2500000:
			cfsetispeed(&opt, B2500000);
			cfsetospeed(&opt, B2500000);
			break;
		case 3000000:
			cfsetispeed(&opt, B3000000);
			cfsetospeed(&opt, B3000000);
			break;
		case 3500000:
			cfsetispeed(&opt, B3500000);
			cfsetospeed(&opt, B3500000);
			break;
		case 4000000:
			cfsetispeed(&opt, B4000000);
			cfsetospeed(&opt, B4000000);
			break;
		default:
			LIBDVR_PRINT("unsupported baudrate %d\n", attr->baudrate);
			break;
	}

	
	switch (attr->parity)
	{
		case COMM_NOPARITY:		
			opt.c_cflag &= ~PARENB;	
			opt.c_iflag &= ~INPCK;	
			break;
		case COMM_ODDPARITY:		
			opt.c_cflag |= PARENB;	
			opt.c_cflag |= PARODD;	
			opt.c_iflag |= INPCK;	
			break;
		case COMM_EVENPARITY:		
			opt.c_cflag |= PARENB;	
			opt.c_cflag &= ~PARODD;	
			opt.c_iflag |= INPCK;	
		default:
			LIBDVR_PRINT("unsupported parity %d\n", attr->parity);
			break;
	}

	
	opt.c_cflag &= ~CSIZE;
	switch (attr->databits)
	{
		case 5:
			opt.c_cflag |= CS5;
			break;
		case 6:
			opt.c_cflag |= CS6;
			break;
		case 7:
			opt.c_cflag |= CS7;
			break;
		case 8:
			opt.c_cflag |= CS8;
			break;
		default:
			LIBDVR_PRINT("unsupported data bits %d\n", attr->databits);
			break;
	}

	
	opt.c_cflag &= ~CSTOPB;
	switch (attr->stopbits)
	{
		case COMM_ONESTOPBIT:
			opt.c_cflag &= ~CSTOPB;
			break;

		case COMM_TWOSTOPBITS:
			opt.c_cflag |= CSTOPB;
			break;
		default:
			LIBDVR_PRINT("unsupported stop bits %d\n", attr->stopbits);
			break;
	}
	opt.c_cc[VTIME]	= 0;
	opt.c_cc[VMIN]	= 1;			

	tcflush(dev_fd, TCIOFLUSH);
	if (tcsetattr(dev_fd, TCSANOW, &opt) < 0)
	{
		ERR_PRINT("tcsetattr\n");
		return -1;
	}

	return 0;
} 



int GetAttribute(void)
{
	int dev_fd = fd_com;
	struct termios opt;

	if (dev_fd < 0)
	{
		return -1;
	}

	memset(&opt, 0, sizeof(struct termios));
	tcgetattr(dev_fd, &opt);
	cfmakeraw(&opt);			

	printf("cfgetospeed(&opt): %d\n", cfgetospeed(&opt));
	printf("cfgetispeed(&opt): %d\n", cfgetispeed(&opt));


	printf("B2400 is %d, B115200 is %d, c_oflag is 0x%x\n", B2400, B115200, opt.c_oflag);



	return 0;
} 



int CommRead(void *pdata, DWORD nbytes)
{	
	if (fd_com < 0)
	{
		fd_com = open(DEV_COM, O_RDWR);
		if (fd_com < 0)
		{
			ERR_PRINT("Can't Open Com Dev");
			return -1;
		}		
	}
	//printf("start to read!\n");
	return read(fd_com, pdata, nbytes);
}


int CommWrite(void *pdata, DWORD len)
{	
	if (fd_com < 0)
	{
		fd_com = open(DEV_COM, O_RDWR);
		if (fd_com < 0)
		{
			ERR_PRINT("Can't Open Com Dev");
			return -1;
		}		
	}
	
	printf("start to write!\n");
	return write(fd_com, pdata, len);
}


int CommPurge(DWORD dw_flags)
{
	if (fd_com < 0)
	{
		fd_com = open(DEV_COM, O_RDWR);
		if (fd_com < 0)
		{
			ERR_PRINT("Can't Open Com Dev");
			return -1;
		}		
	}

	switch (dw_flags)
	{
		case COMM_PURGE_TXABORT:
			tcflow(fd_com, TCOOFF);
			break;
		case COMM_PURGE_RXABORT:
			tcflow(fd_com, TCIOFF);
			break;
		case COMM_PURGE_TXCLEAR:
			tcflush(fd_com, TCOFLUSH);
			break;
		case COMM_PURGE_RXCLEAR:
			tcflush(fd_com, TCIFLUSH);
			break;
		default:
			LIBDVR_PRINT("Unknow flag\n");
			return -1;
	}

	return 0;
}

int main(int argc, char *argv[])
{
	printf("version: %s %s\n", __DATE__, __TIME__);
	if(argc < 4) {
usage:
		printf("usage:\n" \
			"\t%s /dev/ttyAMA1 read 10\n" \
			"\t%s /dev/ttyAMA1 write 0x12 0x13 0x14\n" \
			"", argv[0], argv[0]);
		return -1;
	}

	if(0 == strcmp(argv[2], "read") || 0 == strcmp(argv[2], "write") || 0 == strcmp(argv[2], "clear") ) {
	} else {
		goto usage;
	}

	DEV_COM = argv[1];

	if(CommOpen() < 0)
		return -1;
		
	GetAttribute();

	COMM_ATTR attr = {0 };
	attr.baudrate = 9600;
	attr.databits = 8;			// 8位数据位
	attr.parity = COMM_NOPARITY;		// 无奇偶效验	
	attr.stopbits = COMM_ONESTOPBIT;	// 1位停止位
	//attr.reserved = 0;
	if(SetAttribute(&attr) !=0)
		printf("set com attr failed\n");
		
	GetAttribute();

	if(0 == strcmp(argv[2], "read")){
		int num = atoi(argv[3]);
		if(num <= 0 || num > 1000) {
			printf("read num must more than 0, less than 1000\n");
			exit(-2);
		}

		char *pdata = malloc(num);
		int ret;
		while(1)
		{
			int index = 0;
			ret = CommRead(pdata, num);
			//printf("finish Reading!\n");
			//if(ret != num)
				//printf("warning: we want read %d char, only read %d char\n", num, ret);
			dprintfbin(pdata, MAX(ret, num));
			
			printf("ret-> %d\n---------data start---------\n", ret);
			for(index = 0; index < ret; index++)
			{
				printf("%c", pdata[index]);
			}
			printf("\n---------data end-----------\n");
		}
	}
	if(0 == strcmp(argv[2], "write")){
		char *pdata = malloc(1024);
		int num = argc - 3;
		int i;
		int ret;

		for(i = 0; i < num; i++)
			pdata[i] = (char)strtoul(argv[3 + i], NULL, 16);

		//while(1) {
			ret = CommWrite(pdata, num);
			printf("finish writing!\n");
			if(ret != num)
				printf("warning: we want write %d char, only write %d char\n", num, ret);
			//dprintfbin(pdata, MAX(ret, num));
			//sleep(1);
			usleep(1);
		//}
	}

	if (0 == strcmp(argv[2], "clear"))
	{
		// flush tx, rx
		CommPurge(COMM_PURGE_TXCLEAR);
		CommPurge(COMM_PURGE_RXCLEAR);
	}

	printf("***\n");
	CommDestory();
	return 0;
}



