#include "awinic_nodsp.h"
#include <fcntl.h>
#include <unistd.h>
#include <log/log.h>

int aw_nodsp_open(void)
{
	int fd;
	//ALOGD("%s: enter Awinic open\n",__func__);
	fd = TEMP_FAILURE_RETRY(open("/sys/bus/i2c/drivers/aw87xxx_pa/1-0058/actflag", O_RDWR));
	return fd;
}

int aw_nodsp_write(int fd, const char * buf, int count)
{
	int val;
	//ALOGD("%s: enter Awinic write\n",__func__);
	val = TEMP_FAILURE_RETRY(write(fd, buf, count));
	return val;
}

void aw_nodsp_close(int fd)
{
	ALOGD("%s: enter Awinic close fd\n",__func__);
	close(fd);
}


