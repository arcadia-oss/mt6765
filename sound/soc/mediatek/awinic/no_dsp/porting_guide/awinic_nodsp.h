#ifndef __AWINIC_NODSP_H__
#define __AWINIC_NODSP_H__

int aw_nodsp_open(void);
int aw_nodsp_write(int fd, const char  *buf, int count);
void aw_nodsp_close(int fd);

#endif

