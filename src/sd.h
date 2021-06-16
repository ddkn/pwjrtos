#ifndef _SD_H_
#define _SD_H_

#include <stddef.h>

void sd_init();
int  sd_ls(const char *path);
int  sd_open(const char *tmpl, unsigned int opt);
void sd_close();
void sd_save(void *data, size_t size);

#endif /* _SD_H_ */
