#include "sd.h"

#include <disk/disk_access.h>
#include <fs/fs.h>
#include <ff.h>
#include <logging/log.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

#define MAX_FILENAME_LEN	12
#define MAX_ABSPATH_LEN		16
#define FS_END_OF_DIR		0

enum {
	SD_DISK_MNT_FAIL = -1,
};

LOG_MODULE_REGISTER(sd);

static const char *disk_mnt = "/SD:";
static FATFS fat_fs;

static struct fs_mount_t mp = {
	.type = FS_FATFS,
	.fs_data = &fat_fs,
};

static char curfile[16];
static struct fs_file_t fp;
static int sd_status = -1;

void
sd_close(void)
{
	fs_close(&fp);
	memcpy(curfile, 0, MAX_ABSPATH_LEN);
}

int
sd_open(const char *tmpl, unsigned int opt)
{
	int status;
	int idx = 0;
	int nchar = strlen(disk_mnt) + MAX_FILENAME_LEN;
	char fname[MAX_FILENAME_LEN];
	char tmpfile[nchar];
	struct fs_dirent entry;

	memset(fname, 0, MAX_FILENAME_LEN);
	memset(tmpfile, 0, nchar);

	while (1) {
		fs_file_t_init(&fp);
		sprintf(fname, tmpl, idx);
		sprintf(tmpfile, "%s/%s", disk_mnt, fname);
		status = fs_stat(tmpfile, &entry);
		if (status != -ENOENT) {
			idx++;
			continue;
		} else if ((status < 0) && (status != -ENOENT)) {
			printk("File system Error [%i]", status);
			return -1;
		}

		break;
	}

	status = fs_open(&fp, tmpfile, opt);
	if (status < 0) {
		printk("Error opening %s [E:%d]\n", fname, status);
		return status;
	}

	printk("Opening %s\n", tmpfile);
	strcpy(curfile, tmpfile);

	return 0;
}

void
sd_save(void *data, size_t size)
{
	fs_write(&fp, data, size);
}

void 
sd_init(void)
{
	static const char *disk_pdrv = "SD";
	uint64_t memory_size;
	uint32_t block_count;
	uint32_t block_size;

	do {
		if (disk_access_init(disk_pdrv) != 0) {
			LOG_ERR("SD: Initialization error!");
			break;
		}

		if (disk_access_ioctl(disk_pdrv, DISK_IOCTL_GET_SECTOR_COUNT,
			&block_count)) {
			LOG_ERR("SD: Unable to get sector count");
			break;
		}
		LOG_INF("Block count %u", block_count);

		if (disk_access_ioctl(disk_pdrv, DISK_IOCTL_GET_SECTOR_SIZE, 
			&block_size)) {
			LOG_ERR("SD: Unable to get sector size");
			break;
		}
		LOG_DBG("Sector size %u\n", block_size);
	} while (0);

	memory_size = (uint64_t)block_count * block_size;
	/* >> 10 is the same as divide by 1000 but faster
	 * >> 20 is the same as divide by 1000*1000 or 1000000
	 */
	LOG_DBG("Memory Size is %u MB\n", (uint32_t)(memory_size >> 20));

	mp.mnt_point = disk_mnt;

	sd_status = fs_mount(&mp);
	if (sd_status == FR_OK) {
		printk("microSD disk mounted on %s.\n", disk_mnt);
		/* List contents of microSD for debug output */
		sd_ls(disk_mnt);
	} else {
		printk("Error mounting microSD disk.\n");
	}
}

int 
sd_ls(const char *path)
{
	int status;
	struct fs_dir_t dirp;
	static struct fs_dirent entry;

	fs_dir_t_init(&dirp);

	status = fs_opendir(&dirp, path);
	if (status) {
		printk("Error opening dir %s [E:%d]\n", path, status);
		return status;
	}

	printk("Contents of %s\n", path);
	for (;;) {
		status = fs_readdir(&dirp, &entry);
		if (status || entry.name[0] == FS_END_OF_DIR) {
			break;
		}

		if (entry.type == FS_DIR_ENTRY_DIR) {
			printk("[DIR ] %s\n", entry.name);
		} else {
			printk("[FILE] %s (size = %zu)\n", entry.name, entry.size);
		}
	}

	fs_closedir(&dirp);

	return status;
}
