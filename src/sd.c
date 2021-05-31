#include "sd.h"

#include <disk/disk_access.h>
#include <fs/fs.h>
#include <ff.h>
#include <logging/log.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

#define MAX_ABSPATH_LEN	16
#define FS_END_OF_DIR	0

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

static int
sd_open(const char *filename, uint32_t options)
{
	int status;
	int nchar = strlen(disk_mnt) + strlen(filename) + 1;
	char tmpfile[nchar];

	sprintf(tmpfile, "%s/%s", disk_mnt, filename);
	
	fs_file_t_init(&fp);
	status = fs_open(&fp, tmpfile, options);
	if (status < 0) {
		printk("Error opening %s [E:%d]\n", filename, status);
		return status;
	}
	printk("Opening %s\n", tmpfile);
	strcpy(curfile, tmpfile);

	return EXIT_SUCCESS;
}

static void
sd_close(void)
{
	fs_close(&fp);
	memcpy(curfile, 0, MAX_ABSPATH_LEN);
}

static int 
sd_io_test(const char *path)
{
	int status;
	int bytes;
	char line[100];

	status = sd_open(path, FS_O_READ);
	if (status < 0) {
		return status;
	}

	bytes = fs_read(&fp, line, 15);
	printk("%s [%i]: %s", curfile, bytes, line);
	sd_close();

	status = sd_open("bye.txt", FS_O_CREATE | FS_O_WRITE);
	if (status < 0) {
		return status;
	}

	bytes = fs_write(&fp, "Goodbye.txt", 8);
	printk("Wrote %i chars to %s\n", bytes, curfile);
	sd_close();

	return status;
}

void 
sd_init(void)
{
	static const char *disk_pdrv = "SD";
	uint64_t memory_size_mb;
	uint32_t block_count;
	uint32_t block_size;

	/* do { ... } while (0); required for multiline macros, i.e., LOG_* */
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
		printk("Sector size %u\n", block_size);

		memory_size_mb = (uint64_t)block_count * block_size;
		/* >> 10 is the same as divide by 1000 but faster
		 * >> 20 is the same as divide by 1000*1000 or 1000000
		 */
		printk("Memory Size is %u MB\n", (uint32_t)(memory_size_mb >> 20));

	} while (0);

	mp.mnt_point = disk_mnt;

	sd_status = fs_mount(&mp);

	if (sd_status == FR_OK) {
		printk("microSD disk mounted on %s.\n", disk_mnt);
		/* List contents of microSD for debug output */
		sd_ls(disk_mnt);
		sd_io_test("HELLO.TXT");
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

	printk("\nDirectory %s contents:\n", path);
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
