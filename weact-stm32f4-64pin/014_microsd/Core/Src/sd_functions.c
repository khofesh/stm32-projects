/******************************************************************************
 *  File        : sd_functions.c
 *  Author      : ControllersTech
 *  Website     : https://controllerstech.com
 *  Date        : June 26, 2025
 *
 *  Description :
 *    This file is part of a custom STM32/Embedded tutorial series.
 *    For documentation, updates, and more examples, visit the website above.
 *
 *  Note :
 *    This code is written and maintained by ControllersTech.
 *    You are free to use and modify it for learning and development.
 ******************************************************************************/


#include "fatfs.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "bsp_driver_sd.h"
#include "main.h"

#define MAX_FILES_TO_SHOW 10
#define MAX_DIRS_TO_SHOW 5
#define MAX_RECURSION_DEPTH 2

extern char SDPath[4];
FATFS fs;
BSP_SD_CardInfo myCardInfo;

void uart_delay(void) {
    HAL_Delay(1);
}

int sd_get_space_kb(void) {
	FATFS *pfs;
	DWORD fre_clust, tot_sect, fre_sect, total_kb, free_kb;
	FRESULT res = f_getfree(SDPath, &fre_clust, &pfs);
	if (res != FR_OK) return res;

	tot_sect = (pfs->n_fatent - 2) * pfs->csize;
	fre_sect = fre_clust * pfs->csize;
	total_kb = tot_sect / 2;
	free_kb = fre_sect / 2;
	printf("💾 Total: %lu KB, Free: %lu KB\r\n", total_kb, free_kb);
	uart_delay();
	return FR_OK;
}

int sd_mount(void) {
	FRESULT res;

	printf("=== FatFs Mount Debug ===\r\n");
	printf("SDPath: '%s'\r\n", SDPath);

	// First, let's check the disk status
	printf("Checking disk status...\r\n");
	DSTATUS diskStatus = disk_status(0);  // Drive 0
	printf("Disk status: 0x%02X ", diskStatus);

	if (diskStatus & STA_NOINIT) printf("(Not initialized) ");
	if (diskStatus & STA_NODISK) printf("(No disk) ");
	if (diskStatus & STA_PROTECT) printf("(Write protected) ");
	if (diskStatus == 0) printf("(Ready)");
	printf("\r\n");

	// Try to initialize the disk if needed
	if (diskStatus != 0) {
		printf("Initializing disk...\r\n");
		diskStatus = disk_initialize(0);
		printf("After init - Disk status: 0x%02X\r\n", diskStatus);
	}

	// Try to read the boot sector manually
	printf("Reading boot sector manually...\r\n");
	uint8_t bootSector[512];
	DRESULT diskResult = disk_read(0, bootSector, 0, 1);  // Read sector 0

	if (diskResult == RES_OK) {
		printf("✅ Boot sector read successfully\r\n");

		// Check for FAT signatures
		if (bootSector[510] == 0x55 && bootSector[511] == 0xAA) {
			printf("✅ Valid boot sector signature\r\n");

			// Check filesystem type
			char fsType[9] = {0};
			memcpy(fsType, &bootSector[54], 8);  // FAT12/FAT16 location
			printf("FS Type (54): '%s'\r\n", fsType);

			memcpy(fsType, &bootSector[82], 8);  // FAT32 location
			printf("FS Type (82): '%s'\r\n", fsType);

			// Print some boot sector info
			printf("Bytes per sector: %d\r\n", bootSector[11] | (bootSector[12] << 8));
			printf("Sectors per cluster: %d\r\n", bootSector[13]);
			printf("Reserved sectors: %d\r\n", bootSector[14] | (bootSector[15] << 8));
			printf("Number of FATs: %d\r\n", bootSector[16]);
		} else {
			printf("❌ Invalid boot sector signature\r\n");
		}
	} else {
		printf("❌ Failed to read boot sector: %d\r\n", diskResult);
	}

	printf("Attempting f_mount...\r\n");
	res = f_mount(&fs, SDPath, 1);  // 1 = mount immediately

	switch(res) {
		case FR_OK:
			printf("✅ SD card mounted successfully at %s\r\n", SDPath);
			sd_get_space_kb();
			return FR_OK;

		case FR_NO_FILESYSTEM:
			printf("❌ No filesystem found (Code: %d)\r\n", res);
			printf("The disk appears to have data but no recognizable filesystem\r\n");
			break;

		case FR_NOT_READY:
			printf("❌ SD card not ready (Code: %d)\r\n", res);
			printf("FatFs says disk is not ready despite hardware working\r\n");
			break;

		case FR_DISK_ERR:
			printf("❌ Disk error (Code: %d)\r\n", res);
			break;

		case FR_NOT_ENABLED:
			printf("❌ FatFs not enabled (Code: %d)\r\n", res);
			break;

		default:
			printf("❌ Mount failed with code: %d\r\n", res);
			break;
	}

	printf("Trying to mount with different paths...\r\n");

	// try mounting partition 0 explicitly
	res = f_mount(&fs, "0:", 1);
	if (res == FR_OK) {
		printf("✅ Mounted successfully with '0:'\r\n");
		return FR_OK;
	} else {
		printf("❌ Failed with '0:' - Code: %d\r\n", res);
	}

	printf("=== End FatFs Debug ===\r\n");
	return res;
}


int sd_unmount(void) {
	FRESULT res = f_mount(NULL, SDPath, 1);
	printf("SD card unmounted: %s\r\n\r\n\r\n", (res == FR_OK) ? "OK" : "Failed");
	return res;
}

int sd_write_file(const char *filename, const char *text) {
	FIL file;
	UINT bw;
	FRESULT res = f_open(&file, filename, FA_CREATE_ALWAYS | FA_WRITE);
	if (res != FR_OK) return res;

	res = f_write(&file, text, strlen(text), &bw);
	f_close(&file);
	printf("Write %u bytes to %s\r\n", bw, filename);
	return (res == FR_OK && bw == strlen(text)) ? FR_OK : FR_DISK_ERR;
}

int sd_append_file(const char *filename, const char *text) {
	FIL file;
	UINT bw;
	FRESULT res = f_open(&file, filename, FA_OPEN_ALWAYS | FA_WRITE);
	if (res != FR_OK) return res;

	res = f_lseek(&file, f_size(&file));
	if (res != FR_OK) {
		f_close(&file);
		return res;
	}

	res = f_write(&file, text, strlen(text), &bw);
	f_close(&file);
	printf("Appended %u bytes to %s\r\n", bw, filename);
	return (res == FR_OK && bw == strlen(text)) ? FR_OK : FR_DISK_ERR;
}

int sd_read_file(const char *filename, char *buffer, UINT bufsize, UINT *bytes_read) {
	FIL file;
	*bytes_read = 0;

	FRESULT res = f_open(&file, filename, FA_READ);
	if (res != FR_OK) {
		printf("f_open failed with code: %d\r\n", res);
		return res;
	}

	res = f_read(&file, buffer, bufsize - 1, bytes_read);
	if (res != FR_OK) {
		printf("f_read failed with code: %d\r\n", res);
		f_close(&file);
		return res;
	}

	buffer[*bytes_read] = '\0';

	res = f_close(&file);
	if (res != FR_OK) {
		printf("f_close failed with code: %d\r\n", res);
		return res;
	}

	printf("Read %u bytes from %s\r\n", *bytes_read, filename);
	return FR_OK;
}

typedef struct CsvRecord {
	char field1[32];
	char field2[32];
	int value;
} CsvRecord;

int sd_read_csv(const char *filename, CsvRecord *records, int max_records, int *record_count) {
    FIL file;
    char line[128];
    *record_count = 0;

    FRESULT res = f_open(&file, filename, FA_READ);
    if (res != FR_OK) {
        printf("Failed to open CSV: %s (%d)\r\n", filename, res);
        return res;
    }

    printf("📄 Reading CSV: %s\r\n", filename);
    while (f_gets(line, sizeof(line), &file) && *record_count < max_records) {
        // Strip newline characters
        char *newline = strchr(line, '\n');
        if (newline) *newline = '\0';
        newline = strchr(line, '\r');
        if (newline) *newline = '\0';

        // Skip empty lines
        if (strlen(line) == 0) continue;

        char *token = strtok(line, ",");
        if (!token) continue;

        // Copy field1 with proper null termination
        strncpy(records[*record_count].field1, token, sizeof(records[*record_count].field1) - 1);
        records[*record_count].field1[sizeof(records[*record_count].field1) - 1] = '\0';

        token = strtok(NULL, ",");
        if (!token) continue;

        // Copy field2 with proper null termination
        strncpy(records[*record_count].field2, token, sizeof(records[*record_count].field2) - 1);
        records[*record_count].field2[sizeof(records[*record_count].field2) - 1] = '\0';

        token = strtok(NULL, ",");
        if (token)
            records[*record_count].value = atoi(token);
        else
            records[*record_count].value = 0;

        (*record_count)++;
    }

    f_close(&file);

    for (int i = 0; i < *record_count; i++) {
        printf("[%d] %s | %s | %d\r\n", i,
                records[i].field1,
                records[i].field2,
                records[i].value);
    }

    return FR_OK;
}

int sd_delete_file(const char *filename) {
	FRESULT res = f_unlink(filename);
	printf("Delete %s: %s\r\n", filename, (res == FR_OK ? "OK" : "Failed"));
	return res;
}

int sd_rename_file(const char *oldname, const char *newname) {
	FRESULT res = f_rename(oldname, newname);
	printf("Rename %s to %s: %s\r\n", oldname, newname, (res == FR_OK ? "OK" : "Failed"));
	return res;
}

FRESULT sd_create_directory(const char *path) {
	FRESULT res = f_mkdir(path);
	printf("Create directory %s: %s\r\n", path, (res == FR_OK ? "OK" : "Failed"));
	return res;
}

void sd_list_directory_recursive(const char *path, int depth) {
	DIR dir;
	FILINFO fno;
//	char lfn[256];
//	fno.fname = lfn;
//	fno.fsize = sizeof(lfn);
	FRESULT res = f_opendir(&dir, path);
	if (res != FR_OK) {
		printf("%*s[ERR] Cannot open: %s\r\n", depth * 2, "", path);
		return;
	}

	while (1) {
		res = f_readdir(&dir, &fno);
		if (res != FR_OK || fno.fname[0] == 0) break;

		const char *name = (*fno.fname) ? fno.fname : fno.fname;

		if (fno.fattrib & AM_DIR) {
			if (strcmp(name, ".") && strcmp(name, "..")) {
				printf("%*s📁 %s\r\n", depth * 2, "", name);
				char newpath[512];
				snprintf(newpath, sizeof(newpath), "%s/%s", path, name);
				sd_list_directory_recursive(newpath, depth + 1);
			}
		} else {
			printf("%*s📄 %s (%lu bytes)\r\n", depth * 2, "", name, (unsigned long)fno.fsize);
		}
	}
	f_closedir(&dir);
}

void sd_list_files(void) {
	printf("📂 Files on SD Card:\r\n");
	sd_list_directory_recursive(SDPath, 0);
	printf("\r\n\r\n");
}

void sd_count_files(void) {
    DIR dir;
    FILINFO fno;
    FRESULT res = f_opendir(&dir, SDPath);

    if (res != FR_OK) {
        printf("[ERR] Cannot open root directory\r\n");
        return;
    }

    int file_count = 0;
    int dir_count = 0;
    unsigned long total_size = 0;

    while (1) {
        res = f_readdir(&dir, &fno);
        if (res != FR_OK || fno.fname[0] == 0) break;

        const char *name = (*fno.fname) ? fno.fname : fno.fname;

        if (fno.fattrib & AM_DIR) {
            if (strcmp(name, ".") && strcmp(name, "..")) {
                dir_count++;
            }
        } else {
            file_count++;
            total_size += fno.fsize;
        }
    }

    f_closedir(&dir);

    printf("📊 SD Card Summary:\r\n");
    printf("Files: %d\r\n", file_count);
    printf("Directories: %d\r\n", dir_count);
    printf("Total file size: %lu bytes (%.2f KB)\r\n",
           total_size, (float)total_size / 1024.0f);
    printf("\r\n");
}

void sd_list_files_pattern(const char *pattern) {
    DIR dir;
    FILINFO fno;
    FRESULT res = f_opendir(&dir, SDPath);

    if (res != FR_OK) {
        printf("[ERR] Cannot open root directory\r\n");
        return;
    }

    printf("📂 Files matching '*%s*':\r\n", pattern);

    int found = 0;

    while (1) {
        res = f_readdir(&dir, &fno);
        if (res != FR_OK || fno.fname[0] == 0) break;

        const char *name = (*fno.fname) ? fno.fname : fno.fname;

        // Simple substring search (case insensitive)
        if (!(fno.fattrib & AM_DIR) && strstr(name, pattern)) {
            found++;
            printf("📄 %s (%lu bytes)\r\n", name, (unsigned long)fno.fsize);

            if (found >= MAX_FILES_TO_SHOW) {
                printf("... (stopped at %d matches)\r\n", MAX_FILES_TO_SHOW);
                break;
            }
        }
    }

    f_closedir(&dir);

    if (found == 0) {
        printf("No files found matching '*%s*'\r\n", pattern);
    }
    printf("\r\n");
}

void sd_list_files_root_only(void) {
    DIR dir;
    FILINFO fno;
    FRESULT res = f_opendir(&dir, SDPath);

    if (res != FR_OK) {
        printf("[ERR] Cannot open root directory\r\n");
        return;
    }

    printf("📂 Files in Root Directory:\r\n");

    int file_count = 0;
    int dir_count = 0;

    while (1) {
        res = f_readdir(&dir, &fno);
        if (res != FR_OK || fno.fname[0] == 0) break;

        const char *name = (*fno.fname) ? fno.fname : fno.fname;

        if (fno.fattrib & AM_DIR) {
            if (strcmp(name, ".") && strcmp(name, "..")) {
                dir_count++;
                if (dir_count <= MAX_DIRS_TO_SHOW) {
                    printf("📁 %s/\r\n", name);
                } else if (dir_count == MAX_DIRS_TO_SHOW + 1) {
                    printf("📁 ... (%d more directories)\r\n", dir_count - MAX_DIRS_TO_SHOW);
                }
            }
        } else {
            file_count++;
            if (file_count <= MAX_FILES_TO_SHOW) {
                printf("📄 %s (%lu bytes)\r\n", name, (unsigned long)fno.fsize);
            } else if (file_count == MAX_FILES_TO_SHOW + 1) {
                printf("📄 ... (%d more files)\r\n", file_count - MAX_FILES_TO_SHOW);
            }
        }
    }

    f_closedir(&dir);
    printf("\r\nTotal in root: %d files, %d directories\r\n\r\n", file_count, dir_count);
}

void sd_list_directory_limited(const char *path, int depth, int *file_count, int *dir_count) {
    DIR dir;
    FILINFO fno;
    FRESULT res = f_opendir(&dir, path);

    if (res != FR_OK) {
        printf("%*s[ERR] Cannot open: %s\r\n", depth * 2, "", path);
        return;
    }

    // Stop if we've gone too deep
    if (depth > MAX_RECURSION_DEPTH) {
        printf("%*s[...] (max depth reached)\r\n", depth * 2, "");
        f_closedir(&dir);
        return;
    }

    int local_files = 0;
    int local_dirs = 0;

    while (1) {
        res = f_readdir(&dir, &fno);
        if (res != FR_OK || fno.fname[0] == 0) break;

        const char *name = (*fno.fname) ? fno.fname : fno.fname;

        if (fno.fattrib & AM_DIR) {
            if (strcmp(name, ".") && strcmp(name, "..")) {
                local_dirs++;
                (*dir_count)++;

                if (local_dirs <= MAX_DIRS_TO_SHOW) {
                    printf("%*s📁 %s\r\n", depth * 2, "", name);

                    // Only recurse if we haven't hit limits
                    if (*file_count < MAX_FILES_TO_SHOW && *dir_count < MAX_DIRS_TO_SHOW) {
                        char newpath[512];
                        snprintf(newpath, sizeof(newpath), "%s/%s", path, name);
                        sd_list_directory_limited(newpath, depth + 1, file_count, dir_count);
                    }
                } else if (local_dirs == MAX_DIRS_TO_SHOW + 1) {
                    printf("%*s📁 ... (%d more directories not shown)\r\n", depth * 2, "", local_dirs - MAX_DIRS_TO_SHOW);
                }
            }
        } else {
            local_files++;
            (*file_count)++;

            if (local_files <= MAX_FILES_TO_SHOW) {
                printf("%*s📄 %s (%lu bytes)\r\n", depth * 2, "", name, (unsigned long)fno.fsize);
            } else if (local_files == MAX_FILES_TO_SHOW + 1) {
                printf("%*s📄 ... (%d more files not shown)\r\n", depth * 2, "", local_files - MAX_FILES_TO_SHOW);
            }

            // Stop processing if we've shown enough files
            if (*file_count >= MAX_FILES_TO_SHOW) {
                break;
            }
        }
    }

    f_closedir(&dir);
}

void sd_list_files_limited(void) {
    printf("📂 Files on SD Card (showing first %d files):\r\n", MAX_FILES_TO_SHOW);

    int file_count = 0;
    int dir_count = 0;

    sd_list_directory_limited(SDPath, 0, &file_count, &dir_count);

    printf("\r\nSummary: Showed %d files, %d directories\r\n", file_count, dir_count);
}

