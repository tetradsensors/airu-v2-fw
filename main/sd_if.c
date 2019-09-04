/* SD card and FAT filesystem example.
   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include <stdio.h>
#include <string.h>
#include <time.h>
#include <sys/unistd.h>
#include <sys/stat.h>
#include "esp_err.h"
#include "esp_log.h"
#include "esp_vfs_fat.h"
#include "driver/sdmmc_host.h"
#include "driver/sdspi_host.h"
#include "sdmmc_cmd.h"
#include "sd_if.h"
#include "gps_if.h"

#define SD_LOG_FILE_NAME 				"/sdcard/LOGGING-0.log"
#define SD_LOG_FILE_MOST_RECENT_NAME 	"/sdcard/LOGGING-15.log"
#define SD_LOG_FILE_FORMAT 				"/sdcard/LOGGING-%02d.log"
#define MOUNT_CONFIG_MAXFILE 			20
#define MOUNT_CONFIG_MAXLOGFILE 		15
#define MAX_FILE_SIZE_MB 				1
#define MAX_LOG_PKG_LENGTH 				256
//#define SD_LOG 							0	/* moved to menuconfig */

// Maximum time to wait for the mutex in a logging statement.
#define MAX_MUTEX_WAIT_MS 30
#define MAX_MUTEX_WAIT_TICKS ((MAX_MUTEX_WAIT_MS + portTICK_PERIOD_MS - 1) / portTICK_PERIOD_MS)

static const char *TAG = "SD";
static sdmmc_card_t* card = NULL;
SemaphoreHandle_t s_log_mutex = NULL;
// Share object, need synchronization
static FILE *_semaphoreLogFileInstance;

static bool fs_mounted = false;

//int lineCount(char* filename);
//int deleteLineInFile(char* filename, int deleteLine);

// This example can use SDMMC and SPI peripherals to communicate with SD card.
// By default, SDMMC peripheral is used.
// To enable SPI mode, uncomment the following line:

// #define USE_SPI_MODE

// When testing SD and SPI modes, keep in mind that once the card has been
// initialized in SPI mode, it can not be reinitialized in SD mode without
// toggling power to the card.

#ifdef USE_SPI_MODE
// Pin mapping when using SPI mode.
// With this mapping, SD card can be used both in SPI and 1-line SD mode.
// Note that a pull-up on CS line is required in SD mode.
#define PIN_NUM_MISO 2
#define PIN_NUM_MOSI 15
#define PIN_NUM_CLK  14
#define PIN_NUM_CS   13
#endif //USE_SPI_MODE

esp_err_t SD_Initialize(void)
{
    ESP_LOGI(TAG, "Initializing SD card");

#ifndef USE_SPI_MODE
    ESP_LOGI(TAG, "Using SDMMC peripheral");
    sdmmc_host_t host = SDMMC_HOST_DEFAULT();

    // This initializes the slot without card detect (CD) and write protect (WP) signals.
    // Modify slot_config.gpio_cd and slot_config.gpio_wp if your board has these signals.
    sdmmc_slot_config_t slot_config = SDMMC_SLOT_CONFIG_DEFAULT();

    // To use 1-line SD mode, uncomment the following line:
    // slot_config.width = 1;

    // GPIOs 15, 2, 4, 12, 13 should have external 10k pull-ups.
    // Internal pull-ups are not sufficient. However, enabling internal pull-ups
    // does make a difference some boards, so we do that here.
    gpio_set_pull_mode(15, GPIO_PULLUP_ONLY);   // CMD, needed in 4- and 1- line modes
    gpio_set_pull_mode(2,  GPIO_PULLUP_ONLY);   // D0,  needed in 4- and 1-line modes
    gpio_set_pull_mode(4,  GPIO_PULLUP_ONLY);   // D1,  needed in 4-line mode only
    gpio_set_pull_mode(12, GPIO_PULLUP_ONLY);   // D2,  needed in 4-line mode only
    gpio_set_pull_mode(13, GPIO_PULLUP_ONLY);   // D3,  needed in 4- and 1-line modes

#else
    ESP_LOGI(TAG, "Using SPI peripheral");

    sdmmc_host_t host = SDSPI_HOST_DEFAULT();
    sdspi_slot_config_t slot_config = SDSPI_SLOT_CONFIG_DEFAULT();
    slot_config.gpio_miso = PIN_NUM_MISO;
    slot_config.gpio_mosi = PIN_NUM_MOSI;
    slot_config.gpio_sck  = PIN_NUM_CLK;
    slot_config.gpio_cs   = PIN_NUM_CS;
    // This initializes the slot without card detect (CD) and write protect (WP) signals.
    // Modify slot_config.gpio_cd and slot_config.gpio_wp if your board has these signals.
#endif //USE_SPI_MODE

    // Options for mounting the filesystem.
    // If format_if_mount_failed is set to true, SD card will be partitioned and
    // formatted in case when mounting fails.
    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
        .format_if_mount_failed = false,
        .max_files = MOUNT_CONFIG_MAXFILE,
        .allocation_unit_size = MAX_FILE_SIZE_MB /* * MOUNT_CONFIG_MAXFILE */ * 1024 * 1024
    };

    // Use settings defined above to initialize SD card and mount FAT filesystem.
    // Note: esp_vfs_fat_sdmmc_mount is an all-in-one convenience function.
    // Please check its source code and implement error recovery when developing
    // production applications.

    esp_err_t ret = esp_vfs_fat_sdmmc_mount("/sdcard", &host, &slot_config, &mount_config, &card);

    if (ret != ESP_OK) {
        if (ret == ESP_FAIL) {
            ESP_LOGE(TAG, "Failed to mount filesystem. "
                "If you want the card to be formatted, set format_if_mount_failed = true.");
        } else {
            ESP_LOGE(TAG, "Failed to initialize the card (%s). "
                "Make sure SD card lines have pull-up resistors in place.", esp_err_to_name(ret));
        }
        return ret;
    }

    // Card has been initialized, print its properties
    sdmmc_card_print_info(stdout, card);

#ifdef CONFIG_SD_CARD_DEBUG
	printf("Setting sd card as logger...\n\r");
	esp_log_set_vprintf(esp_sd_log_write);
	periodic_timer_callback(NULL);
#endif
    fs_mounted = true;
    return ret;
}


esp_err_t sd_deinit(void)
{
    return esp_vfs_fat_sdmmc_unmount();
}

/*
 * Currently the date comes exclusively from the GPS module. I'm doing this
 * because I don't want the date and time to jump between NTP and GPS, and I
 * believe GPS is a local time. GPS should always be correct as long as there's
 * a battery, so I don't think it will be a big deal.
 */
esp_err_t sd_write_data(char* pkt, uint8_t year, uint8_t month, uint8_t day)
{
//	esp_err_t err = ESP_FAIL;
//	time_t now; /* time_t == long */
//	struct tm timeinfo;
    struct stat st;
    char filename[64];

    ESP_LOGI(TAG, "SD Packet:\n%s", pkt);

	// Files are created daily. Filename is YYYY-MM-DD.csv
//    time(&now);
//    localtime_r(&now, &timeinfo);
//    strftime(filename, sizeof(filename), "/sdcard/%y-%m-%d.csv", &timeinfo);

	sprintf(filename, "/sdcard/%02d-%02d-%02d.csv", year, month, day);

    ESP_LOGI(TAG, "Filename: %s", filename);

    // If file doesn't exist, need to add header
    bool exists = stat(filename, &st) == 0;
    ESP_LOGI(TAG, "File %s", exists ? "exists" : "does not exist.");

    FILE* f = fopen(filename, "a");
    if (f == NULL) {
    	ESP_LOGE(TAG, "Failed to open %s...", filename);
    	return ESP_FAIL;
    }

    // Write the header if it's a new file
    if (!exists) {
    	fprintf(f, "%s", SD_HDR);
    }

    // Write the data
    fprintf(f, "%s", pkt);
    fclose(f);
    return ESP_OK;
}

vprintf_like_t esp_sd_log_write(const char* format, va_list ap)
{
	esp_err_t err = ESP_FAIL;
	FILE *loggingInstance;

	loggingInstance = getLogFileInstance();
    if (loggingInstance == NULL) {
    	printf("esp_sd_log_write Failed to retrieve %s...\n", SD_LOG_FILE_NAME);
        return err;
    } else {

		vfprintf(_semaphoreLogFileInstance, format, ap);
		fflush(_semaphoreLogFileInstance);
		releaseLogFileInstance();
		err = ESP_OK;
    }
	return err;
}

// Call back for updating and checking log file name
void periodic_timer_callback(void* arg)
{
#ifdef CONFIG_SD_CARD_DEBUG
    bool exists;
    char SDLogfileName[SD_FILENAME_LENGTH];
    FILE* logFileInstance = NULL;
    static bool firstTime = true;
    static uint8_t logFileCounting = 1;
    struct stat st;
    int ret =0 ;

    printf("periodic_timer_callback ENTERRED\n");
	/*
		Check for current file size and existence
		Create new file if size > 2Mb or not exist
		New file name will be stored in loggingList.txt
	*/
    printf("First time: %d\n\r", firstTime);
	if(!firstTime) {
		logFileInstance = getLogFileInstance();
	}

    exists = stat(SD_LOG_FILE_NAME, &st) == 0;
	if (st.st_size > MAX_FILE_SIZE_MB * 1024 * 1024) {	// 2 Mb max or Not Exist
		if (_semaphoreLogFileInstance) {
			if(!firstTime) {
				fclose(_semaphoreLogFileInstance);
				_semaphoreLogFileInstance = NULL;
				sprintf(SDLogfileName, SD_LOG_FILE_FORMAT, logFileCounting);
				logFileCounting++;
				if ((ret = remove(SDLogfileName)) != 0) {
					printf("%s: ERR cannot REMOVE file [%s]\n", __func__, SDLogfileName);
				}
				if ((ret = rename(SD_LOG_FILE_NAME, SDLogfileName)) != 0) {
					printf("%s: ERR cannot RENAME file [%s]\n", __func__, SDLogfileName);
				}
			}
		}
	}

	if (logFileCounting > MOUNT_CONFIG_MAXLOGFILE) {
		char oldLogFileName[SD_FILENAME_LENGTH];
		for (uint8_t i = 1; i > MOUNT_CONFIG_MAXLOGFILE-1; i++) {
	    	sprintf(SDLogfileName, SD_LOG_FILE_FORMAT, i);
	    	// Make sure the furthest file get removed
			if ((ret = remove(SDLogfileName)) != 0) {
				printf("%s: ERR cannot REMOVE file [%s]\n", __func__, SDLogfileName);
			}
	    	sprintf(oldLogFileName, SD_LOG_FILE_FORMAT, i+1);
	    	// Rename file_i+1 into file_i
			rename(oldLogFileName, SDLogfileName);
			if ((ret = rename(SD_LOG_FILE_NAME, SDLogfileName)) != 0) {
				printf("%s: ERR cannot RENAME file [%s]\n", __func__, SDLogfileName);
			}
		}
		// Remove the LOGGING-15.log since its content is copied into LOGGING-14.log previously
		remove(SD_LOG_FILE_MOST_RECENT_NAME);
		logFileCounting = 15;
	}

    if(firstTime) {
    	firstTime = false;
    }
    else if (logFileInstance != NULL) {
    	releaseLogFileInstance();
    }
    _semaphoreLogFileInstance = logFileInstance;
#endif
}

FILE *getLogFileInstance() {
    if (!s_log_mutex) {
        s_log_mutex = xSemaphoreCreateMutex();
    }
    if (xSemaphoreTake(s_log_mutex, MAX_MUTEX_WAIT_TICKS) == pdFALSE) {
        return NULL;
    }

    if(_semaphoreLogFileInstance != NULL) {
    	return _semaphoreLogFileInstance;
    } else {
		_semaphoreLogFileInstance = fopen(SD_LOG_FILE_NAME, "a");
    	if (_semaphoreLogFileInstance == NULL) {
    		printf("ERROR opening Log file %s\n", SD_LOG_FILE_NAME);
    		releaseLogFileInstance();
    		return NULL;
    	} else
    		return _semaphoreLogFileInstance;
    }
}

void releaseLogFileInstance() {
    if( s_log_mutex != NULL ) {
    	if (_semaphoreLogFileInstance) {
			fclose(_semaphoreLogFileInstance);
			_semaphoreLogFileInstance = NULL;
    	}
		if(xSemaphoreGive(s_log_mutex) != pdTRUE) {
			return;
		}
    }
}


FILE* sd_fopen(const char* filename)
{
	esp_err_t err;
	FILE* fp;
	char fn_full[SD_FILENAME_LENGTH];

	// Mount if needed
	if(!fs_mounted){
		if((err = SD_Initialize()) != ESP_OK){
			return NULL;
		}
	}

	if(snprintf(fn_full, SD_FILENAME_LENGTH, "/sdcard/%s", filename) > SD_FILENAME_LENGTH){
		ESP_LOGE(TAG, "Filename too long: %s", fn_full);
		return NULL;
	}

	ESP_LOGI(TAG, "Opening %s", fn_full);
	if((fp = fopen(fn_full, "r")) == NULL){
		ESP_LOGE(TAG, "Could not open %s", fn_full);
	}

	return fp;
}
