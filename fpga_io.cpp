#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>
#include <signal.h>
#include <ctype.h>
#include <termios.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <linux/spi/spidev.h>

#include "fpga_io.h"
#include "file_io.h"
#include "input.h"
#include "osd.h"
#include "menu.h"
#include "shmem.h"
#include "offload.h"
#include "gpiod.h"

#include "fpga_nic301.h"

#define fatal(x) /*munmap((void*)map_base, FPGA_REG_SIZE);*/ close(fd); exit(x)

static const char *gpio_chip_name = "gpiochip0";
#define GPIIO_PIN_FPGA_RESET 102
#define GPIIO_PIN_FPGA_EN    103
#define GPIIO_PIN_OSD_EN     104
#define GPIIO_PIN_IO_EN      105
static struct gpiod_chip *gpio_chip;
static struct gpiod_line *gpio_line_fpga_reset;
static struct gpiod_line *gpio_line_fpga_en;
static struct gpiod_line *gpio_line_osd_en;
static struct gpiod_line *gpio_line_io_en;

static const char *spi_device = "/dev/spidev1.0";
#define SPI_SPEED 8000000
#define SPI_TRACE 1
static uint8_t spi_mode = SPI_MODE_3;

uint8_t tx_buf[2];    	// TX buffer (16 bit unsigned integer)
uint8_t rx_buf[2];    	// RX buffer (16 bit unsigned integer)

struct spi_ioc_transfer spi_transfer =
{	.tx_buf = (unsigned long)tx_buf,
	.rx_buf = (unsigned long)rx_buf,
	.len = 2,
	.speed_hz = SPI_SPEED,
	.delay_usecs = 0,
	.bits_per_word = 8,
	// if this is zero, then CS is enabled
	.cs_change = 0,
};

int spi_fd;

static uint32_t *map_base;

#define MAP_ADDR 0x0
#define writel(val, reg) *MAP_ADDR(reg) = val
#define readl(reg) *MAP_ADDR(reg)

#define clrsetbits_le32(addr, clear, set) writel((readl(addr) & ~(clear)) | (set), addr)
#define setbits_le32(addr, set)           writel( readl(addr) | (set), addr)
#define clrbits_le32(addr, clear)         writel( readl(addr) & ~(clear), addr)

/* Timeout count */
#define FPGA_TIMEOUT_CNT		0x1000000

/* Check whether FPGA Init_Done signal is high */
static int is_fpgamgr_initdone_high(void)
{
	return 1;
}

/* Check whether FPGA is ready to be accessed */
static int fpgamgr_test_fpga_ready(void)
{
	/* Check for init done signal */
	if (!is_fpgamgr_initdone_high())
		return 0;

	/* Check again to avoid false glitches */
	if (!is_fpgamgr_initdone_high())
		return 0;

	return 1;
}

#define DIV_ROUND_UP(n,d) (((n) + (d) - 1) / (d))

static int make_env(const char *name, const char *cfg)
{
	printf("make_env\n");
	void* buf = 0; //shmem_map(0x1FFFF000, 0x1000);
	if (!buf) return -1;

	volatile char* str = (volatile char*)buf;
	memset((void*)str, 0, 0xF00);

	*str++ = 0x21;
	*str++ = 0x43;
	*str++ = 0x65;
	*str++ = 0x87;
	*str++ = 'c';
	*str++ = 'o';
	*str++ = 'r';
	*str++ = 'e';
	*str++ = '=';
	*str++ = '"';

	for (uint32_t i = 0; i < strlen(name); i++)
	{
		*str++ = name[i];
	}

	*str++ = '"';
	*str++ = '\n';
	FileLoad(cfg, (void*)str, 0);
	// shmem_unmap(buf, 0x1000);
	return 0;
}

int fpga_load_rbf(const char *name, const char *cfg, const char *xml)
{
	OsdDisable();
	static char path[1024];
	int ret = 0;

	if(cfg)
	{
		fpga_core_reset(1);
		make_env(name, cfg);
		//do_bridge(0);
		reboot(0);
	}

	printf("Loading RBF: %s\n", name);

	if(name[0] == '/') strcpy(path, name);
	else sprintf(path, "%s/%s", !strcasecmp(name, "menu.rbf") ? getStorageDir(0) : getRootDir(), name);

	// TODO
	int rbf = -1; //open(path, O_RDONLY);
	if (rbf < 0)
	{
		char error[4096];
		snprintf(error,4096,"%s\nNot Found", name);
		printf("Couldn't open file %s\n", path);
		Info(error,5000);
		return -1;
	}

	app_restart(!strcasecmp(name, "menu.rbf") ? "menu.rbf" : path, xml);
	return ret;
}

int fpga_io_init()
{
	gpio_chip = gpiod_chip_open_by_name(gpio_chip_name);
	if (!gpio_chip) goto err;

	gpio_line_fpga_reset = gpiod_chip_get_line(gpio_chip, GPIIO_PIN_FPGA_RESET);
	gpio_line_fpga_en    = gpiod_chip_get_line(gpio_chip, GPIIO_PIN_FPGA_EN);
	gpio_line_osd_en     = gpiod_chip_get_line(gpio_chip, GPIIO_PIN_OSD_EN);
	gpio_line_io_en      = gpiod_chip_get_line(gpio_chip, GPIIO_PIN_IO_EN);
	if (!gpio_line_fpga_en     ||
	    !gpio_line_osd_en      ||
	    !gpio_line_io_en       ||
	    !gpio_line_fpga_reset) goto err;

	gpiod_line_request_output(gpio_line_fpga_reset, "FPGA_RESET", 0);
	gpiod_line_request_output(gpio_line_fpga_en, "FPGA_EN", 0);
	gpiod_line_request_output(gpio_line_osd_en, "OSD_EN", 0);
	gpiod_line_request_output(gpio_line_io_en, "IO_EN", 0);

	return 0;

	err:
		printf("Error: fpga_io_init() failed!\n");
		return -1;
}

int fpga_core_id()
{
	return 0x5CA623A4;
}

int fpga_get_fio_size()
{
	// 0: normal (8-bit) or 1: wide (16-bit) IO
	return 0;
}

int fpga_get_io_version()
{
	// 0: obsolete, 1: current
	return 1;
}

void fpga_set_led(uint32_t on)
{
}

int fpga_get_buttons()
{
	// OSD/User Buttons
	return 0;
}

int fpga_get_io_type()
{
	// the FPGA board switches
	return 0;
}

void reboot(int cold)
{
	sync();
	fpga_core_reset(1);

	usleep(500000);

	system("reboot");
}

char *getappname()
{
	static char dest[PATH_MAX];
	memset(dest, 0, sizeof(dest));

	char path[64];
	sprintf(path, "/proc/%d/exe", getpid());
	readlink(path, dest, PATH_MAX);

	return dest;
}

void app_restart(const char *path, const char *xml)
{
	sync();
	fpga_core_reset(1);

	input_switch(0);
	input_uinp_destroy();

	offload_stop();

	char *appname = getappname();
	printf("restarting the %s\n", appname);
	execl(appname, appname, path, xml, NULL);

	printf("Something went wrong. Rebooting...\n");
	reboot(1);
}

void fpga_core_reset(int reset)
{
	printf("fpga_core_reset(%d)\n", reset);
	gpiod_line_set_value(gpio_line_fpga_reset, reset);
}

int is_fpga_ready(int quick)
{
	return fpgamgr_test_fpga_ready();
}

#define SSPI_STROBE  (1<<17)
#define SSPI_ACK     SSPI_STROBE

void fpga_spi_en(uint32_t mask, uint32_t en)
{
	if (SPI_TRACE) printf("fpga_spi_en(%8x, %x)\n", mask, en);
	if (mask & SSPI_FPGA_EN) gpiod_line_set_value(gpio_line_fpga_en, en);
	if (mask & SSPI_OSD_EN)  gpiod_line_set_value(gpio_line_osd_en,  en);
	if (mask & SSPI_IO_EN)   gpiod_line_set_value(gpio_line_io_en,   en);

	if (en) {
		spi_fd = open(spi_device, O_RDWR);
		if (spi_fd == -1) {
			printf("ERROR: cannot open SPI device %s\n", spi_device);
			return;
		}

		if (ioctl(spi_fd, SPI_IOC_WR_MODE, &spi_mode) == -1) {
			printf("ERROR: cannot set SPI mode\n");
			return;
		}
	} else {
		if (spi_fd >= 0) {
			close(spi_fd);
		}
	}
}

void fpga_wait_to_reset()
{
	printf("FPGA is not ready. JTAG uploading?\n");
	printf("Waiting for FPGA to be ready...\n");

	fpga_core_reset(1);

	while (!is_fpga_ready(0))
	{
		sleep(1);
	}
	reboot(0);
}

uint16_t fpga_spi(uint16_t word)
{
	if (SPI_TRACE) printf("fpga_spi(%04x)", word);
	tx_buf[0] = word >> 8;
	tx_buf[1] = word & 0xff;
	if (ioctl(spi_fd, SPI_IOC_MESSAGE(1), &spi_transfer) < 1)
	{
		printf("Can't send SPI message");
		return -1;
	}
	uint16_t result = (rx_buf[0] << 8) | rx_buf[1];
	if (SPI_TRACE) printf(" => %04x\n", result);
	return result;
}

uint16_t fpga_spi_fast(uint16_t word)
{
	if (SPI_TRACE) printf("fpga_spi_fast(%04x)\n", word);
	fpga_spi(word);
	return 0;
}

void fpga_spi_fast_block_write(const uint16_t *buf, uint32_t length)
{
	printf("fpga_spi_fast_block_write(%d)\n", length);
}

void fpga_spi_fast_block_read(uint16_t *buf, uint32_t length)
{
	printf("fpga_spi_fast_block_read(%d)\n", length);
}

void fpga_spi_fast_block_write_8(const uint8_t *buf, uint32_t length)
{
	printf("fpga_spi_fast_block_write_8(%d)\n", length);
}

void fpga_spi_fast_block_read_8(uint8_t *buf, uint32_t length)
{
	printf("fpga_spi_fast_block_read_8(%d)\n", length);
}

void fpga_spi_fast_block_write_be(const uint16_t *buf, uint32_t length)
{
	printf("fpga_spi_fast_block_write_be(%d)\n", length);
}

void fpga_spi_fast_block_read_be(uint16_t *buf, uint32_t length)
{
	printf("fpga_spi_fast_block_read_be(%d)\n", length);
}
