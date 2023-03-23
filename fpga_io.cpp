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

#include "cfg.h"
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
const static bool spi_trace = 0;
#define SPI_EN_TRACE 0

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

#define DIV_ROUND_UP(n,d) (((n) + (d) - 1) / (d))

static int make_env(const char *name, const char *cfg)
{
	printf("make_env\n");
	// TODO
	return 0;
}

int fpga_load_rbf(const char *name, const char *config, const char *xml) {
  OsdDisable();
  static char path[1024];
  static char command[2048];
  int ret = 0;

  if (config) {
    fpga_core_reset();
    make_env(name, config);
    // do_bridge(0);
    reboot(0);
  }

  if (name[0] == '/')
    strcpy(path, name);
  else
    sprintf(path, "%s/%s",
            !strcasecmp(name, cfg.menu_core_filename) ? getStorageDir(0) : getRootDir(),
            name);

  printf("Loading bitstream: %s\n", path);
  sprintf(command, "openFPGALoader -c dirtyJtag %s", path);

  int rbf = system(command);

  if (rbf != 0) {
    char error[4096];
    snprintf(error, 4096, "error uploading %s\n", path);
    printf("Command failed: %s with code %d\n", command, rbf);
    Info(error, 5000);
    return -1;
  }

  printf("Success!\n");
  app_restart(!strcasecmp(name, cfg.menu_core_filename) ? cfg.menu_core_filename : path, xml);
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
	fpga_core_reset();

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
	fpga_core_reset();

	input_switch(0);
	input_uinp_destroy();

	offload_stop();

	char *appname = getappname();
	printf("restarting the %s\n", appname);
	execl(appname, appname, path, xml, NULL);

	printf("Something went wrong. Rebooting...\n");
	reboot(1);
}

void fpga_core_reset()
{
	printf("fpga_core_reset()\n");
	gpiod_line_set_value(gpio_line_fpga_reset, 1);
	usleep(100000);
	gpiod_line_set_value(gpio_line_fpga_reset, 0);
}

int is_fpga_ready(int quick)
{
	// This used to check via HPS if the FPGA has been configured
	// We don't have that in themMoment.
	return true;
}

#define SSPI_STROBE  (1<<17)
#define SSPI_ACK     SSPI_STROBE

void fpga_spi_en(uint32_t mask, uint32_t en)
{
	static uint8_t spi_mode = SPI_MODE_0;

	if (SPI_EN_TRACE) printf("fpga_spi_en(%8x, %x)\n", mask, en);
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
			printf("ERROR: cannot set SPI write mode\n");
			return;
		}
		if (ioctl(spi_fd, SPI_IOC_RD_MODE, &spi_mode) == -1) {
			printf("ERROR: cannot set SPI read mode\n");
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

	fpga_core_reset();

	while (!is_fpga_ready(0))
	{
		sleep(1);
	}
	reboot(0);
}

uint16_t fpga_spi(uint16_t word)
{
	if (spi_trace) printf("fpga_spi(%04x)", word);
	tx_buf[0] = word >> 8;
	tx_buf[1] = word & 0xff;
	if (ioctl(spi_fd, SPI_IOC_MESSAGE(1), &spi_transfer) < 1)
	{
		printf("Can't send SPI message");
		return -1;
	}
	uint16_t result = (rx_buf[0] << 8) | rx_buf[1];
	if (spi_trace) printf(" => %04x\n", result);
	return result;
}

uint16_t fpga_spi_fast(uint16_t word)
{
	if (spi_trace) printf("fpga_spi_fast(%04x)\n", word);
	fpga_spi(word);
	return 0;
}

void fpga_spi_fast_block_write(const uint16_t *buffer, uint32_t length)
{
	printf("fpga_spi_fast_block_write(%d)\n", length);
	uint32_t rem = length % 16;
	length /= 16;
	tx_buf[0] = 0; tx_buf[1] = 0;

	ioctl(spi_fd, SPI_IOC_MESSAGE(1), &spi_transfer);

	uint8_t *buf = (uint8_t *)buffer;
	while (length--) {
		tx_buf[1] = *buf++; tx_buf[0] = *buf++;
		ioctl(spi_fd, SPI_IOC_MESSAGE(1), &spi_transfer);
		tx_buf[1] = *buf++; tx_buf[0] = *buf++;
		ioctl(spi_fd, SPI_IOC_MESSAGE(1), &spi_transfer);
		tx_buf[1] = *buf++; tx_buf[0] = *buf++;
		ioctl(spi_fd, SPI_IOC_MESSAGE(1), &spi_transfer);
		tx_buf[1] = *buf++; tx_buf[0] = *buf++;
		ioctl(spi_fd, SPI_IOC_MESSAGE(1), &spi_transfer);
		tx_buf[1] = *buf++; tx_buf[0] = *buf++;
		ioctl(spi_fd, SPI_IOC_MESSAGE(1), &spi_transfer);
		tx_buf[1] = *buf++; tx_buf[0] = *buf++;
		ioctl(spi_fd, SPI_IOC_MESSAGE(1), &spi_transfer);
		tx_buf[1] = *buf++; tx_buf[0] = *buf++;
		ioctl(spi_fd, SPI_IOC_MESSAGE(1), &spi_transfer);
		tx_buf[1] = *buf++; tx_buf[0] = *buf++;
		ioctl(spi_fd, SPI_IOC_MESSAGE(1), &spi_transfer);
		tx_buf[1] = *buf++; tx_buf[0] = *buf++;
		ioctl(spi_fd, SPI_IOC_MESSAGE(1), &spi_transfer);
		tx_buf[1] = *buf++; tx_buf[0] = *buf++;
		ioctl(spi_fd, SPI_IOC_MESSAGE(1), &spi_transfer);
		tx_buf[1] = *buf++; tx_buf[0] = *buf++;
		ioctl(spi_fd, SPI_IOC_MESSAGE(1), &spi_transfer);
		tx_buf[1] = *buf++; tx_buf[0] = *buf++;
		ioctl(spi_fd, SPI_IOC_MESSAGE(1), &spi_transfer);
		tx_buf[1] = *buf++; tx_buf[0] = *buf++;
		ioctl(spi_fd, SPI_IOC_MESSAGE(1), &spi_transfer);
		tx_buf[1] = *buf++; tx_buf[0] = *buf++;
		ioctl(spi_fd, SPI_IOC_MESSAGE(1), &spi_transfer);
		tx_buf[1] = *buf++; tx_buf[0] = *buf++;
		ioctl(spi_fd, SPI_IOC_MESSAGE(1), &spi_transfer);
		tx_buf[1] = *buf++; tx_buf[0] = *buf++;
		ioctl(spi_fd, SPI_IOC_MESSAGE(1), &spi_transfer);
	}

	while (rem--) {
		tx_buf[1] = *buf++; tx_buf[0] = *buf++;
		ioctl(spi_fd, SPI_IOC_MESSAGE(1), &spi_transfer);
	}
}

void fpga_spi_fast_block_read(uint16_t *buf, uint32_t length)
{
	uint32_t rem = length % 16;
	length /= 16;
	tx_buf[0] = 0; tx_buf[1] = 0;

	ioctl(spi_fd, SPI_IOC_MESSAGE(1), &spi_transfer);

	while (length--) {
		ioctl(spi_fd, SPI_IOC_MESSAGE(1), &spi_transfer);
		*buf++ = (rx_buf[0] << 8) | rx_buf[1];
		ioctl(spi_fd, SPI_IOC_MESSAGE(1), &spi_transfer);
		*buf++ = (rx_buf[0] << 8) | rx_buf[1];
		ioctl(spi_fd, SPI_IOC_MESSAGE(1), &spi_transfer);
		*buf++ = (rx_buf[0] << 8) | rx_buf[1];
		ioctl(spi_fd, SPI_IOC_MESSAGE(1), &spi_transfer);
		*buf++ = (rx_buf[0] << 8) | rx_buf[1];
		ioctl(spi_fd, SPI_IOC_MESSAGE(1), &spi_transfer);
		*buf++ = (rx_buf[0] << 8) | rx_buf[1];
		ioctl(spi_fd, SPI_IOC_MESSAGE(1), &spi_transfer);
		*buf++ = (rx_buf[0] << 8) | rx_buf[1];
		ioctl(spi_fd, SPI_IOC_MESSAGE(1), &spi_transfer);
		*buf++ = (rx_buf[0] << 8) | rx_buf[1];
		ioctl(spi_fd, SPI_IOC_MESSAGE(1), &spi_transfer);
		*buf++ = (rx_buf[0] << 8) | rx_buf[1];
		ioctl(spi_fd, SPI_IOC_MESSAGE(1), &spi_transfer);
		*buf++ = (rx_buf[0] << 8) | rx_buf[1];
		ioctl(spi_fd, SPI_IOC_MESSAGE(1), &spi_transfer);
		*buf++ = (rx_buf[0] << 8) | rx_buf[1];
		ioctl(spi_fd, SPI_IOC_MESSAGE(1), &spi_transfer);
		*buf++ = (rx_buf[0] << 8) | rx_buf[1];
		ioctl(spi_fd, SPI_IOC_MESSAGE(1), &spi_transfer);
		*buf++ = (rx_buf[0] << 8) | rx_buf[1];
		ioctl(spi_fd, SPI_IOC_MESSAGE(1), &spi_transfer);
		*buf++ = (rx_buf[0] << 8) | rx_buf[1];
		ioctl(spi_fd, SPI_IOC_MESSAGE(1), &spi_transfer);
		*buf++ = (rx_buf[0] << 8) | rx_buf[1];
		ioctl(spi_fd, SPI_IOC_MESSAGE(1), &spi_transfer);
		*buf++ = (rx_buf[0] << 8) | rx_buf[1];
		ioctl(spi_fd, SPI_IOC_MESSAGE(1), &spi_transfer);
		*buf++ = (rx_buf[0] << 8) | rx_buf[1];
	}

	while (rem--) {
		ioctl(spi_fd, SPI_IOC_MESSAGE(1), &spi_transfer);
		*buf++ = (rx_buf[0] << 8) | rx_buf[1];
	}
}

void fpga_spi_fast_block_write_8(const uint8_t *buf, uint32_t length)
{
	printf("fpga_spi_fast_block_write_8(%d)\n", length);
	uint32_t rem = length % 16;
	length /= 16;
	tx_buf[0] = 0; tx_buf[1] = 0;

	ioctl(spi_fd, SPI_IOC_MESSAGE(1), &spi_transfer);

	while (length--) {
		tx_buf[1] = *buf++;
		ioctl(spi_fd, SPI_IOC_MESSAGE(1), &spi_transfer);
		tx_buf[1] = *buf++;
		ioctl(spi_fd, SPI_IOC_MESSAGE(1), &spi_transfer);
		tx_buf[1] = *buf++;
		ioctl(spi_fd, SPI_IOC_MESSAGE(1), &spi_transfer);
		tx_buf[1] = *buf++;
		ioctl(spi_fd, SPI_IOC_MESSAGE(1), &spi_transfer);
		tx_buf[1] = *buf++;
		ioctl(spi_fd, SPI_IOC_MESSAGE(1), &spi_transfer);
		tx_buf[1] = *buf++;
		ioctl(spi_fd, SPI_IOC_MESSAGE(1), &spi_transfer);
		tx_buf[1] = *buf++;
		ioctl(spi_fd, SPI_IOC_MESSAGE(1), &spi_transfer);
		tx_buf[1] = *buf++;
		ioctl(spi_fd, SPI_IOC_MESSAGE(1), &spi_transfer);
		tx_buf[1] = *buf++;
		ioctl(spi_fd, SPI_IOC_MESSAGE(1), &spi_transfer);
		tx_buf[1] = *buf++;
		ioctl(spi_fd, SPI_IOC_MESSAGE(1), &spi_transfer);
		tx_buf[1] = *buf++;
		ioctl(spi_fd, SPI_IOC_MESSAGE(1), &spi_transfer);
		tx_buf[1] = *buf++;
		ioctl(spi_fd, SPI_IOC_MESSAGE(1), &spi_transfer);
		tx_buf[1] = *buf++;
		ioctl(spi_fd, SPI_IOC_MESSAGE(1), &spi_transfer);
		tx_buf[1] = *buf++;
		ioctl(spi_fd, SPI_IOC_MESSAGE(1), &spi_transfer);
		tx_buf[1] = *buf++;
		ioctl(spi_fd, SPI_IOC_MESSAGE(1), &spi_transfer);
		tx_buf[1] = *buf++;
		ioctl(spi_fd, SPI_IOC_MESSAGE(1), &spi_transfer);
	}

	while (rem--) {
		tx_buf[1] = *buf++;
		ioctl(spi_fd, SPI_IOC_MESSAGE(1), &spi_transfer);
	}
}

void fpga_spi_fast_block_read_8(uint8_t *buf, uint32_t length)
{
	printf("fpga_spi_fast_block_read(%d)\n", length);
	uint32_t rem = length % 16;
	length /= 16;
	tx_buf[0] = 0; tx_buf[1] = 0;

	ioctl(spi_fd, SPI_IOC_MESSAGE(1), &spi_transfer);

	while (length--) {
		ioctl(spi_fd, SPI_IOC_MESSAGE(1), &spi_transfer);
		*buf++ = rx_buf[1];
		ioctl(spi_fd, SPI_IOC_MESSAGE(1), &spi_transfer);
		*buf++ = rx_buf[1];
		ioctl(spi_fd, SPI_IOC_MESSAGE(1), &spi_transfer);
		*buf++ = rx_buf[1];
		ioctl(spi_fd, SPI_IOC_MESSAGE(1), &spi_transfer);
		*buf++ = rx_buf[1];
		ioctl(spi_fd, SPI_IOC_MESSAGE(1), &spi_transfer);
		*buf++ = rx_buf[1];
		ioctl(spi_fd, SPI_IOC_MESSAGE(1), &spi_transfer);
		*buf++ = rx_buf[1];
		ioctl(spi_fd, SPI_IOC_MESSAGE(1), &spi_transfer);
		*buf++ = rx_buf[1];
		ioctl(spi_fd, SPI_IOC_MESSAGE(1), &spi_transfer);
		*buf++ = rx_buf[1];
		ioctl(spi_fd, SPI_IOC_MESSAGE(1), &spi_transfer);
		*buf++ = rx_buf[1];
		ioctl(spi_fd, SPI_IOC_MESSAGE(1), &spi_transfer);
		*buf++ = rx_buf[1];
		ioctl(spi_fd, SPI_IOC_MESSAGE(1), &spi_transfer);
		*buf++ = rx_buf[1];
		ioctl(spi_fd, SPI_IOC_MESSAGE(1), &spi_transfer);
		*buf++ = rx_buf[1];
		ioctl(spi_fd, SPI_IOC_MESSAGE(1), &spi_transfer);
		*buf++ = rx_buf[1];
		ioctl(spi_fd, SPI_IOC_MESSAGE(1), &spi_transfer);
		*buf++ = rx_buf[1];
		ioctl(spi_fd, SPI_IOC_MESSAGE(1), &spi_transfer);
		*buf++ = rx_buf[1];
		ioctl(spi_fd, SPI_IOC_MESSAGE(1), &spi_transfer);
		*buf++ = rx_buf[1];
	}

	while (rem--) {
		ioctl(spi_fd, SPI_IOC_MESSAGE(1), &spi_transfer);
		*buf++ = rx_buf[1];
	}
}

void fpga_spi_fast_block_write_be(const uint16_t *buf, uint32_t length)
{
	printf("*** fpga_spi_fast_block_write_be(%d) is not implemented ***\n", length);
}

void fpga_spi_fast_block_read_be(uint16_t *buf, uint32_t length)
{
	printf("*** fpga_spi_fast_block_read_be(%d) is not implemented ***\n", length);
}
