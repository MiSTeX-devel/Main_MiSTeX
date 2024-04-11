#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>
#include <signal.h>
#include <ctype.h>
#include <termios.h>
#include <sys/types.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <fcntl.h>

#include "shmem.h"

static int memfd = -1;

void *shmem_map(uint32_t address, uint32_t size)
{
	printf("******** TODO: shmem_map not implemented\n");
	printf("*********addr: 0x%8x size: %d\n", address, size);
	return 0;
}

int shmem_unmap(void* map, uint32_t size)
{
	printf("******** TODO: shmem_unmap not implemented\n");
	printf("*********size: %d\n", size);

	return 1;
}

int shmem_put(uint32_t address, uint32_t size, void *buf)
{
	printf("******** TODO: shmem_put not implemented\n");
	printf("*********addr: 0x%8x size: %d\n", address, size);

	return 1;
}

int shmem_get(uint32_t address, uint32_t size, void *buf)
{
	printf("******** TODO: shmem_get not implemented\n");
	printf("*********addr: 0x%8x size: %d\n", address, size);

	return 1;
}
