
MEMORY
{
	ROM (rx)  : ORIGIN = 0x08000000, LENGTH = 1m
	RAM (rwx) : ORIGIN = 0x20000000, LENGTH = 96k
}

INCLUDE ../../common/memory_sections.ld
