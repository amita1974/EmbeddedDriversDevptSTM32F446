/*
 * test_printf_itm_swv.c
 *
 *  Created on: Sep 10 2020
 *      Author: AA
 */


// short test code to demonstrate that the prints are sometimes not getting out in full and sometimes a complete print may be missing.

#include <stdio.h>

int main (void) {
	uint16_t i = 0;
	while (1) {
			printf("%03d Hello world, this is a beautiful day!!!  and I am writing a longer text in the print message 123456789012345678901234567890123456789012345678901234567890\n", i++);
			for (uint32_t j = 0; j < 500000; j++);
	}
	return 0;
}
