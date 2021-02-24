/*
 * MY9221.c
 *
 * Defines necessary functions to control a MY9221 LED driver
 *
 *  Created on: 08 Feb 2021
 *      Author: Maksim Drachov
 */

#include <stdio.h>
#include "stdbool.h"

void CMDArray_Init(bool *CMDArray, int hspd, int bs, int gck, int sep, int osc, int pol, int cntset, int onest)
{
	switch (hspd)
	{
		case 0:
			printf("MY9221: Slow mode selected");
			CMDArray[10] = false;
			break;

		case 1:
			printf("MY9221: Fast mode selected");
			CMDArray[10] = true;
			break;

		default:
			printf("MY9221: MODE BAD INPUT (CMDArray_Init)");
			CMDArray[10] = -1;
			break;
	}

	switch (bs)
	{
		case 0:
			printf("MY9221: 8-bit grayscale selected");
			CMDArray[8] = false;
			CMDArray[9] = false;
			break;

		case 1:
			printf("MY9221: 12-bit grayscale selected");
			CMDArray[8] = true;
			CMDArray[9] = false;
			break;

		case 2:
			printf("MY9221: 14-bit grayscale selected");
			CMDArray[8] = false;
			CMDArray[9] = true;
			break;

		case 3:
			printf("MY9221: 16-bit grayscale selected");
			CMDArray[8] = true;
			CMDArray[9] = true;
			break;

		default:
			printf("MY9221: GRAYSCALE BAD INPUT (CMDArray_Init)");
			CMDArray[8] = -1;
			CMDArray[9] = -1;
			break;
	}

	switch (gck)
	{
		case 0:
			printf("MY9221: original freq selected");
			CMDArray[5] = false;
			CMDArray[6] = false;
			CMDArray[7] = false;
			break;

		case 1:
			printf("MY9221: original freq/2 selected");
			CMDArray[5] = true;
			CMDArray[6] = false;
			CMDArray[7] = false;
			break;

		case 2:
			printf("MY9221: original freq/4 selected");
			CMDArray[5] = false;
			CMDArray[6] = true;
			CMDArray[7] = false;
			break;

		case 3:
			printf("MY9221: original freq/8 selected");
			CMDArray[5] = true;
			CMDArray[6] = true;
			CMDArray[7] = false;
			break;

		case 4:
			printf("MY9221: original freq/16 selected");
			CMDArray[5] = false;
			CMDArray[6] = false;
			CMDArray[7] = true;
			break;

		case 5:
			printf("MY9221: original freq/64 selected");
			CMDArray[5] = true;
			CMDArray[6] = false;
			CMDArray[7] = true;
			break;

		case 6:
			printf("MY9221: original freq/128 selected");
			CMDArray[5] = false;
			CMDArray[6] = true;
			CMDArray[7] = true;
			break;

		case 7:
			printf("MY9221: original freq/256 selected");
			CMDArray[5] = true;
			CMDArray[6] = true;
			CMDArray[7] = true;
			break;

		default:
			printf("MY9221: INTERNAL OSCILLATOR BAD INPUT (CMDArray_Init)");
			CMDArray[5] = -1;
			CMDArray[6] = -1;
			CMDArray[7] = -1;
			break;

	}

	switch (sep)
	{
		case 0:
			printf("MY9221: MY-PWM output waveform selected");
			CMDArray[4] = false;
			break;

		case 1:
			printf("MY9221: APDM output waveform selected");
			CMDArray[4] = true;
			break;

		default:
			printf("MY9221: OUTPUT WAVEFORM BAD INPUT (CMDArray_Init)");
			CMDArray[4] = -1;
			break;
	}

	switch (osc)
	{
		case 0:
			printf("MY9221: Internal oscillator selected");
			CMDArray[3] = false;
			break;

		case 1:
			printf("MY9221: External oscillator selected");
			CMDArray[3] = true;
			CMDArray[7] = false;
			CMDArray[6] = false;
			CMDArray[5] = false;
			break;

		default:
			printf("MY9221: GRAYSCALE CLOCK BAD INPUT (CMDArray_Init)");
			CMDArray[3] = -1;
			break;
	}

	switch (pol)
	{
		case 0:
			printf("MY9221: LED driver selected");
			CMDArray[2] = false;
			break;

		case 1:
			printf("MY9221: MY-PWM/APDM selected");
			CMDArray[2] = true;
			break;

		default:
			printf("MY9221: OUTPUT POLARITY BAD INPUT (CMDArray_Init)");
			CMDArray[2] = -1;
			break;
	}

	switch (cntset)
	{
		case 0:
			printf("MY9221: Free running mode selected");
			CMDArray[1] = false;
			break;

		case 1:
			if (CMDArray[3] != 1)
			{
				printf("MY9221: MODE BAD INPUT (CMDArray_Init) [osc needs to be 1!]");
				CMDArray[1] = -1;
				break;
			}
			printf("MY9221: Counter reset mode selected");
			CMDArray[1] = true;
			break;

		default:
			printf("MY9221: MODE BAD INPUT (CMDArray_Init)");
			CMDArray[1] = -1;
			break;
	}

	switch (onest)
	{
		case 0:
			printf("MY9221: Frame cycle repeat mode selected");
			CMDArray[0] = false;
			break;

		case 1:
			if (CMDArray[1] != 1)
			{
				printf("MY9221: CNTSET NEEDS TO BE 1! (CMDArray_Init)");
				CMDArray[0] = -1;
				break;
			}
			printf("MY9221: Frame cycle one-shot mode selected");
			CMDArray[0] = true;
			break;

		default:
			printf("MY9221: ONE-SHOT SELECT BAD INPUT (CMDArray_Init)");
			CMDArray[0] = -1;
			break;
	}
}

void Grayscale_Init(bool *GrArr, int bs, bool *ClrArr)
{
	int GrayscaleBits;
	int Frame;
	int Bit;

	switch (bs)
	{
		case 0:
			printf("MY9221: 8-bit grayscale selected");
			GrayscaleBits = 8;
			break;

		case 1:
			printf("MY9221: 12-bit grayscale selected");
			GrayscaleBits = 12;
			break;

		case 2:
			printf("MY9221: 14-bit grayscale selected");
			GrayscaleBits = 14;
			break;

		case 3:
			printf("MY9221: 16-bit grayscale selected");
			GrayscaleBits = 16;
			break;

		default:
			printf("MY9221: GRAYSCALE BAD INPUT (Grayscale_Init)");
			break;
	}

	for (Frame = 11; Frame >= 0; --Frame)
	{
		for (Bit = 0; Bit < GrayscaleBits; ++Bit)
		{
			GrArr[Frame*16+Bit] = ClrArr[Bit];
		}
		for (Bit = GrayscaleBits; Bit < (16-GrayscaleBits); ++Bit)
		{
			GrArr[Frame*16+Bit] = false;
		}

	}
}

void InputArray_Init(uint32_t *InputArray, bool *CMDArray, bool *GrArr)
{
	int i;
	int LED_Line;

	for (LED_Line = 0; LED_Line <16; LED_Line++)
	{
		for (i = 0; i < 192; ++i)
		{
			if (GrArr[i] == true)
			{
				// SET BIT
				InputArray[i] |= (1 << LED_Line);
				InputArray[i] &= ~(1 << (LED_Line+16));
			}
			else
			{
				// RESET BIT
				InputArray[i] |= (1 << (LED_Line+16));
				InputArray[i] &= ~(1 << LED_Line);
			}
		}
		for (i = 192; i < 208; ++i)
		{
			if (CMDArray[i-192] == true)
			{
				// SET BIT
				InputArray[i] |= (1 << LED_Line);
				InputArray[i] &= ~(1 << (LED_Line+16));
			}
			else
			{
				// RESET BIT
				InputArray[i] |= (1 << (LED_Line+16));
				InputArray[i] &= ~(1 << LED_Line);
			}
		}
	}

}



