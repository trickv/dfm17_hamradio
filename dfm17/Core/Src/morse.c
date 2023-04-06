/*
 * a morse dit (short) is a 10.
 * a morse dah (long) is a 1110.
 * these get interpreted by the radio as a binary stream
 *
 * Ported from https://github.com/DL7AD/pecanpico9/blob/master/tracker/software/protocols/morse/morse.c
 */

#include <string.h>

static uint32_t morse_position;

/*
 * ADDB: add a bit to the morse_position of the buffer
 *
 * DFM17 note: this was ported from pecanpico9 but the bits are being
 * interpreted backwards from what this code did on pecanpico9. This shifts bits
 * to the MSB first, rather than LSB.
 */
#define ADDB(buffer, bit) { \
	buffer[morse_position/8] |= ((bit & 0x1) << (7-(morse_position % 8))); \
	morse_position++; \
}

// adds a 1110 (long tone + short space) to the buffer
void dah(uint8_t *buffer)
{
	ADDB(buffer, 1);
	ADDB(buffer, 1);
	ADDB(buffer, 1);
	ADDB(buffer, 0);
}

// adds a 10 (short tone + short space) to the buffer
void dit(uint8_t *buffer)
{
	ADDB(buffer, 1);
	ADDB(buffer, 0);
}

// adds the specified number of spaces to the buffer
void blank(uint8_t *buffer, uint32_t ticks) {
	for(uint32_t i=0; i<ticks; i++)
		ADDB(buffer, 0);
}

// encode a single charachter (char letter) into morse and append it
// to (uint buffer) the buffer
void morse_encode_char(uint8_t *buffer, char letter)
{
	switch(letter) {
		case 'A':
			dit(buffer);
			dah(buffer);
		break;
		case 'B':
			dah(buffer);
			dit(buffer);
			dit(buffer);
			dit(buffer);
		break;
		case 'C':
			dah(buffer);
			dit(buffer);
			dah(buffer);
			dit(buffer);
		break;
		case 'D':
			dah(buffer);
			dit(buffer);
			dit(buffer);
		break;
		case 'E':
			dit(buffer);
		break;
		case 'F':
			dit(buffer);
			dit(buffer);
			dah(buffer);
			dit(buffer);
		break;
		case 'G':
			dah(buffer);
			dah(buffer);
			dit(buffer);
		break;
		case 'H':
			dit(buffer);
			dit(buffer);
			dit(buffer);
			dit(buffer);
		break;
		case 'I':
			dit(buffer);
			dit(buffer);
		break;
		case 'J':
			dit(buffer);
			dah(buffer);
			dah(buffer);
			dah(buffer);
		break;
		case 'K':
			dah(buffer);
			dit(buffer);
			dah(buffer);
		break;
		case 'L':
			dit(buffer);
			dah(buffer);
			dit(buffer);
			dit(buffer);
		break;
		case 'M':
			dah(buffer);
			dah(buffer);
		break;
		case 'N':
			dah(buffer);
			dit(buffer);
		break;
		case 'O':
			dah(buffer);
			dah(buffer);
			dah(buffer);
		break;
		case 'P':
			dit(buffer);
			dah(buffer);
			dah(buffer);
			dit(buffer);
		break;
		case 'Q':
			dah(buffer);
			dah(buffer);
			dit(buffer);
			dah(buffer);
		break;
		case 'R':
			dit(buffer);
			dah(buffer);
			dit(buffer);
		break;
		case 'S':
			dit(buffer);
			dit(buffer);
			dit(buffer);
		break;
		case 'T':
			dah(buffer);
		break;
		case 'U':
			dit(buffer);
			dit(buffer);
			dah(buffer);
		break;
		case 'V':
			dit(buffer);
			dit(buffer);
			dit(buffer);
			dah(buffer);
		break;
		case 'W':
			dit(buffer);
			dah(buffer);
			dah(buffer);
		break;
		case 'X':
			dah(buffer);
			dit(buffer);
			dit(buffer);
			dah(buffer);
		break;
		case 'Y':
			dah(buffer);
			dit(buffer);
			dah(buffer);
			dah(buffer);
		break;
		case 'Z':
			dah(buffer);
			dah(buffer);
			dit(buffer);
			dit(buffer);
		break;
		case '1':
			dit(buffer);
			dah(buffer);
			dah(buffer);
			dah(buffer);
			dah(buffer);
		break;
		case '2':
			dit(buffer);
			dit(buffer);
			dah(buffer);
			dah(buffer);
			dah(buffer);
		break;
		case '3':
			dit(buffer);
			dit(buffer);
			dit(buffer);
			dah(buffer);
			dah(buffer);
		break;
		case '4':
			dit(buffer);
			dit(buffer);
			dit(buffer);
			dit(buffer);
			dah(buffer);
		break;
		case '5':
			dit(buffer);
			dit(buffer);
			dit(buffer);
			dit(buffer);
			dit(buffer);
		break;
		case '6':
			dah(buffer);
			dit(buffer);
			dit(buffer);
			dit(buffer);
			dit(buffer);
		break;
		case '7':
			dah(buffer);
			dah(buffer);
			dit(buffer);
			dit(buffer);
			dit(buffer);
		break;
		case '8':
			dah(buffer);
			dah(buffer);
			dah(buffer);
			dit(buffer);
			dit(buffer);
		break;
		case '9':
			dah(buffer);
			dah(buffer);
			dah(buffer);
			dah(buffer);
			dit(buffer);
		break;
		case '0':
			dah(buffer);
			dah(buffer);
			dah(buffer);
			dah(buffer);
			dah(buffer);
		break;
		case ' ':
			blank(buffer, 3);
		break;
		case '.':
			dit(buffer);
			dah(buffer);
			dit(buffer);
			dah(buffer);
			dit(buffer);
			dah(buffer);
		break;
	}
	blank(buffer, 4);
}

/*
 * morse_encode(output buffer, length of outoput buffer, input buffer)
 * encodes a char array into morse binary stream format needed by the radio.
 * stops when we find a null terminator or we hit the output buffer length
 * returns the number of bits in the outpuy buffer
 */
uint32_t morse_encode(uint8_t* buffer, uint32_t length, const char* in)
{
	memset(buffer, 0, length); // Tidy up
	morse_position = 0; // Bitlength

	// Encode morse
	for(uint32_t i=0; in[i] != 0 && morse_position < length*8; i++)
		morse_encode_char(buffer, in[i]);

	return morse_position;
}
