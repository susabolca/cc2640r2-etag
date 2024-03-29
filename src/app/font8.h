// Created by http://oleddisplay.squix.ch/ Consider a donation
// In case of problems make sure that you are using the font file with the correct version!
const uint8_t Dialog_plain_8Bitmaps[] = {

	// Bitmap Data:
	0x00, // ' '
	0xF4, // '!'
	0xB4, // '"'
	0x32,0xBE,0xAF,0xB2,0x80, // '#'
	0x27,0xE8,0xF2,0xFC,0x80, // '$'
	0xE9,0x53,0xC0,0xF2,0xA5,0xC0, // '%'
	0x72,0x19,0xDB,0x3C, // '&'
	0xC0, // '''
	0x6A,0xA4, // '('
	0x95,0x58, // ')'
	0xAB,0x9D,0x50, // '*'
	0x21,0x3E,0x42,0x00, // '+'
	0xC0, // ','
	0xC0, // '-'
	0x80, // '.'
	0x25,0x25,0x20, // '/'
	0x69,0x99,0x96, // '0'
	0xC9,0x25,0xC0, // '1'
	0xE1,0x12,0x4F, // '2'
	0xE1,0x16,0x1F, // '3'
	0x26,0x6A,0xF2, // '4'
	0xF8,0xE1,0x1E, // '5'
	0x7C,0x8F,0x97, // '6'
	0xF1,0x22,0x24, // '7'
	0x69,0x96,0x9F, // '8'
	0xE9,0xF1,0x3E, // '9'
	0x90, // ':'
	0x98, // ';'
	0x0B,0xBC,0x10, // '<'
	0xF8,0x3E, // '='
	0x83,0x9F,0x00, // '>'
	0xE5,0x20,0x80, // '?'
	0x3C,0x8E,0xED,0x5B,0xC8,0x0E,0x00, // '@'
	0x21,0x14,0xA7,0x44, // 'A'
	0xF9,0x9E,0x9F, // 'B'
	0x76,0x61,0x0C,0x3C, // 'C'
	0xF4,0x63,0x18,0xF8, // 'D'
	0xF8,0x8F,0x8F, // 'E'
	0xF2,0x79,0x00, // 'F'
	0x76,0x67,0x18,0xB8, // 'G'
	0x99,0x9F,0x99, // 'H'
	0xFC, // 'I'
	0x55,0x57, // 'J'
	0x95,0x31,0xCB,0x4C, // 'K'
	0x88,0x88,0x8F, // 'L'
	0xDE,0xFF,0x5A,0xC4, // 'M'
	0x9D,0xDB,0xB9, // 'N'
	0x74,0x63,0x18,0xB8, // 'O'
	0xF9,0xF8,0x88, // 'P'
	0x74,0x63,0x18,0xB8,0x40, // 'Q'
	0xF4,0xB9,0x69,0x44, // 'R'
	0x78,0xE3,0x1F, // 'S'
	0xF9,0x08,0x42,0x10, // 'T'
	0x99,0x99,0x96, // 'U'
	0x8A,0x94,0xA2,0x10, // 'V'
	0x93,0x55,0xB3,0x66,0xC8,0x80, // 'W'
	0xDA,0x88,0x45,0x44, // 'X'
	0xDA,0x88,0x42,0x10, // 'Y'
	0xF8,0x88,0xC4,0x7C, // 'Z'
	0xEA,0xAC, // '['
	0x91,0x24,0x48, // '\'
	0xD5,0x5C, // ']'
	0x26,0x80, // '^'
	0xF0, // '_'
	0x80, // '`'
	0x7F,0x9F, // 'a'
	0x88,0x8E,0x99,0xE0, // 'b'
	0x72,0x30, // 'c'
	0x11,0x17,0x99,0x70, // 'd'
	0x7F,0x87, // 'e'
	0x69,0x74,0x90, // 'f'
	0x79,0x97,0x16, // 'g'
	0x88,0x8F,0x99,0x90, // 'h'
	0xBC, // 'i'
	0x45,0x57, // 'j'
	0x88,0x8A,0xCC,0xA0, // 'k'
	0xFE, // 'l'
	0xFF,0x26,0x4C,0x90, // 'm'
	0xF9,0x99, // 'n'
	0x69,0x96, // 'o'
	0xE9,0x9E,0x88, // 'p'
	0x79,0x97,0x11, // 'q'
	0xF2,0x40, // 'r'
	0xF8,0xF0, // 's'
	0x5D,0x26, // 't'
	0x99,0x9F, // 'u'
	0x96,0x66, // 'v'
	0xB6,0xD4,0x92, // 'w'
	0xF6,0x6F, // 'x'
	0x96,0x64,0x4C, // 'y'
	0xF2,0x6F, // 'z'
	0x69,0x44,0x98, // '{'
	0xFF, // '|'
	0xC9,0x14,0xB0 // '}'
};
const GFXglyph Dialog_plain_8Glyphs[] = {
// bitmapOffset, width, height, xAdvance, xOffset, yOffset
	  {     0,   1,   1,   4,    0,   -1 }, // ' '
	  {     1,   1,   6,   4,    1,   -6 }, // '!'
	  {     2,   3,   2,   5,    1,   -6 }, // '"'
	  {     3,   5,   7,   8,    1,   -7 }, // '#'
	  {     8,   5,   7,   6,    1,   -6 }, // '$'
	  {    13,   7,   6,   9,    1,   -6 }, // '%'
	  {    19,   5,   6,   7,    1,   -6 }, // '&'
	  {    23,   1,   2,   3,    1,   -6 }, // '''
	  {    24,   2,   7,   4,    1,   -7 }, // '('
	  {    26,   2,   7,   4,    1,   -7 }, // ')'
	  {    28,   5,   4,   5,    0,   -6 }, // '*'
	  {    31,   5,   5,   8,    1,   -5 }, // '+'
	  {    35,   1,   2,   4,    1,   -1 }, // ','
	  {    36,   2,   1,   4,    1,   -3 }, // '-'
	  {    37,   1,   1,   4,    1,   -1 }, // '.'
	  {    38,   3,   7,   4,    0,   -6 }, // '/'
	  {    41,   4,   6,   6,    1,   -6 }, // '0'
	  {    44,   3,   6,   6,    1,   -6 }, // '1'
	  {    47,   4,   6,   6,    1,   -6 }, // '2'
	  {    50,   4,   6,   6,    1,   -6 }, // '3'
	  {    53,   4,   6,   6,    1,   -6 }, // '4'
	  {    56,   4,   6,   6,    1,   -6 }, // '5'
	  {    59,   4,   6,   6,    1,   -6 }, // '6'
	  {    62,   4,   6,   6,    1,   -6 }, // '7'
	  {    65,   4,   6,   6,    1,   -6 }, // '8'
	  {    68,   4,   6,   6,    1,   -6 }, // '9'
	  {    71,   1,   4,   4,    1,   -4 }, // ':'
	  {    72,   1,   5,   4,    1,   -4 }, // ';'
	  {    73,   5,   4,   8,    1,   -5 }, // '<'
	  {    76,   5,   3,   8,    1,   -4 }, // '='
	  {    78,   5,   4,   8,    1,   -5 }, // '>'
	  {    81,   3,   6,   5,    1,   -6 }, // '?'
	  {    84,   7,   7,   9,    1,   -6 }, // '@'
	  {    91,   5,   6,   6,    0,   -6 }, // 'A'
	  {    95,   4,   6,   6,    1,   -6 }, // 'B'
	  {    98,   5,   6,   7,    1,   -6 }, // 'C'
	  {   102,   5,   6,   7,    1,   -6 }, // 'D'
	  {   106,   4,   6,   6,    1,   -6 }, // 'E'
	  {   109,   3,   6,   6,    1,   -6 }, // 'F'
	  {   112,   5,   6,   7,    1,   -6 }, // 'G'
	  {   116,   4,   6,   7,    1,   -6 }, // 'H'
	  {   119,   1,   6,   3,    1,   -6 }, // 'I'
	  {   120,   2,   8,   3,    0,   -6 }, // 'J'
	  {   122,   5,   6,   6,    1,   -6 }, // 'K'
	  {   126,   4,   6,   5,    1,   -6 }, // 'L'
	  {   129,   5,   6,   8,    1,   -6 }, // 'M'
	  {   133,   4,   6,   7,    1,   -6 }, // 'N'
	  {   136,   5,   6,   7,    1,   -6 }, // 'O'
	  {   140,   4,   6,   6,    1,   -6 }, // 'P'
	  {   143,   5,   7,   7,    1,   -6 }, // 'Q'
	  {   148,   5,   6,   7,    1,   -6 }, // 'R'
	  {   152,   4,   6,   6,    1,   -6 }, // 'S'
	  {   155,   5,   6,   6,    0,   -6 }, // 'T'
	  {   159,   4,   6,   7,    1,   -6 }, // 'U'
	  {   162,   5,   6,   6,    0,   -6 }, // 'V'
	  {   166,   7,   6,   9,    0,   -6 }, // 'W'
	  {   172,   5,   6,   6,    0,   -6 }, // 'X'
	  {   176,   5,   6,   6,    0,   -6 }, // 'Y'
	  {   180,   5,   6,   6,    1,   -6 }, // 'Z'
	  {   184,   2,   7,   4,    1,   -6 }, // '['
	  {   186,   3,   7,   4,    0,   -6 }, // '\'
	  {   189,   2,   7,   4,    1,   -6 }, // ']'
	  {   191,   5,   2,   8,    1,   -6 }, // '^'
	  {   193,   4,   1,   5,    0,    1 }, // '_'
	  {   194,   2,   1,   5,    1,   -6 }, // '`'
	  {   195,   4,   4,   6,    1,   -4 }, // 'a'
	  {   197,   4,   7,   6,    1,   -7 }, // 'b'
	  {   201,   3,   4,   5,    1,   -4 }, // 'c'
	  {   203,   4,   7,   6,    1,   -7 }, // 'd'
	  {   207,   4,   4,   6,    1,   -4 }, // 'e'
	  {   209,   3,   7,   4,    0,   -7 }, // 'f'
	  {   212,   4,   6,   6,    1,   -4 }, // 'g'
	  {   215,   4,   7,   6,    1,   -7 }, // 'h'
	  {   219,   1,   6,   3,    1,   -6 }, // 'i'
	  {   220,   2,   8,   3,    0,   -6 }, // 'j'
	  {   222,   4,   7,   6,    1,   -7 }, // 'k'
	  {   226,   1,   7,   3,    1,   -7 }, // 'l'
	  {   227,   7,   4,   9,    1,   -4 }, // 'm'
	  {   231,   4,   4,   6,    1,   -4 }, // 'n'
	  {   233,   4,   4,   6,    1,   -4 }, // 'o'
	  {   235,   4,   6,   6,    1,   -4 }, // 'p'
	  {   238,   4,   6,   6,    1,   -4 }, // 'q'
	  {   241,   3,   4,   4,    1,   -4 }, // 'r'
	  {   243,   3,   4,   5,    1,   -4 }, // 's'
	  {   245,   3,   5,   4,    0,   -5 }, // 't'
	  {   247,   4,   4,   6,    1,   -4 }, // 'u'
	  {   249,   4,   4,   6,    0,   -4 }, // 'v'
	  {   251,   6,   4,   8,    0,   -4 }, // 'w'
	  {   254,   4,   4,   6,    0,   -4 }, // 'x'
	  {   256,   4,   6,   6,    1,   -4 }, // 'y'
	  {   259,   4,   4,   5,    1,   -4 }, // 'z'
	  {   261,   3,   7,   6,    1,   -6 }, // '{'
	  {   264,   1,   8,   4,    1,   -6 }, // '|'
	  {   265,   3,   7,   6,    1,   -6 } // '}'
};
const GFXfont Dialog_plain_8 = {
(uint8_t  *)Dialog_plain_8Bitmaps,(GFXglyph *)Dialog_plain_8Glyphs,0x20, 0x7E, 10};