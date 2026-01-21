#![no_main]
#![no_std]

extern crate alloc;
use alloc::vec;

use stm32h7xx_hal::{ethernet, pac, prelude::*, spi, stm32};
use stm32h755zi as _;

use smoltcp::{
    iface::{Config, Interface, SocketSet},
    socket::tcp,
    time::Instant,
    wire::{EthernetAddress, IpAddress, IpCidr},
};

use embedded_alloc::LlffHeap as EmbeddedAllocator;

// Locally administered MAC address
const MAC_ADDRESS: [u8; 6] = [0x02, 0x00, 0x11, 0x22, 0x33, 0x44];

const ARDUCHIP_TEST1: u8 = 0x00;
const ARDUCHIP_FIFO: u8 = 0x04;
const ARDUCHIP_TRIG: u8 = 0x41;
const ARDUCHIP_FIFO_SIZE1: u8 = 0x42;
const ARDUCHIP_FIFO_SIZE2: u8 = 0x43;
const ARDUCHIP_FIFO_SIZE3: u8 = 0x44;
const ARDUCHIP_BURST_FIFO_READ: u8 = 0x3C;

const SENSOR_ADDRESS: u8 = 0x30;

const OV2640_CHIPID_HIGH: u8 = 0x0A;
const OV2640_CHIPID_LOW: u8 = 0x0B;

const OV2640_QVGA: [[u8; 2]; 193] = [
    [0xff, 0x0],
    [0x2c, 0xff],
    [0x2e, 0xdf],
    [0xff, 0x1],
    [0x3c, 0x32],
    [0x11, 0x0],
    [0x9, 0x2],
    [0x4, 0xa8],
    [0x13, 0xe5],
    [0x14, 0x48],
    [0x2c, 0xc],
    [0x33, 0x78],
    [0x3a, 0x33],
    [0x3b, 0xfb],
    [0x3e, 0x0],
    [0x43, 0x11],
    [0x16, 0x10],
    [0x39, 0x2],
    [0x35, 0x88],
    [0x22, 0xa],
    [0x37, 0x40],
    [0x23, 0x0],
    [0x34, 0xa0],
    [0x6, 0x2],
    [0x6, 0x88],
    [0x7, 0xc0],
    [0xd, 0xb7],
    [0xe, 0x1],
    [0x4c, 0x0],
    [0x4a, 0x81],
    [0x21, 0x99],
    [0x24, 0x40],
    [0x25, 0x38],
    [0x26, 0x82],
    [0x5c, 0x0],
    [0x63, 0x0],
    [0x46, 0x22],
    [0xc, 0x3a],
    [0x5d, 0x55],
    [0x5e, 0x7d],
    [0x5f, 0x7d],
    [0x60, 0x55],
    [0x61, 0x70],
    [0x62, 0x80],
    [0x7c, 0x5],
    [0x20, 0x80],
    [0x28, 0x30],
    [0x6c, 0x0],
    [0x6d, 0x80],
    [0x6e, 0x0],
    [0x70, 0x2],
    [0x71, 0x94],
    [0x73, 0xc1],
    [0x3d, 0x34],
    [0x12, 0x4],
    [0x5a, 0x57],
    [0x4f, 0xbb],
    [0x50, 0x9c],
    [0xff, 0x0],
    [0xe5, 0x7f],
    [0xf9, 0xc0],
    [0x41, 0x24],
    [0xe0, 0x14],
    [0x76, 0xff],
    [0x33, 0xa0],
    [0x42, 0x20],
    [0x43, 0x18],
    [0x4c, 0x0],
    [0x87, 0xd0],
    [0x88, 0x3f],
    [0xd7, 0x3],
    [0xd9, 0x10],
    [0xd3, 0x82],
    [0xc8, 0x8],
    [0xc9, 0x80],
    [0x7c, 0x0],
    [0x7d, 0x0],
    [0x7c, 0x3],
    [0x7d, 0x48],
    [0x7d, 0x48],
    [0x7c, 0x8],
    [0x7d, 0x20],
    [0x7d, 0x10],
    [0x7d, 0xe],
    [0x90, 0x0],
    [0x91, 0xe],
    [0x91, 0x1a],
    [0x91, 0x31],
    [0x91, 0x5a],
    [0x91, 0x69],
    [0x91, 0x75],
    [0x91, 0x7e],
    [0x91, 0x88],
    [0x91, 0x8f],
    [0x91, 0x96],
    [0x91, 0xa3],
    [0x91, 0xaf],
    [0x91, 0xc4],
    [0x91, 0xd7],
    [0x91, 0xe8],
    [0x91, 0x20],
    [0x92, 0x0],
    [0x93, 0x6],
    [0x93, 0xe3],
    [0x93, 0x3],
    [0x93, 0x3],
    [0x93, 0x0],
    [0x93, 0x2],
    [0x93, 0x0],
    [0x93, 0x0],
    [0x93, 0x0],
    [0x93, 0x0],
    [0x93, 0x0],
    [0x93, 0x0],
    [0x93, 0x0],
    [0x96, 0x0],
    [0x97, 0x8],
    [0x97, 0x19],
    [0x97, 0x2],
    [0x97, 0xc],
    [0x97, 0x24],
    [0x97, 0x30],
    [0x97, 0x28],
    [0x97, 0x26],
    [0x97, 0x2],
    [0x97, 0x98],
    [0x97, 0x80],
    [0x97, 0x0],
    [0x97, 0x0],
    [0xa4, 0x0],
    [0xa8, 0x0],
    [0xc5, 0x11],
    [0xc6, 0x51],
    [0xbf, 0x80],
    [0xc7, 0x10],
    [0xb6, 0x66],
    [0xb8, 0xa5],
    [0xb7, 0x64],
    [0xb9, 0x7c],
    [0xb3, 0xaf],
    [0xb4, 0x97],
    [0xb5, 0xff],
    [0xb0, 0xc5],
    [0xb1, 0x94],
    [0xb2, 0xf],
    [0xc4, 0x5c],
    [0xa6, 0x0],
    [0xa7, 0x20],
    [0xa7, 0xd8],
    [0xa7, 0x1b],
    [0xa7, 0x31],
    [0xa7, 0x0],
    [0xa7, 0x18],
    [0xa7, 0x20],
    [0xa7, 0xd8],
    [0xa7, 0x19],
    [0xa7, 0x31],
    [0xa7, 0x0],
    [0xa7, 0x18],
    [0xa7, 0x20],
    [0xa7, 0xd8],
    [0xa7, 0x19],
    [0xa7, 0x31],
    [0xa7, 0x0],
    [0xa7, 0x18],
    [0x7f, 0x0],
    [0xe5, 0x1f],
    [0xe1, 0x77],
    [0xdd, 0x7f],
    [0xc2, 0xe],
    [0xff, 0x0],
    [0xe0, 0x4],
    [0xc0, 0xc8],
    [0xc1, 0x96],
    [0x86, 0x3d],
    [0x51, 0x90],
    [0x52, 0x2c],
    [0x53, 0x0],
    [0x54, 0x0],
    [0x55, 0x88],
    [0x57, 0x0],
    [0x50, 0x92],
    [0x5a, 0x50],
    [0x5b, 0x3c],
    [0x5c, 0x0],
    [0xd3, 0x4],
    [0xe0, 0x0],
    [0xff, 0x0],
    [0x5, 0x0],
    [0xda, 0x8],
    [0xd7, 0x3],
    [0xe0, 0x0],
    [0x5, 0x0],
];

const IMAGE_HEADER_SIZE: usize = 1078;

const BMP_HEADER_GRAYSCALED: [u8; 1078] = [
  // BMP header : 14 bytes
  0x42, 0x4D,              // BM
  0x36, 0x30, 0x01, 0x00,  // Size : 77878 bytes
  0x00, 0x00,              // Application
  0x00, 0x00,              // Application
  0x36, 0x04, 0x00, 0x00,  // Offset : 1078 bytes

  // DIB Header : 40 bytes
  0x28, 0x00, 0x00, 0x00,  // DIB Header size : 40 bytes
  0x40, 0x01, 0x00, 0x00,  // Image width : 320 px
  0xF0, 0x00, 0x00, 0x00,  // Image height : 240 px
  0x01, 0x00,              // Number of planes (1)
  0x08, 0x00,              // Bits per pixel : 8 bits
  0x00, 0x00, 0x00, 0x00,  // Compression method : 0 (NONE)
  0x00, 0x2C, 0x01, 0x00,  // Raw Bitmap size : 76800 bytes
  0xC4, 0x0E, 0x00, 0x00,  // Horizontal resolution : 3780 px/m
  0xC4, 0x0E, 0x00, 0x00,  // Vertical resolution : 3780 px/m
  0x00, 0x00, 0x00, 0x00,  // Colors in palette : 0 (256)
  0x00, 0x00, 0x00, 0x00,  // Importants colors : 0

  // Color Table : 256 * 4 = 1024 bytes
  0, 0, 0, 255,        //   0
  1, 1, 1, 255,        //   1
  2, 2, 2, 255,        //   2
  3, 3, 3, 255,        //   3
  4, 4, 4, 255,        //   4
  5, 5, 5, 255,        //   5
  6, 6, 6, 255,        //   6
  7, 7, 7, 255,        //   7
  8, 8, 8, 255,        //   8
  9, 9, 9, 255,        //   9
  10, 10, 10, 255,     //  10
  11, 11, 11, 255,     //  11
  12, 12, 12, 255,     //  12
  13, 13, 13, 255,     //  13
  14, 14, 14, 255,     //  14
  15, 15, 15, 255,     //  15
  16, 16, 16, 255,     //  16
  17, 17, 17, 255,     //  17
  18, 18, 18, 255,     //  18
  19, 19, 19, 255,     //  19
  20, 20, 20, 255,     //  20
  21, 21, 21, 255,     //  21
  22, 22, 22, 255,     //  22
  23, 23, 23, 255,     //  23
  24, 24, 24, 255,     //  24
  25, 25, 25, 255,     //  25
  26, 26, 26, 255,     //  26
  27, 27, 27, 255,     //  27
  28, 28, 28, 255,     //  28
  29, 29, 29, 255,     //  29
  30, 30, 30, 255,     //  30
  31, 31, 31, 255,     //  31
  32, 32, 32, 255,     //  32
  33, 33, 33, 255,     //  33
  34, 34, 34, 255,     //  34
  35, 35, 35, 255,     //  35
  36, 36, 36, 255,     //  36
  37, 37, 37, 255,     //  37
  38, 38, 38, 255,     //  38
  39, 39, 39, 255,     //  39
  40, 40, 40, 255,     //  40
  41, 41, 41, 255,     //  41
  42, 42, 42, 255,     //  42
  43, 43, 43, 255,     //  43
  44, 44, 44, 255,     //  44
  45, 45, 45, 255,     //  45
  46, 46, 46, 255,     //  46
  47, 47, 47, 255,     //  47
  48, 48, 48, 255,     //  48
  49, 49, 49, 255,     //  49
  50, 50, 50, 255,     //  50
  51, 51, 51, 255,     //  51
  52, 52, 52, 255,     //  52
  53, 53, 53, 255,     //  53
  54, 54, 54, 255,     //  54
  55, 55, 55, 255,     //  55
  56, 56, 56, 255,     //  56
  57, 57, 57, 255,     //  57
  58, 58, 58, 255,     //  58
  59, 59, 59, 255,     //  59
  60, 60, 60, 255,     //  60
  61, 61, 61, 255,     //  61
  62, 62, 62, 255,     //  62
  63, 63, 63, 255,     //  63
  64, 64, 64, 255,     //  64
  65, 65, 65, 255,     //  65
  66, 66, 66, 255,     //  66
  67, 67, 67, 255,     //  67
  68, 68, 68, 255,     //  68
  69, 69, 69, 255,     //  69
  70, 70, 70, 255,     //  70
  71, 71, 71, 255,     //  71
  72, 72, 72, 255,     //  72
  73, 73, 73, 255,     //  73
  74, 74, 74, 255,     //  74
  75, 75, 75, 255,     //  75
  76, 76, 76, 255,     //  76
  77, 77, 77, 255,     //  77
  78, 78, 78, 255,     //  78
  79, 79, 79, 255,     //  79
  80, 80, 80, 255,     //  80
  81, 81, 81, 255,     //  81
  82, 82, 82, 255,     //  82
  83, 83, 83, 255,     //  83
  84, 84, 84, 255,     //  84
  85, 85, 85, 255,     //  85
  86, 86, 86, 255,     //  86
  87, 87, 87, 255,     //  87
  88, 88, 88, 255,     //  88
  89, 89, 89, 255,     //  89
  90, 90, 90, 255,     //  90
  91, 91, 91, 255,     //  91
  92, 92, 92, 255,     //  92
  93, 93, 93, 255,     //  93
  94, 94, 94, 255,     //  94
  95, 95, 95, 255,     //  95
  96, 96, 96, 255,     //  96
  97, 97, 97, 255,     //  97
  98, 98, 98, 255,     //  98
  99, 99, 99, 255,     //  99
  100, 100, 100, 255,  // 100
  101, 101, 101, 255,  // 101
  102, 102, 102, 255,  // 102
  103, 103, 103, 255,  // 103
  104, 104, 104, 255,  // 104
  105, 105, 105, 255,  // 105
  106, 106, 106, 255,  // 106
  107, 107, 107, 255,  // 107
  108, 108, 108, 255,  // 108
  109, 109, 109, 255,  // 109
  110, 110, 110, 255,  // 110
  111, 111, 111, 255,  // 111
  112, 112, 112, 255,  // 112
  113, 113, 113, 255,  // 113
  114, 114, 114, 255,  // 114
  115, 115, 115, 255,  // 115
  116, 116, 116, 255,  // 116
  117, 117, 117, 255,  // 117
  118, 118, 118, 255,  // 118
  119, 119, 119, 255,  // 119
  120, 120, 120, 255,  // 120
  121, 121, 121, 255,  // 121
  122, 122, 122, 255,  // 122
  123, 123, 123, 255,  // 123
  124, 124, 124, 255,  // 124
  125, 125, 125, 255,  // 125
  126, 126, 126, 255,  // 126
  127, 127, 127, 255,  // 127
  128, 128, 128, 255,  // 128
  129, 129, 129, 255,  // 129
  130, 130, 130, 255,  // 130
  131, 131, 131, 255,  // 131
  132, 132, 132, 255,  // 132
  133, 133, 133, 255,  // 133
  134, 134, 134, 255,  // 134
  135, 135, 135, 255,  // 135
  136, 136, 136, 255,  // 136
  137, 137, 137, 255,  // 137
  138, 138, 138, 255,  // 138
  139, 139, 139, 255,  // 139
  140, 140, 140, 255,  // 140
  141, 141, 141, 255,  // 141
  142, 142, 142, 255,  // 142
  143, 143, 143, 255,  // 143
  144, 144, 144, 255,  // 144
  145, 145, 145, 255,  // 145
  146, 146, 146, 255,  // 146
  147, 147, 147, 255,  // 147
  148, 148, 148, 255,  // 148
  149, 149, 149, 255,  // 149
  150, 150, 150, 255,  // 150
  151, 151, 151, 255,  // 151
  152, 152, 152, 255,  // 152
  153, 153, 153, 255,  // 153
  154, 154, 154, 255,  // 154
  155, 155, 155, 255,  // 155
  156, 156, 156, 255,  // 156
  157, 157, 157, 255,  // 157
  158, 158, 158, 255,  // 158
  159, 159, 159, 255,  // 159
  160, 160, 160, 255,  // 160
  161, 161, 161, 255,  // 161
  162, 162, 162, 255,  // 162
  163, 163, 163, 255,  // 163
  164, 164, 164, 255,  // 164
  165, 165, 165, 255,  // 165
  166, 166, 166, 255,  // 166
  167, 167, 167, 255,  // 167
  168, 168, 168, 255,  // 168
  169, 169, 169, 255,  // 169
  170, 170, 170, 255,  // 170
  171, 171, 171, 255,  // 171
  172, 172, 172, 255,  // 172
  173, 173, 173, 255,  // 173
  174, 174, 174, 255,  // 174
  175, 175, 175, 255,  // 175
  176, 176, 176, 255,  // 176
  177, 177, 177, 255,  // 177
  178, 178, 178, 255,  // 178
  179, 179, 179, 255,  // 179
  180, 180, 180, 255,  // 180
  181, 181, 181, 255,  // 181
  182, 182, 182, 255,  // 182
  183, 183, 183, 255,  // 183
  184, 184, 184, 255,  // 184
  185, 185, 185, 255,  // 185
  186, 186, 186, 255,  // 186
  187, 187, 187, 255,  // 187
  188, 188, 188, 255,  // 188
  189, 189, 189, 255,  // 189
  190, 190, 190, 255,  // 190
  191, 191, 191, 255,  // 191
  192, 192, 192, 255,  // 192
  193, 193, 193, 255,  // 193
  194, 194, 194, 255,  // 194
  195, 195, 195, 255,  // 195
  196, 196, 196, 255,  // 196
  197, 197, 197, 255,  // 197
  198, 198, 198, 255,  // 198
  199, 199, 199, 255,  // 199
  200, 200, 200, 255,  // 200
  201, 201, 201, 255,  // 201
  202, 202, 202, 255,  // 202
  203, 203, 203, 255,  // 203
  204, 204, 204, 255,  // 204
  205, 205, 205, 255,  // 205
  206, 206, 206, 255,  // 206
  207, 207, 207, 255,  // 207
  208, 208, 208, 255,  // 208
  209, 209, 209, 255,  // 209
  210, 210, 210, 255,  // 210
  211, 211, 211, 255,  // 211
  212, 212, 212, 255,  // 212
  213, 213, 213, 255,  // 213
  214, 214, 214, 255,  // 214
  215, 215, 215, 255,  // 215
  216, 216, 216, 255,  // 216
  217, 217, 217, 255,  // 217
  218, 218, 218, 255,  // 218
  219, 219, 219, 255,  // 219
  220, 220, 220, 255,  // 220
  221, 221, 221, 255,  // 221
  222, 222, 222, 255,  // 222
  223, 223, 223, 255,  // 223
  224, 224, 224, 255,  // 224
  225, 225, 225, 255,  // 225
  226, 226, 226, 255,  // 226
  227, 227, 227, 255,  // 227
  228, 228, 228, 255,  // 228
  229, 229, 229, 255,  // 229
  230, 230, 230, 255,  // 230
  231, 231, 231, 255,  // 231
  232, 232, 232, 255,  // 232
  233, 233, 233, 255,  // 233
  234, 234, 234, 255,  // 234
  235, 235, 235, 255,  // 235
  236, 236, 236, 255,  // 236
  237, 237, 237, 255,  // 237
  238, 238, 238, 255,  // 238
  239, 239, 239, 255,  // 239
  240, 240, 240, 255,  // 240
  241, 241, 241, 255,  // 241
  242, 242, 242, 255,  // 242
  243, 243, 243, 255,  // 243
  244, 244, 244, 255,  // 244
  245, 245, 245, 255,  // 245
  246, 246, 246, 255,  // 246
  247, 247, 247, 255,  // 247
  248, 248, 248, 255,  // 248
  249, 249, 249, 255,  // 249
  250, 250, 250, 255,  // 250
  251, 251, 251, 255,  // 251
  252, 252, 252, 255,  // 252
  253, 253, 253, 255,  // 253
  254, 254, 254, 255,  // 254
  255, 255, 255, 255,  // 255
];

const BMP_HEADER_COLORED: [u8; 66] = [
    // BMP header : 14 bytes
    0x42, 0x4D, // BM
    0x42, 0x58, 0x02, 0x00, // Size : 153666 bytes
    0x00, 0x00, // Application
    0x00, 0x00, // Application
    0x42, 0x00, 0x00, 0x00, // Offset : 66 bytes
    // DIB Header : 52 bytes
    0x28, 0x00, 0x00, 0x00, // DIB Header size : 40 bytes
    0x40, 0x01, 0x00, 0x00, // Image width : 320 px
    0xF0, 0x00, 0x00, 0x00, // Image height : 240 px
    0x01, 0x00,             // Color plane : 1
    0x10, 0x00,             // Bits per pixel : 16 bits
    0x03, 0x00, 0x00, 0x00, // Compression method : 3 (BITFIELDS)
    0x00, 0x58, 0x02, 0x00, // Raw Bitmap size : 153600 bytes
    0xC4, 0x0E, 0x00, 0x00, // Horizontal resolution : 3780 px/m
    0xC4, 0x0E, 0x00, 0x00, // Vertical resolution : 3780 px/m
    0x00, 0x00, 0x00, 0x00, // Colors in palette : 0 (256)
    0x00, 0x00, 0x00, 0x00, // Importants colors : 0
    // Bit masks
    0x00, 0xF8, 0x00, 0x00, // Red mask :   1111 1000  0000 0000
    0xE0, 0x07, 0x00, 0x00, // Green mask : 0000 0111  1110 0000
    0x1F, 0x00, 0x00, 0x00, // Blue mask :  0000 0000  0001 1111
];


const OV2640_JPEG_INIT: [[u8; 2]; 14] = [
    // Reset
    [0xff, 0x01],
    [0x12, 0x80],

    // Delay ~100 ms nécessaire après reset

    // Sélection banque DSP
    [0xff, 0x00],

    // Activer JPEG
    [0x2c, 0xff],
    [0x2e, 0xdf],

    // Format JPEG
    [0xff, 0x01],
    [0x12, 0x40], // COM7 = JPEG

    // Configuration DSP
    [0xff, 0x00],
    [0xe0, 0x14],
    [0xe1, 0x77],
    [0xe5, 0x1f],
    [0xd7, 0x03],
    [0xda, 0x10],
    [0xe0, 0x00],
];

const IMAGE_HEADER: [u8; IMAGE_HEADER_SIZE] = BMP_HEADER_GRAYSCALED;

macro_rules! i2c_read {
    ($i2c:ident, $reg:expr) => {{
        let mut value = [0u8];
        $i2c.write_read(SENSOR_ADDRESS, &[$reg], &mut value)
            .expect("I2C read");
        value[0]
    }};
}

macro_rules! i2c_write {
    ($i2c:ident, $reg:expr, $val:expr) => {
        $i2c.write(SENSOR_ADDRESS, &[$reg, $val])
            .expect("I2C write");
    };
}

macro_rules! spi_read {
    ($spi:ident, $cs:ident, $reg:expr) => {{
        $cs.set_low();
        $spi.write(&[$reg]).expect("SPI write");
        let value = $spi.transfer(&mut [0u8]).expect("SPI read")[0];
        $cs.set_high();
        value
    }};
}

macro_rules! spi_write {
    ($spi:ident, $cs:ident, $reg:expr, $val:expr) => {
        $cs.set_low();
        $spi.write(&[$reg | 0x80, $val]).expect("SPI write");
        $cs.set_high();
    };
}

macro_rules! clear_fifo_flag {
    ($spi:ident, $cs:ident) => {
        spi_write!($spi, $cs, ARDUCHIP_FIFO, 0x01)
    };
}

macro_rules! start_capture {
    ($spi:ident, $cs:ident) => {
        spi_write!($spi, $cs, ARDUCHIP_FIFO, 0x02)
    };
}

macro_rules! capture_done {
    ($spi:ident, $cs:ident) => {{ spi_read!($spi, $cs, ARDUCHIP_TRIG) & 0x08 != 0 }};
}

macro_rules! read_fifo_flag {
    ($spi:ident, $cs:ident) => {{
        ((spi_read!($spi, $cs, ARDUCHIP_FIFO_SIZE3) as u32) << 16
            | (spi_read!($spi, $cs, ARDUCHIP_FIFO_SIZE2) as u32) << 8
            | (spi_read!($spi, $cs, ARDUCHIP_FIFO_SIZE1) as u32))
            & 0x7fffff
    }};
}

macro_rules! burst_fifo_read {
    ($spi:ident, $cs:ident, $length:expr) => {{
        let mut v = [0u8; 153666];
        for i in 0..BMP_HEADER.len() {
            v[i] = BMP_HEADER[i];
        }
        $cs.set_low();
        $spi.write(&[ARDUCHIP_BURST_FIFO_READ]).expect("SPI write");
        let mut i = BMP_HEADER.len();
        for _ in 0..153600 / 2 {
            let vh = $spi.transfer(&mut [0u8]).expect("SPI read")[0];
            v[i] = $spi.transfer(&mut [0u8]).expect("SPI read")[0];
            v[i + 1] = vh;
            i += 2;
        }
        $cs.set_high();
        v
    }};
}

fn rgb565_to_gray(pixel: u16) -> u8 {
    let r = (pixel >> 11) & 0x1F;
    let g = (pixel >> 5) & 0x3F;
    let b = pixel & 0x1F;

    // Mise à l’échelle intégrée
    let y =
        77 * (r * 255 / 31) +
            150 * (g * 255 / 63) +
            29 * (b * 255 / 31);

    (y >> 8) as u8
}

pub fn rgb565_buffer_to_gray(
    src: &[u8],
    dst: &mut [u8],
) {
    let mut j = 0;

    for i in (0..src.len()).step_by(2) {
        let pixel = u16::from_be_bytes([src[i], src[i + 1]]);
        dst[j] = rgb565_to_gray(pixel);
        j += 1;
    }
}


#[global_allocator]
static ALLOCATOR: EmbeddedAllocator = EmbeddedAllocator::empty();

// the program entry point
#[cortex_m_rt::entry]
fn main() -> ! {
    {
        use core::mem::MaybeUninit;
        const ALLOCATION_BUFFER_SIZE: usize = 256 * 1024;
        static mut ALLOCATION_BUFFER: [MaybeUninit<u8>; ALLOCATION_BUFFER_SIZE] =
            [MaybeUninit::uninit(); ALLOCATION_BUFFER_SIZE];
        unsafe {
            ALLOCATOR.init(&raw mut ALLOCATION_BUFFER as usize, ALLOCATION_BUFFER_SIZE);
        }
    }

    let dp = stm32::Peripherals::take().unwrap();
    let cp = stm32::CorePeripherals::take().unwrap();

    // Power
    let pwrcfg = dp.PWR.constrain().freeze();

    // Clocks...
    let rcc = dp.RCC.constrain();
    let ccdr = rcc
        .sys_ck(200.MHz())
        .hclk(200.MHz())
        .pll1_r_ck(100.MHz()) // for TRACECK
        .pll1_q_ck(48.MHz())
        .freeze(pwrcfg, &dp.SYSCFG);

    // Get the delay provider.
    let mut delay = cp.SYST.delay(ccdr.clocks);

    // Initialise IO...
    let gpioa = dp.GPIOA.split(ccdr.peripheral.GPIOA);
    let gpiob = dp.GPIOB.split(ccdr.peripheral.GPIOB);
    let gpioc = dp.GPIOC.split(ccdr.peripheral.GPIOC);
    let gpiod = dp.GPIOD.split(ccdr.peripheral.GPIOD);
    let gpiog = dp.GPIOG.split(ccdr.peripheral.GPIOG);

    let rmii_ref_clk = gpioa.pa1.into_alternate();
    let rmii_mdio = gpioa.pa2.into_alternate();
    let rmii_mdc = gpioc.pc1.into_alternate();
    let rmii_crs_dv = gpioa.pa7.into_alternate();
    let rmii_rxd0 = gpioc.pc4.into_alternate();
    let rmii_rxd1 = gpioc.pc5.into_alternate();
    let rmii_tx_en = gpiog.pg11.into_alternate();
    let rmii_txd0 = gpiog.pg13.into_alternate();
    let rmii_txd1 = gpiob.pb13.into_alternate();

    let sck = gpioa.pa5.into_alternate();
    let miso = gpioa.pa6.into_alternate();
    let mosi = gpiob.pb5.into_alternate();
    let mut cs = gpiod.pd15.into_push_pull_output();
    cs.set_high();

    let scl = gpiob.pb6.into_alternate_open_drain();
    let sda = gpiob.pb7.into_alternate_open_drain();

    // Initialise ethernet...
    assert_eq!(ccdr.clocks.hclk().raw(), 200_000_000); // HCLK 200MHz
    assert_eq!(ccdr.clocks.pclk1().raw(), 100_000_000); // PCLK 100MHz
    assert_eq!(ccdr.clocks.pclk2().raw(), 100_000_000); // PCLK 100MHz
    assert_eq!(ccdr.clocks.pclk4().raw(), 100_000_000); // PCLK 100MHz

    let mac_addr = smoltcp::wire::EthernetAddress::from_bytes(&MAC_ADDRESS);
    let (mut eth_dma, _eth_mac) = unsafe {
        static mut DES_RING: ethernet::DesRing<4, 4> = ethernet::DesRing::new();

        ethernet::new(
            dp.ETHERNET_MAC,
            dp.ETHERNET_MTL,
            dp.ETHERNET_DMA,
            (
                rmii_ref_clk,
                rmii_mdio,
                rmii_mdc,
                rmii_crs_dv,
                rmii_rxd0,
                rmii_rxd1,
                rmii_tx_en,
                rmii_txd0,
                rmii_txd1,
            ),
            #[allow(static_mut_refs)]
            &mut DES_RING,
            mac_addr,
            ccdr.peripheral.ETH1MAC,
            &ccdr.clocks,
        )
    };

    let mut iface = Interface::new(
        Config::new(EthernetAddress::from_bytes(&MAC_ADDRESS).into()),
        &mut eth_dma,
        Instant::from_micros(cortex_m::peripheral::DWT::cycle_count() / 200),
    );
    iface.update_ip_addrs(|ip_addrs| {
        ip_addrs
            .push(IpCidr::new(IpAddress::v4(192, 168, 122, 100), 24))
            .unwrap();
    });

    let tcp_rx_buffer = tcp::SocketBuffer::new(vec![0; 65535 / 10]);
    let tcp_tx_buffer = tcp::SocketBuffer::new(vec![0; 3 * 65535]);
    let tcp_socket = tcp::Socket::new(tcp_rx_buffer, tcp_tx_buffer);

    let mut sockets = SocketSet::new(vec![]);
    let socket_handle = sockets.add(tcp_socket);

    let mut spi: spi::Spi<pac::SPI1, _, u8> = dp.SPI1.spi(
        (sck, miso, mosi),
        spi::MODE_0,
        3.MHz(),
        ccdr.peripheral.SPI1,
        &ccdr.clocks,
    );
    let mut i2c = dp
        .I2C1
        .i2c((scl, sda), 100.kHz(), ccdr.peripheral.I2C1, &ccdr.clocks);

    // ArduCAM reset
    spi_write!(spi, cs, 0x07, 0x80);
    delay.delay_ms(100_u16);
    spi_write!(spi, cs, 0x07, 0x00);
    delay.delay_ms(100_u16);

    // SPI Test
    spi_write!(spi, cs, ARDUCHIP_TEST1, 0x55);
    //defmt::println!("SPI_TEST = 0x{=u8:X}", spi_read!(spi, cs, ARDUCHIP_TEST1));

    // I2C Test
    i2c_write!(i2c, 0xFF, 0x01);
    //defmt::println!("VID = 0x{=u8:X}", i2c_read!(i2c, OV2640_CHIPID_HIGH));
    //defmt::println!("PID = 0x{=u8:X}", i2c_read!(i2c, OV2640_CHIPID_LOW));

    // Camera Init
    i2c_write!(i2c, 0xFF, 0x01);
    i2c_write!(i2c, 0x12, 0x80);
    delay.delay_ms(100_u16);

    // Camera config
    for [reg, val] in OV2640_QVGA {
        i2c_write!(i2c, reg, val);
    }
    delay.delay_ms(1000_u16);
    defmt::println!("BEGIN LOOP");
    let mut capture_requested = false;
    loop {

        let timestamp = Instant::from_micros(cortex_m::peripheral::DWT::cycle_count() / 200);
        iface.poll(timestamp, &mut eth_dma, &mut sockets);

        let socket = sockets.get_mut::<tcp::Socket>(socket_handle);

        if !socket.is_open() {
            defmt::println!("Socket OPEN");
            socket.listen(80).unwrap();
        }

        if socket.may_recv() {
            let data_received = socket
                .recv(|buffer| {
                    if !buffer.is_empty() {
                        defmt::println!("{=str}", str::from_utf8(buffer).unwrap());
                        (buffer.len(), true)
                    } else {
                        (0, false)
                    }
                })
                .unwrap();
            if socket.can_send() && data_received {
                // Take photo
                if !capture_requested {
                    defmt::println!("capture");
                    capture_requested = true;
                    clear_fifo_flag!(spi, cs);
                    start_capture!(spi, cs);
                }

                while !capture_done!(spi, cs) {
                    delay.delay_ms(10_u16);
                }
                delay.delay_ms(50_u16);

                let length = read_fifo_flag!(spi, cs);

                if length >= 153600 {
                    capture_requested = false;
                    defmt::println!("RESPONSE");
                    socket
                        .send_slice("HTTP/1.1 200\nContent-Type: image/bmp\nContent-Length: 77878\n\n".as_bytes())
                        .unwrap();
                    socket.send_slice(&IMAGE_HEADER).unwrap();
                    let mut finished = false;
                    cs.set_low();
                    spi.write(&[ARDUCHIP_BURST_FIFO_READ]).expect("SPI write");
                    let mut send_length = 0;
                    while send_length < 76800 {
                        send_length += socket
                            .send(|buffer| {
                                let mut written_length = 0;
                                let mut raw = [0u8; 153600];
                                spi.transfer(&mut raw).expect("SPI burst read");
                                for i in 0..buffer.len() {
                                    if send_length + written_length == 76800 {
                                        break;
                                    }
                                    let byte1 = raw[2*i] as u16;
                                    let byte2 = raw[2*i + 1] as u16;
                                    let color = (byte1 << 8) | byte2;
                                    buffer[i] = rgb565_to_gray(color);
                                    written_length += 1;
                                    if written_length < 1 {
                                        defmt::println!("{=u8:02X}", buffer[i]);
                                    }
                                }
                                (written_length, written_length)
                            })
                            .unwrap();
                    }
                    //defmt::println!("Socket CLOSE");
                    //socket.close();
                }
            }
        } else if socket.may_send() {
            //defmt::println!("Socket CLOSE");
            //socket.close();
        }
    }
}
