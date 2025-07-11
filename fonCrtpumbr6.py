data = [
    0x00, 0x06, 0x00, 0x80, 0x07, 0x00, 0xC0, 0x07, 0x00, 0xE0, 0x07, 0x00, 0xF0, 0x03, 0x00, 0xF8, 0x03, 0x00, 0xF8,
    0x00, 0x00, 0xF8, 0x00, 0x00, 0xFC, 0x00, 0x00, 0x7C, 0x00, 0x00, 0x7C, 0xE0, 0x03, 0x7C, 0xF0, 0x07, 0xFC, 0xF0,
    0x0F, 0xFC, 0xF8, 0x1F, 0xFC, 0xF8, 0x1F, 0xFC, 0xF8, 0x3F, 0xF8, 0x81, 0x3F, 0xF8, 0x01, 0x3F, 0xF8, 0x01, 0x3E,
    0xF8, 0x03, 0x3E, 0xF0, 0x03, 0x3E, 0xF0, 0x07, 0x1F, 0xE0, 0x87, 0x1F, 0xE0, 0xCF, 0x0F, 0xE0, 0xFF, 0x0F, 0xC0,
    0xFF, 0x07, 0xC0, 0xFF, 0x03, 0x80, 0xFF, 0x01, 0x00, 0x7F, 0x00, 0x00, 0x08, 0x00 
]

def gprint_bitmap(data):
    print("  // Binary representation in 3-byte chunks")
    for i in range(0, len(data), 3):
        triple = data[i:i + 3]
        if len(triple) < 3:
            break
        val = triple[0] | (triple[1] << 8) | (triple[2] << 16)
        byte1 = (val >> 0) & 0xFF
        byte2 = (val >> 8) & 0xFF
        byte3 = (val >> 16) & 0xFF
        print(f"  0b{byte3:08b}, 0b{byte2:08b}, 0b{byte1:08b},")
        binary_str = f"{val:024b}"

def print_bitmap(data):
    print("  //")
    for i in range(0, len(data), 3):
        triple = data[i:i + 3]
        if len(triple) < 3:
            break
        val = triple[0] | (triple[1] << 8) | (triple[2] << 16)
        byte1 = (val >> 0) & 0xFF
        byte2 = (val >> 8) & 0xFF
        byte3 = (val >> 16) & 0xFF
        bin_str = f"{val:024b}"
        byte1_str = bin_str[0:8]
        byte2_str = bin_str[8:16]
        byte3_str = bin_str[16:24]
        print(f" 0b{byte1_str}, 0b{byte2_str}, 0b{byte3_str},")

all_numbers_data = [
    [
   0x00,0x00,0x00,0x00,0x1C,0x00,0x00,0x0E,0x00,0x80,0xFF,0x01,0xC0,0x8D,0x03,0xE0,0x06,0x07,0xE0,0xFB,0x0E,0x70,0xFF,0x1D,0x78,0xCF,0x3D,0x3C,0xC3,0x3D,0x3C,0xE6,0x78,0x3E,0x7C,0x78,0x1E,0x00,0xF8,0x1E,0x00,0xF0,0x1F,0x00,0xF0,0x1F,0x00,0xF0,0x1F,0x00,0xF0,0x1F,0x00,0xF0,0x1F,0x00,0xF0,0x1F,0x00,0x70,0x1F,0x00,0x70,0x1F,0x00,0x70,0x1E,0x00,0x38,0x3E,0x00,0x38,0x3E,0x00,0x38,0x3C,0x00,0x18,0x3C,0x00,0x1C,0x78,0x00,0x0C,0xF0,0x00,0x06,0xE0,0x00,0x03,0xC0,0x81,0x03,0x80,0xC7,0x00,0x00,0x7E,0x00,],# // Code for char num 48
   [0x00,0x30,0x00,0x78,0x3C,0x00,0x68,0x3E,0x00,0x88,0x3F,0x00,0xF8,0x3E,0x00,0x00,0x3E,0x00,0x00,0x3E,0x00,0x00,0x3E,0x00,0x00,0x3E,0x00,0x00,0x3C,0x00,0x00,0x3C,0x00,0x00,0x3C,0x00,0x00,0x1C,0x00,0x00,0x1C,0x00,0x00,0x1C,0x00,0x00,0x1C,0x00,0x00,0x1C,0x00,0x00,0x1C,0x00,0x00,0x1C,0x00,0x00,0x1C,0x00,0x00,0x1C,0x00,0x00,0x1C,0x00,0x00,0x1C,0x00,0x00,0x1C,0x00,0x00,0x18,0x00,0x00,0x18,0x00,0x00,0x18,0x00,0x00,0x18,0x00,0x00,0x18,0x00,0x00,0xD8,0x03,0x00,0x7E,0x00,0xC0,0x03,0x00,0x60,0x00,0x00, ],#// Code for char num 49
   [0x00,0x00,0x00,0x00,0xFE,0x00,0x00,0xC3,0x03,0xC0,0x81,0x07,0xE0,0x00,0x0F,0x60,0x00,0x0F,0x70,0x1E,0x1E,0x70,0x3F,0x1E,0x70,0x3F,0x1E,0xF0,0x38,0x1E,0xE0,0x1D,0x1E,0xC0,0x07,0x0F,0x00,0x00,0x0F,0x00,0x80,0x07,0x00,0x80,0x03,0x00,0xC0,0x01,0x00,0xE0,0x00,0x00,0x70,0x00,0x00,0x18,0x00,0x00,0x0C,0x00,0x00,0x07,0x00,0x80,0x01,0x00,0xC0,0x00,0x00,0x60,0x00,0x00,0x30,0x80,0x0F,0x18,0xC0,0x18,0x18,0xC0,0x36,0x0C,0xC0,0x36,0xFC,0x83,0x33,0xFC,0x1F,0x30,0xFC,0x3F,0x18,0x0C,0xFC,0x0C,0x00,0xC0,0x07, ],#// Code for char num 50
   [0x00,0xFF,0x00,0xE0,0xE1,0x03,0xF0,0xC0,0x07,0x78,0x80,0x0F,0x38,0x0F,0x0F,0xB8,0x1D,0x1F,0xB0,0x1D,0x1F,0x70,0x0E,0x1F,0xE0,0x07,0x1F,0x00,0x00,0x0F,0x00,0x80,0x0F,0x00,0x80,0x07,0x00,0xC0,0x03,0x00,0xE0,0x01,0x00,0x70,0x00,0x60,0x38,0x00,0xE0,0x0F,0x00,0x00,0x78,0x00,0x00,0xE0,0x01,0x00,0xC0,0x03,0x00,0xC0,0x07,0x00,0x80,0x07,0x00,0x80,0x0F,0x00,0x80,0x0F,0xE0,0x87,0x0F,0x30,0x8F,0x0F,0x18,0x8E,0x0F,0xD8,0x8E,0x07,0xD8,0xC7,0x03,0xB8,0xC3,0x03,0x30,0xE0,0x01,0xE0,0x70,0x00,0x80,0x1F,0x00, ],#// Code for char num 51
   [0xF0,0x07,0x07,0x3C,0x1E,0x1F,0x0E,0x1C,0x1F,0x07,0x3C,0x0F,0xF7,0x3C,0x0F,0xE7,0xBC,0x0F,0xEE,0xBE,0x0F,0x3C,0x9E,0x07,0x00,0x9F,0x07,0x00,0x8F,0x07,0x80,0x87,0x07,0xC0,0x83,0x07,0xE0,0x81,0x07,0xE0,0x80,0x03,0x70,0x80,0x03,0x38,0x80,0x03,0x1C,0x80,0x03,0x1E,0x80,0xE3,0x0E,0xC0,0xFF,0xFF,0xFF,0xFF,0x00,0x80,0x03,0x00,0x80,0x03,0x00,0x80,0x03,0x00,0x80,0x03,0x00,0x80,0x03,0x00,0x80,0x03,0x00,0x80,0x03,0x00,0x80,0x03,0x00,0x80,0x03,0x00,0x80,0x03,0x00,0xC0,0x7F,0x00,0x7E,0x00,0x80,0x03,0x00, ],#// Code for char num 52
   [0x00,0x00,0x00,0xFE,0xFF,0x0F,0xF6,0xFF,0x1F,0x86,0x3F,0x1C,0x0E,0x00,0x1C,0x0C,0x80,0x0C,0x0C,0xC0,0x06,0x1C,0xC0,0x03,0x1C,0x00,0x00,0x1C,0x00,0x00,0x3C,0x00,0x00,0x3C,0x00,0x00,0xFC,0x3F,0x00,0x1C,0xF0,0x01,0x0C,0xC0,0x07,0x00,0x80,0x0F,0x00,0x00,0x1F,0x00,0x00,0x3E,0x00,0x00,0x3E,0x00,0x00,0x7C,0xC0,0x07,0x7C,0xE0,0x0E,0x7C,0x78,0x0C,0x7C,0x38,0x0C,0x7C,0xFC,0x07,0x3C,0xFC,0x03,0x3C,0x3C,0x00,0x1E,0x3C,0x00,0x1E,0x78,0x00,0x0F,0x78,0x80,0x07,0xF0,0x80,0x03,0xC0,0xE1,0x00,0x00,0x3F,0x00, ],#// Code for char num 53
   [0x00,0xE0,0x07,0x00,0xF8,0x0F,0x00,0x1E,0x1E,0x00,0x07,0x38,0x80,0x03,0x38,0xC0,0x01,0x38,0xC0,0x01,0x3C,0xE0,0x00,0x1E,0xF0,0x00,0x00,0x70,0x00,0x00,0x78,0x00,0x00,0x78,0x00,0x00,0x7C,0x00,0x00,0x3C,0x00,0x00,0x3C,0x00,0x00,0x3E,0xFC,0x01,0x3E,0xC6,0x07,0x3E,0x03,0x0F,0xBE,0x03,0x1E,0xBE,0x3B,0x3E,0xBE,0x3B,0x7C,0x3E,0x3F,0x7C,0x3C,0x0F,0x7C,0x7C,0x00,0x7C,0x7C,0x00,0x7C,0x78,0x00,0x3C,0xF8,0x00,0x3C,0xF0,0x00,0x1E,0xF0,0x01,0x0E,0xE0,0x03,0x06,0xC0,0x07,0x03,0x00,0x8F,0x01,0x00,0xFC,0x00, ],#// Code for char num 54
   [0x00,0x00,0x00,0xF8,0x00,0x00,0xF8,0x0F,0x18,0xF8,0xFF,0x0F,0xF8,0xFF,0x0F,0x18,0xFF,0x0F,0x18,0x00,0x06,0x18,0x00,0x06,0x18,0x00,0x03,0x18,0x80,0x03,0x00,0x80,0x01,0x00,0xC0,0x01,0x00,0xC0,0x00,0x00,0xE0,0x00,0x00,0x70,0x00,0x00,0x70,0x00,0x00,0x38,0x00,0x00,0x3C,0x00,0x00,0x1C,0x00,0x00,0x1E,0x00,0x00,0x0E,0x00,0x00,0x0F,0x00,0x00,0x07,0x00,0x80,0x07,0x00,0x80,0xF7,0x00,0x80,0xF3,0x00,0xC0,0xDB,0x01,0xC0,0x9B,0x01,0xC0,0xBB,0x01,0xC0,0x83,0x01,0x80,0xC7,0x00,0x00,0xEF,0x00,0x00,0x3E,0x00, ],#// Code for char num 55
   [0x00,0xFE,0x00,0xC0,0xC3,0x03,0xE0,0x00,0x07,0x70,0x00,0x0E,0x78,0xF0,0x1C,0x3C,0xF8,0x1D,0x3C,0xCC,0x1D,0x3C,0x0C,0x1C,0x3C,0x1C,0x0F,0x3C,0xF8,0x07,0x3C,0xF0,0x03,0x78,0xC0,0x01,0x70,0xE0,0x00,0xE0,0x60,0x00,0xC0,0x31,0x00,0x80,0x1B,0x00,0x00,0x0E,0x00,0x00,0x3F,0x00,0x80,0xF3,0x00,0xC0,0xE1,0x03,0xE0,0xC0,0x07,0xF0,0x80,0x0F,0x70,0x00,0x0F,0x78,0x00,0x1E,0x78,0x00,0x1E,0x78,0x00,0x1E,0x78,0x00,0x1E,0x78,0x00,0x1E,0xF0,0x00,0x0E,0xF0,0x00,0x07,0xE0,0x81,0x07,0x80,0xC7,0x01,0x00,0x7E,0x00, ],#// Code for char num 56
   [0x00,0x00,0x00,0x00,0x3F,0x00,0xC0,0xE3,0x00,0xE0,0xC1,0x03,0xF0,0x80,0x07,0x78,0x00,0x0F,0x78,0x00,0x0F,0x7C,0x00,0x1E,0x3C,0x00,0x1E,0x3E,0x00,0x3E,0x3E,0x00,0x3C,0x3E,0x00,0x7C,0x3E,0xF0,0x7C,0x3E,0xF8,0x7C,0x3E,0xCC,0x7D,0x7C,0x8C,0x7D,0x7C,0x8C,0x7D,0xF8,0x9C,0x7D,0xF0,0xC1,0x7C,0xC0,0x63,0x7C,0x00,0x3F,0x3C,0x00,0x00,0x3E,0x00,0x00,0x3E,0x00,0x00,0x1E,0x00,0x00,0x1E,0x00,0x00,0x0F,0x00,0x00,0x0F,0xE0,0x80,0x07,0x70,0x80,0x03,0x70,0xC0,0x03,0xF0,0xE0,0x01,0xE0,0x71,0x00,0x80,0x1F,0x00 ],#// Code for char num 57
]

import time
partenza = 24  # Modifica questo valore per cambiare il punto di partenza
salto = 99    # Incremento fisso di 99 per ogni numero

# Genera le stringhe nel formato richiesto
print("\n// Formatted strings:")
for i, number_data in enumerate(all_numbers_data):
    rows = len(number_data) // 3  # Ogni riga è 3 byte
    print(f"  {{ 24, 33, {partenza + i*salto} }}, // '{i+1}'")
print("\n")

# Genera la rappresentazione binaria
for i, number_data in enumerate(all_numbers_data):
    print(f"// Character '{i}'")
    print_bitmap(number_data)
    print("\n")  # Spazio tra i caratteri
    time.sleep(0.1)  # Piccolo ritardo