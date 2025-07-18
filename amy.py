datra = [
    0x00, 0x00, 0x00, 0x0C, 0x00, 0x0C, 0x00, 0x0E, 0x00, 0x0E, 0x00, 0x0E, 0x00, 0x0F, 0x00, 0x0F, 0x00, 0x0F, 0x80,
    0x0D, 0x80, 0x0D, 0xC0, 0x0C, 0xC0, 0x0C, 0xF8, 0x0C, 0x6C, 0x0C, 0xEC, 0x0C, 0xBC, 0x0D, 0x3C, 0x0F, 0x18, 0x0F,
    0x18, 0x0E, 0x3C, 0x0E, 0x6C, 0x0C, 0xC6, 0x0C, 0x83, 0x0D, 0x01, 0x1F,  # // Code for char num 65
    0x00, 0x38, 0x00, 0x6E, 0x80, 0xC3, 0xC0, 0xC6, 0xC0, 0x86, 0x60, 0x86, 0x30, 0x86, 0x30, 0xC6, 0x30, 0xC6, 0x18,
    0x66, 0x18, 0x76, 0x18, 0x1E, 0x0C, 0x1E, 0x0C, 0x33, 0x0C, 0x33, 0x0C, 0x63, 0x0C, 0x63, 0x8C, 0x61, 0x86, 0x61,
    0xC6, 0x60, 0xC6, 0x60, 0xE6, 0x60, 0xE6, 0x30, 0xB6, 0x39, 0x1C, 0x0F,  # // Code for char num 66
    0x00, 0x0F, 0xC0, 0x19, 0x60, 0x30, 0x60, 0x30, 0x30, 0x30, 0x30, 0x30, 0x18, 0x30, 0x18, 0x33, 0x0C, 0x33, 0x0C,
    0x1F, 0x0C, 0x00, 0x0C, 0x00, 0x0C, 0x00, 0x06, 0x00, 0x06, 0x00, 0x06, 0x00, 0x06, 0x00, 0x06, 0x00, 0x06, 0x00,
    0x06, 0x0C, 0x0C, 0x0C, 0x0C, 0x06, 0x0C, 0x06, 0x98, 0x03, 0xF0, 0x00  # // Code for char num 67
]


def reverse_bits(byte):
    """Inverte l'ordine dei bit in un byte (es: 0b00001111 -> 0b11110000)"""
    return int(f"{byte:08b}"[::-1], 2)


print("{")
line_count = 0
for i in range(0, len(datra), 2):
    doppio = datra[i:i + 2]
    if len(doppio) < 2:
        break
    # Little-endian: primo byte è LSB, secondo è MSB
    byte2 = reverse_bits(doppio[0])  # Inverti i bit
    byte1 = reverse_bits(doppio[1])  # Inverti i bit
    # Stampa i byte con bit invertiti
    print(f"  0b{byte2:08b}, 0b{byte1:08b},")
    bin1 = ''.join(['*' if bit == '1' else ' ' for bit in f"{byte1:08b}"])
    bin2 = ''.join(['*' if bit == '1' else ' ' for bit in f"{byte2:08b}"])
    #print(f"  0b{byte2:08b}, 0b{byte1:08b},  // {bin2} {bin1}")
    line_count += 1
    if line_count % 25 == 0:  # Ogni 25 righe
        print()  # Linea vuota

print("}")