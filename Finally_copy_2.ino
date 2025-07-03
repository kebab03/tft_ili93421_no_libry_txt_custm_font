#include <Arduino.h>
#include <Adafruit_GFX.h>
#include <Fonts/FreeSans9pt7b.h>

// --- Struttura per descrivere un glifo ---
typedef struct {
  uint8_t width;
  uint8_t height;
  uint16_t offset;  // indice nel bitmap
} GlyphInfo;

// --- Bitmap font 8x12 (solo 'A' in questo esempio) ---
const uint8_t MyFont8x12_Bitmap[] PROGMEM = {
  // 'A', 8x12 bitmap
  0b00011000, //    **
  0b00100100, //   *  *
  0b00100100, //   *  *
  0b01000010, //  *    *
  0b01000010, //  *    *
  0b01111110, //  ******
  0b10000001, // *      *
  0b10000001, // *      *
  0b10000001, // *      *
  0b10000001, // *      *
  0b10000001, // *      *
  0b00000000, //     

  // 'B', 8x12 bitmap
  0b11111100,
  0b10000010,
  0b10000010,
  0b10000010,
  0b11111100,
  0b10000010,
  0b10000001,
  0b10000001,
  0b10000001,
  0b10000010,
  0b11111100,
  0b00000000,

  // '9', 24x33 pixel
  0b00000000, 0b00000000, 0b00000000, // Riga 0
  0b00000000, 0b11111100, 0b00000000, // Riga 1
  0b00000011, 0b11000111, 0b00000000, // Riga 2
  0b00001111, 0b00000011, 0b11000000, // Riga 3
  0b00011110, 0b00000001, 0b11100000, // Riga 4
  0b00111100, 0b00000000, 0b11110000, // Riga 5
  0b00111100, 0b00000000, 0b11110000, // Riga 6
  0b01111100, 0b00000000, 0b01111000, // Riga 7
  0b01111000, 0b00000000, 0b01111000, // Riga 8
  0b11111100, 0b00000000, 0b01111100, // Riga 9
  0b11111100, 0b00000000, 0b00111100, // Riga 10
  0b11111100, 0b00000000, 0b00111110, // Riga 11
  0b11111100, 0b00111100, 0b00111110, // Riga 12
  0b11111100, 0b01111100, 0b00111110, // Riga 13
  0b11111100, 0b11001110, 0b00111110, // Riga 14
  0b01111110, 0b11000110, 0b00111110, // Riga 15
  0b01111110, 0b11000110, 0b00111110, // Riga 16
  0b00111111, 0b11001110, 0b00111110, // Riga 17
  0b00011111, 0b10000110, 0b00111110, // Riga 18
  0b00000111, 0b00001100, 0b00111110, // Riga 19
  0b00000001, 0b11111000, 0b00111100, // Riga 20
  0b00000000, 0b00000000, 0b01111100, // Riga 21
  0b00000000, 0b00000000, 0b01111100, // Riga 22
  0b00000000, 0b00000000, 0b01111000, // Riga 23
  0b00000000, 0b00000000, 0b01111000, // Riga 24
  0b00000000, 0b00000000, 0b11110000, // Riga 25
  0b00000000, 0b00000000, 0b11110000, // Riga 26
  0b00000111, 0b00000001, 0b11100000, // Riga 27
  0b00001110, 0b00000001, 0b11000000, // Riga 28
  0b00001110, 0b00000011, 0b11000000, // Riga 29
  0b00001111, 0b00000111, 0b10000000, // Riga 30
  0b00000111, 0b10001110, 0b00000000, // Riga 31
  0b00000001, 0b11111000, 0b00000000  // Riga 32
};

// --- Glifi associati ---
const GlyphInfo MyFont8x12_Glyphs[] PROGMEM = {
  { 8, 12, 0 },   // 'A' - offset 0, 12 righe
  { 8, 12, 12 },  // 'B' - offset 12, 12 righe
  { 24, 33, 24 }  // '9' - offset 24, 33 righe (3 byte per riga)
};

// --- Definizioni colori ---
#define BLACK    0x0000
#define BLUE     0x001F
#define RED      0xF800
#define GREEN    0x07E0
#define CYAN     0x07FF
#define MAGENTA  0xF81F
#define YELLOW   0xFFE0 
#define WHITE    0xFFFF
#define ORANGE   0xFD20
#define PINK     0xF81F

// --- Dimensioni Display ---
#define TFT_WIDTH  240
#define TFT_HEIGHT 320
#define TFT_ROTATION 2

// --- Pin di controllo ---
#define LCD_RD   A0
#define LCD_WR   A1
#define LCD_RS   A2
#define LCD_CS   A3
#define LCD_RST  A4

// --- Pin dei dati (PORTD: D2-D7, PORTB: D8-D9) ---
#define DATA_PORT PORTD
#define DATA_DDR  DDRD
#define DATA_MASK 0xFC  // D2-D7

#define DATA_PORT_H PORTB
#define DATA_DDR_H  DDRB
#define DATA_MASK_H 0x03 // D8-D9

void LCD_write(uint8_t d) {
  digitalWrite(LCD_WR, LOW);
  
  PORTD &= ~0xFC;
  PORTB &= ~0x03;
  
  PORTD |= (d & 0xFC);
  PORTB |= (d & 0x03);
  
  digitalWrite(LCD_WR, HIGH);
  delayMicroseconds(1);
}

void LCD_command(uint8_t cmd) {
  digitalWrite(LCD_RS, LOW);
  LCD_write(cmd);
}

void LCD_data(uint8_t data) {
  digitalWrite(LCD_RS, HIGH);
  LCD_write(data);
}

void LCD_data16(uint16_t data) {
  LCD_data(data >> 8);
  LCD_data(data & 0xFF);
}

void setRotation(uint8_t rotation) {
  LCD_command(0x36); // Memory Access Control
  switch (rotation) {
    case 0: LCD_data(0x48); break; // Portrait
    case 1: LCD_data(0x28); break; // Landscape
    case 2: LCD_data(0x88); break; // Inverted Portrait
    case 3: LCD_data(0xE8); break; // Inverted Landscape
  }
}

void setWindow(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2) {
  LCD_command(0x2A); // Col address set
  LCD_data16(x1);
  LCD_data16(x2);

  LCD_command(0x2B); // Row address set
  LCD_data16(y1);
  LCD_data16(y2);

  LCD_command(0x2C); // Memory write
}

void LCD_Init() {
  pinMode(LCD_RD, OUTPUT);
  pinMode(LCD_WR, OUTPUT);
  pinMode(LCD_RS, OUTPUT);
  pinMode(LCD_CS, OUTPUT);
  pinMode(LCD_RST, OUTPUT);

  DATA_DDR |= DATA_MASK;
  DATA_DDR_H |= DATA_MASK_H;

  digitalWrite(LCD_RST, HIGH);
  delay(5);
  digitalWrite(LCD_RST, LOW);
  delay(15);
  digitalWrite(LCD_RST, HIGH);
  delay(15);

  digitalWrite(LCD_CS, HIGH);
  digitalWrite(LCD_WR, HIGH);
  digitalWrite(LCD_RD, HIGH);
  digitalWrite(LCD_CS, LOW);

  // Inizializzazione ILI9341
  LCD_command(0xCF); LCD_data(0x00); LCD_data(0x83); LCD_data(0x30);
  LCD_command(0xED); LCD_data(0x64); LCD_data(0x03); LCD_data(0x12); LCD_data(0x81);
  LCD_command(0xE8); LCD_data(0x85); LCD_data(0x01); LCD_data(0x78);
  LCD_command(0xCB); LCD_data(0x39); LCD_data(0x2C); LCD_data(0x00); LCD_data(0x34); LCD_data(0x02);
  LCD_command(0xF7); LCD_data(0x20);
  LCD_command(0xEA); LCD_data(0x00); LCD_data(0x00);
  LCD_command(0xC0); LCD_data(0x26);
  LCD_command(0xC1); LCD_data(0x11);
  LCD_command(0xC5); LCD_data(0x35); LCD_data(0x3E);
  LCD_command(0xC7); LCD_data(0xBE);
  LCD_command(0x36); setRotation(TFT_ROTATION);
  LCD_command(0x3A); LCD_data(0x55); // 16-bit color
  LCD_command(0xB1); LCD_data(0x00); LCD_data(0x1B);
  LCD_command(0xF2); LCD_data(0x08);
  LCD_command(0x26); LCD_data(0x01);
  LCD_command(0xE0); LCD_data(0x1F); LCD_data(0x1A); LCD_data(0x18); LCD_data(0x0A);
  LCD_data(0x0F); LCD_data(0x06); LCD_data(0x45); LCD_data(0x87);
  LCD_data(0x32); LCD_data(0x0A); LCD_data(0x07); LCD_data(0x02);
  LCD_data(0x07); LCD_data(0x05); LCD_data(0x00);
  LCD_command(0xE1); LCD_data(0x00); LCD_data(0x25); LCD_data(0x27); LCD_data(0x05);
  LCD_data(0x10); LCD_data(0x09); LCD_data(0x3A); LCD_data(0x78);
  LCD_data(0x4D); LCD_data(0x05); LCD_data(0x18); LCD_data(0x0D);
  LCD_data(0x38); LCD_data(0x3A); LCD_data(0x1F);
  LCD_command(0x11); delay(120);
  LCD_command(0x29);
}

void drawPixelLowLevel(uint16_t x, uint16_t y, uint16_t color) {
  if (x >= TFT_WIDTH || y >= TFT_HEIGHT) return;
  //Serial.print("x"+x);
  setWindow(x, y, x, y);
  LCD_data16(color);
}

void fillScreen(uint16_t color) {
  setWindow(0, 0, TFT_WIDTH - 1, TFT_HEIGHT - 1);
  for (uint32_t i = 0; i < (uint32_t)TFT_WIDTH * TFT_HEIGHT; i++) {
    LCD_data16(color);
  }
}

// --- CLASSE PERSONALIZZATA per Adafruit_GFX ---
class MyTFT : public Adafruit_GFX {
public:
  MyTFT() : Adafruit_GFX(TFT_WIDTH, TFT_HEIGHT) {}

  void drawPixel(int16_t x, int16_t y, uint16_t color) override {
    drawPixelLowLevel(x, y, color);
  }

  // Disegna un singolo carattere
  
  void drawCharLowLevel(int16_t x, int16_t y, char c, uint16_t color) {
  if (c == ' ') return;
  
  uint8_t charIndex;
  
  if (c == 'A') {
    charIndex = 0;
  } else if (c == 'B') {
    charIndex = 1;
  } else if (c == '9') {
    charIndex = 2;
  } else {
    return;
  }

  GlyphInfo g;
  memcpy_P(&g, &MyFont8x12_Glyphs[charIndex], sizeof(GlyphInfo));

  for (uint8_t row = 0; row < g.height; row++) {
    uint32_t bits = 0;
    
    if (g.width <= 8) {
      // Caratteri 8-bit (A, B)
      bits = pgm_read_byte(&MyFont8x12_Bitmap[g.offset + row]);
      for (uint8_t col = 0; col < g.width; col++) {
        if (bits & (0x80 >> col)) {  // Controllo per 8-bit
          drawPixelLowLevel(x + col, y + row, color);
        }
      }
    } else {
      // Caratteri 24-bit (9)
      bits = (uint32_t)pgm_read_byte(&MyFont8x12_Bitmap[g.offset + row*3]) << 16;
      bits |= (uint32_t)pgm_read_byte(&MyFont8x12_Bitmap[g.offset + row*3 + 1]) << 8;
      bits |= pgm_read_byte(&MyFont8x12_Bitmap[g.offset + row*3 + 2]);
      for (uint8_t col = 0; col < g.width; col++) {
        if (bits & (0x800000 >> col)) {  // Controllo per 24-bit
          drawPixelLowLevel(x + col, y + row, color);
        }
      }
    }
  }
}
  void OLDdrawCharLowLevel(int16_t x, int16_t y, char c, uint16_t color) {
    if (c == ' ') return;
    
    uint8_t charIndex;

    //Serial.print("line 246 in drawCharLowLevelDrawing '");
    //Serial.println(c);
    // Serial.print("' (");
    // Serial.print(g.width);
    // Serial.print("x");
    // Serial.print(g.height);
    // Serial.print(") at ");
    // Serial.print(x);
    // Serial.print(",");
    // Serial.print(y);
    // Serial.print(" offset ");
    // Serial.println(g.offset);

    
    if (c == 'A') {
      charIndex = 0;
    } else if (c == 'B') {
      charIndex = 1;
    } else if (c == '9') {
      charIndex = 2;
    } else {
      return; // Carattere non supportato
    }
// Serial.print("270 charIndex:");
// Serial.println(charIndex);
    GlyphInfo g;
    memcpy_P(&g, &MyFont8x12_Glyphs[charIndex], sizeof(GlyphInfo));
    for (uint8_t row = 0; row < g.height; row++) {
//       Serial.print("274 g.height: ");
// Serial.print(g.height);
// Serial.println(" height");


      uint32_t bits = 0;
//       Serial.print("275 g.width: ");
// Serial.print(g.width);
// Serial.println(" righe");

      if (g.width <= 8) { 
        bits = (uint32_t)pgm_read_byte(&MyFont8x12_Bitmap[g.offset + row]);
      //Serial.print("286 g.height =8:: ");
Serial.print(bits,BIN);
Serial.println(" 289 bits");
      } else {
      //  Serial.print("g.height >8: %d righe\n", g.height);
// Serial.print("291 g.height >8:: ");
// Serial.print(g.height);
// Serial.println(" righe");

// Serial.print("Sono in Row: ");
// Serial.println(row);

        bits = (uint32_t)pgm_read_byte(&MyFont8x12_Bitmap[g.offset + row*3]) << 16;
        bits |= (uint32_t)pgm_read_byte(&MyFont8x12_Bitmap[g.offset + row*3 + 1]) << 8;
        bits |= pgm_read_byte(&MyFont8x12_Bitmap[g.offset + row*3 + 2]);
      }
      
          Serial.print("303 bits : ");
Serial.println(bits,BIN);
      for (uint8_t col = 0; col < g.width; col++) {
        if (bits & (0x800000 >> col)) { 
//           Serial.print("302 g.width: ");
// Serial.print(g.width);
// Serial.println(" righe");

// Serial.print("Sono in Row: ");
// Serial.println(row);

          drawPixelLowLevel(x + col, y + row, color);
        }
      //   else if  (bits & (0x80 >> col)) {
      //   drawPixelLowLevel(x + col, y + row, color);
      // }
      }
    }
  }

  // Stampa una stringa
  // void printLowLevel(int16_t x, int16_t y, const char* str, uint16_t color) {
  //   int16_t xo = x;
  //   Serial.print(*str);
  //   while (*str) {
  //     if (*str == '\n') {
  //       y += 33; // Altezza del carattere più grande
  //       x = xo;
  //     } else {
  //       drawCharLowLevel(x, y, *str, color);
        
  //       // Avanza il cursore in base alla larghezza del carattere
  //       if (*str == '9') {
  //   Serial.print("Drawing '9' at (");
  //   Serial.print(x);
  //   Serial.print(",");
  //   Serial.print(y);
  //   Serial.print("), size ");
  //   Serial.print(g.width);
  //   Serial.print("x");
  //   Serial.print(g.height);
  //   Serial.println();

  //         x += 24;
  //       } else {
  //         Serial.print("A o B")
  //         x += 8;
  //       }
  //     }
  //     str++;
  //   }
  // }
//   void printLowLevel(int16_t x, int16_t y, const char* str, uint16_t color) {
//   int16_t xo = x;
//   while (*str) {
//     if (*str == '\n') {
//             Serial.print('y +12');
//       y += 12;
//       x = xo;
//     } else {
//       drawCharLowLevel(x, y, *str, color);
//       // Avanza il cursore in base alla larghezza del carattere
//       switch(*str) {
//         case '9': x += 24; break;
//         case 'A':
//         case 'B': x += 8; break;
//         default: x += 8; // spazio o caratteri non supportati
//       }
//     }
//     str++;
//   }
// }
void printLowLevel(int16_t x, int16_t y, const char* str, uint16_t color) {
  int16_t xo = x;
  while (*str) {
    if (*str == '\n') {
      y += 33; // Altezza del carattere più grande ('9')
      x = xo;
    } else {
      drawCharLowLevel(x, y, *str, color);
      // Avanza il cursore in base alla larghezza del carattere
      if (*str == '9') {
        x += 24;
      } else {
        x += 8;
      }
    }
    str++;
  }
}

};

MyTFT tft;

void testAllChars() {
  fillScreen(BLACK);
  tft.printLowLevel(10, 30, "A", RED);
  delay(1000);
  tft.printLowLevel(30, 30, "B", GREEN);
  delay(1000);
  tft.printLowLevel(60, 30, "9", BLUE);
  delay(1000);
  tft.printLowLevel(70, 80, "A B 9", WHITE);
}

void setup() {
  Serial.begin(9600);
  LCD_Init();
  delay(100);
  
  //testAllChars();

  fillScreen(BLACK);
    //tft.printLowLevel(10, 30, "A", RED);
  delay(1000);
  tft.printLowLevel(30, 30, "B", GREEN);
  delay(1000);
  tft.printLowLevel(60, 120, "9", BLUE);
}

void loop() {
  // Nulla da fare qui
}