#include <Arduino.h>
#include <Adafruit_GFX.h>
#include <Fonts/FreeSans9pt7b.h>

// === Struttura glifo ===
typedef struct {
  uint8_t width;
  uint8_t height;
  uint16_t offset;
} GlyphInfo;

// === Bitmap carattere 'A' ===
const uint8_t MyFont8x12_Bitmap[] PROGMEM = {
  0b00011000,
  0b00100100,
  0b00100100,
  0b01000010,
  0b01000010,
  0b01111110,
  0b10000001,
  0b10000001,
  0b10000001,
  0b10000001,
  0b10000001,
  0b00000000
};

const GlyphInfo MyFont8x12_Glyphs[] PROGMEM = {
  {8, 12, 0}  // solo 'A'
};

// === Colori ===
#define BLACK    0x0000
#define BLUE     0x001F
#define RED      0xF800
#define GREEN    0x07E0
#define CYAN     0x07FF
#define MAGENTA  0xF81F
#define YELLOW   0xFFE0 
#define WHITE    0xFFFF
#define ORANGE   0xFD20

// === Dimensioni TFT ===
#define TFT_WIDTH  240
#define TFT_HEIGHT 320
#define TFT_ROTATION 2

// === Pin TFT ===
#define LCD_RD   A0
#define LCD_WR   A1
#define LCD_RS   A2
#define LCD_CS   A3
#define LCD_RST  A4

#define DATA_PORT PORTD
#define DATA_DDR  DDRD
#define DATA_MASK 0xFC

#define DATA_PORT_H PORTB
#define DATA_DDR_H  DDRB
#define DATA_MASK_H 0x03

// === Funzioni basso livello ===
void LCD_write(uint8_t d) {
  digitalWrite(LCD_WR, LOW);
  PORTD = (PORTD & ~DATA_MASK) | (d & 0xFC);
  PORTB = (PORTB & ~DATA_MASK_H) | (d & 0x03);
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
  LCD_command(0x36);
  switch (rotation) {
    case 0: LCD_data(0x48); break;
    case 1: LCD_data(0x28); break;
    case 2: LCD_data(0x88); break;
    case 3: LCD_data(0xE8); break;
  }
}

void setWindow(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2) {
  LCD_command(0x2A);
  LCD_data16(x1);
  LCD_data16(x2);
  LCD_command(0x2B);
  LCD_data16(y1);
  LCD_data16(y2);
  LCD_command(0x2C);
}

void LCD_Init() {
  pinMode(LCD_RD, OUTPUT);
  pinMode(LCD_WR, OUTPUT);
  pinMode(LCD_RS, OUTPUT);
  pinMode(LCD_CS, OUTPUT);
  pinMode(LCD_RST, OUTPUT);

  DATA_DDR |= DATA_MASK;
  DATA_DDR_H |= DATA_MASK_H;

  digitalWrite(LCD_RST, HIGH); delay(5);
  digitalWrite(LCD_RST, LOW); delay(15);
  digitalWrite(LCD_RST, HIGH); delay(15);

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
  LCD_command(0x3A); LCD_data(0x55); // 16-bit
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
  setWindow(x, y, x, y);
  LCD_data16(color);
}

void fillScreen(uint16_t color) {
  setWindow(0, 0, TFT_WIDTH - 1, TFT_HEIGHT - 1);
  for (uint32_t i = 0; i < (uint32_t)TFT_WIDTH * TFT_HEIGHT; i++) {
    LCD_data16(color);
  }
}

// === Classe personalizzata GFX ===
class MyTFT : public Adafruit_GFX {
public:
  MyTFT() : Adafruit_GFX(TFT_WIDTH, TFT_HEIGHT) {}

  void drawPixel(int16_t x, int16_t y, uint16_t color) override {
    drawPixelLowLevel(x, y, color);
  }

  void drawCharLowLevel(int16_t x, int16_t y, char c, uint16_t color, uint8_t scale = 1) {
    if (c != 'A') return;
    
    GlyphInfo g;
    memcpy_P(&g, &MyFont8x12_Glyphs[0], sizeof(GlyphInfo));
    
    for (uint8_t row = 0; row < g.height; row++) {
      uint8_t bits = pgm_read_byte(&MyFont8x12_Bitmap[g.offset + row]);
      
      for (uint8_t col = 0; col < g.width; col++) {
        if (bits & (0x80 >> col)) {
          // Disegna un blocco "scale × scale" per ogni pixel
          for (uint8_t sy = 0; sy < scale; sy++) {
            for (uint8_t sx = 0; sx < scale; sx++) {
              drawPixelLowLevel(x + col * scale + sx, y + row * scale + sy, color);
            }
          }
        }
      }
    }
  }

  void printLowLevel(int16_t x, int16_t y, const char* str, uint16_t color, uint8_t scale = 1) {
    int16_t xo = x;
    while (*str) {
      if (*str == '\n') {
        y += 12 * scale;
        x = xo;
      } else {
        if (*str == 'A') drawCharLowLevel(x, y, *str, color, scale);
        x += 8 * scale;
      }
      str++;
    }
  }
};

MyTFT tft;

void setup() {
  Serial.begin(9600);
  LCD_Init();
  fillScreen(BLACK);

  // Testo normale (scala 1x)
  tft.printLowLevel(10, 20, "A A A", WHITE, 1);

  // Testo 2x più grande
  tft.printLowLevel(10, 50, "A A A", YELLOW, 2);

  // Testo 3x più grande
  tft.printLowLevel(10, 100, "A A A", RED, 3);
}

void loop() {
  // loop vuoto
}