#ifndef MATRIZ
#define MATRIZ

#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/clocks.h"

// Tamanho da matriz de LEDs
#define MATRIZ_ROWS 5
#define MATRIZ_COLS 5
#define MATRIZ_SIZE MATRIZ_ROWS * MATRIZ_COLS

// Cores
#define VERMELHO 0
#define VERDE 1
#define AZUL 2
#define BRANCO 3
#define AMARELO 4
#define APAGADO 5

// Estrutura para armazenar um pixel com valores RGB
typedef struct Rgb
{
  uint8_t r;
  uint8_t g;
  uint8_t b;
} Rgb;

void matriz_init();
Rgb matriz_get_cor(uint8_t cor);
void matriz_clear();
void matriz_send_data();

void matriz_draw_linha(uint pontoInit, uint linha, uint n, Rgb cor);
void matriz_draw_coluna(uint pontoInit, uint coluna, uint8_t n, Rgb cor);
void matriz_draw_frame(Rgb frame[MATRIZ_ROWS][MATRIZ_COLS]);

#endif