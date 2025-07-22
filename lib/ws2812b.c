#include "ws2812b.h"
#include "ws2812b.pio.h"

/*------------------------ VARIÁVEIS GLOBAIS ------------------------*/

// Pino
static const uint matriz_pin = 7;

// Buffer para matriz de LEDs
static Rgb matriz[MATRIZ_SIZE];

// Variáveis para configuração do PIO
static PIO np_pio;
static uint sm;

/*------------------------- FUNÇÕES HELPER --------------------------*/

// Implementação da função PIO
void ws2812b_program_init(PIO pio, uint sm, uint offset, uint pin, float freq) 
{

  pio_gpio_init(pio, pin);
  
  pio_sm_set_consecutive_pindirs(pio, sm, pin, 1, true);
  
  // Program configuration.
  pio_sm_config c = ws2812b_program_get_default_config(offset);
  sm_config_set_sideset_pins(&c, pin); // Uses sideset pins.
  sm_config_set_out_shift(&c, true, true, 8); // 8 bit transfers, right-shift.
  sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_TX); // Use only TX FIFO.
  float prescaler = clock_get_hz(clk_sys) / (10.f * freq); // 10 cycles per transmission, freq is frequency of encoded bits.
  sm_config_set_clkdiv(&c, prescaler);
  
  pio_sm_init(pio, sm, offset, &c);
  pio_sm_set_enabled(pio, sm, true);
}

// Altera a cor de um LED no buffer da matriz
static void alterar_cor_led(int i, uint8_t red, uint8_t green, uint8_t blue)
{
  matriz[i].r = red;
  matriz[i].g = green;
  matriz[i].b = blue;
}

// Converte a posição de um array bidimensional para a posição correspondente na matriz de LED
static uint mapear_pos_buffer(uint linha, uint coluna)
{ 
  // Inverter linha
  uint linhaMatriz = MATRIZ_ROWS - 1 - linha;

  uint colunaMatriz;
  if (linha % 2 == 0)
  {
    // Se a linha for par, a informação é enviada da direita para a esquerda (inversão da coluna)
    colunaMatriz = MATRIZ_COLS - 1 - coluna;
  }
  else
  {
    // Senão, é enviada da esquerda para a direita (coluna é a mesma)
    colunaMatriz = coluna;
  }

  // Converter novo indíce de array bidimensional para unidimensional e retornar valor
  return MATRIZ_ROWS * linhaMatriz + colunaMatriz;
}

/*----------------------------- FUNÇÕES -----------------------------*/

// Inicializa a matriz de LED WS2812B. Baseado no exemplo neopixel_pio do repo BitDogLab
void matriz_init()
{
  // Cria PIO
  uint offset = pio_add_program(pio0, &ws2812b_program);
  np_pio = pio0;

  // Pede uma máquina de estado do PIO
  sm = pio_claim_unused_sm(np_pio, false);
  if (sm < 0) 
  {
    np_pio = pio1;
    sm = pio_claim_unused_sm(np_pio, true);
  }

  // Roda programa na máquina de estado
  ws2812b_program_init(np_pio, sm, offset, matriz_pin, 800000.f);

  matriz_clear();
  matriz_send_data();
}

// Retorna uma struct RGB da cor específicada
Rgb matriz_get_cor(uint8_t cor)
{ 
  Rgb cor_struct;
  switch (cor)
  {
    case VERMELHO:
      cor_struct.r = 1;
      cor_struct.g = 0;
      cor_struct.b = 0;
      break;
    case VERDE:
      cor_struct.r = 0;
      cor_struct.g = 1;
      cor_struct.b = 0;
      break;
    case AZUL:
      cor_struct.r = 0;
      cor_struct.g = 0;
      cor_struct.b = 1;
      break;
    case BRANCO:
      cor_struct.r = 1;
      cor_struct.g = 1;
      cor_struct.b = 1;
      break;
    case AMARELO:
      cor_struct.r = 1;
      cor_struct.g = 1;
      cor_struct.b = 0;
      break;
    default:
      cor_struct.r = 0;
      cor_struct.g = 0;
      cor_struct.b = 0;
      break;
  }
  return cor_struct;
}

// Limpa buffer
void matriz_clear()
{
   for (uint i = 0; i < MATRIZ_SIZE; i++) 
   {
     alterar_cor_led(i, 0, 0, 0);
   }
}

// Atualiza a matriz de LEDs com o conteúdo do buffer
void matriz_send_data()
{
  for (int i = 0; i < MATRIZ_SIZE; i++)
  {
    pio_sm_put_blocking(np_pio, sm, matriz[i].g);
    pio_sm_put_blocking(np_pio, sm, matriz[i].r);
    pio_sm_put_blocking(np_pio, sm, matriz[i].b);

  }
  sleep_us(100); // Espera 100us para a próxima mudança, de acordo com o datasheet
}

// Desenha n LEDs em uma linha na matriz dado uma coluna inicial (da esquerda para direita)
void matriz_draw_linha(uint pontoInit, uint linha, uint n, Rgb cor)
{   
    for (int i = pontoInit; i < pontoInit + n; i++)
    {
        // Obter indíce correto de acordo com o modo de envio de informação do WS2812B e adicionar pixel correspondente ao buffer
        uint index = mapear_pos_buffer(linha, i);
        alterar_cor_led(index, cor.r, cor.g, cor.b);
    }
}

// Desenha n LEDs em uma coluna na matriz dado uma linha inicial (de baixo para cima)
void matriz_draw_coluna(uint pontoInit, uint coluna, uint8_t n, Rgb cor)
{
  for (int l = pontoInit; l > pontoInit - n; l--)
  {
    // Obter indíce correto de acordo com o modo de envio de informação do WS2812B e adicionar pixel correspondente ao buffer
    uint index = mapear_pos_buffer(l, coluna);
    alterar_cor_led(index, cor.r, cor.g, cor.b);
  }
}

// Desenha um frame MATRIZ_ROWSxMATRIZ_COLS no buffer da matriz de LED
void matriz_draw_frame(Rgb frame[MATRIZ_ROWS][MATRIZ_COLS])
{
  // Iterar por cada pixel
  for (int linha = 0; linha < MATRIZ_ROWS; linha++)
  {
    for (int coluna = 0; coluna < MATRIZ_COLS; coluna++)
    { 
      // Obter indíce correto de acordo com o modo de envio de informação do WS2812B e adicionar pixel correspondente ao buffer
      uint index = mapear_pos_buffer(linha, coluna);
      alterar_cor_led(index, frame[linha][coluna].r, frame[linha][coluna].g, frame[linha][coluna].b);
    }
  }
}
