//===================================================================================================//
//                                              DRV 8302                                             //
//===================================================================================================//
#include "DRV8302.h"

// Defined based on your schematic
#define STATUS_LED GPIO_NUM_42 

void DRV8302::init()
{
    gpio_set_direction(INL_A, GPIO_MODE_OUTPUT);
    gpio_set_direction(INH_A, GPIO_MODE_OUTPUT);
    gpio_set_direction(INL_B, GPIO_MODE_OUTPUT);
    gpio_set_direction(INH_B, GPIO_MODE_OUTPUT);
    gpio_set_direction(INL_C, GPIO_MODE_OUTPUT);
    gpio_set_direction(INH_C, GPIO_MODE_OUTPUT);
    gpio_set_direction(A_SENSE, GPIO_MODE_INPUT);
    gpio_set_direction(B_SENSE, GPIO_MODE_INPUT);
    gpio_set_direction(C_SENSE, GPIO_MODE_INPUT);
    gpio_set_direction(DC_CAL, GPIO_MODE_OUTPUT);
    gpio_set_direction(MPWM, GPIO_MODE_OUTPUT);
    gpio_set_direction(MOC, GPIO_MODE_OUTPUT);
   
    gpio_set_direction(NFAULT, GPIO_MODE_INPUT);
    gpio_set_pull_mode(NFAULT, GPIO_PULLUP_ONLY);  // Active HIGH logic in your code
   
    gpio_set_direction(NOCTW, GPIO_MODE_INPUT);
    gpio_set_pull_mode(NOCTW, GPIO_PULLUP_ONLY);  // Active HIGH logic in your code
   
    gpio_set_direction(GAIN, GPIO_MODE_OUTPUT);
    gpio_set_direction(SO1, GPIO_MODE_INPUT);
    gpio_set_direction(SO2, GPIO_MODE_INPUT);
    
    gpio_set_direction(EN_DRV, GPIO_MODE_OUTPUT); // Set to output to drive the pin
    
    // Adding the LED initialization
    gpio_set_direction(STATUS_LED, GPIO_MODE_OUTPUT);
}

void DRV8302::setup() {
  // Habilitar driver y buck
  gpio_set_level(EN_DRV, 1);  // Active HIGH
  vTaskDelay(1000);
  printf("DRV8302 habilitado, iniciando calibracion...\n");
  // Verificar estado
  if (gpio_get_level(NFAULT) == 0) {
    printf("ERROR: DRV8302 reporta FALLO");
  } else {
    printf("DRV8302 habilitado correctamente");
    printf("El buck deberia estar generando 5V");
  }
}

void DRV8302::loop() {
  // Monitoreo continuo de fallas
  if (gpio_get_level(NFAULT) == 0) {
    printf("Fallo detectado!");
    // PATTERN: Fast blink for Fault
    gpio_set_level(STATUS_LED, 1);
    vTaskDelay(100 / portTICK_PERIOD_MS);
    gpio_set_level(STATUS_LED, 0);
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
  else if (gpio_get_level(NOCTW) == 0) {
    printf("Advertencia termica!");
    // PATTERN: Slower blink for Thermal Warning
    gpio_set_level(STATUS_LED, 1);
    vTaskDelay(500 / portTICK_PERIOD_MS);
    gpio_set_level(STATUS_LED, 0);
    vTaskDelay(500 / portTICK_PERIOD_MS);
  }
  else {
    // Normal operation: Solid LED
    gpio_set_level(STATUS_LED, 1);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}