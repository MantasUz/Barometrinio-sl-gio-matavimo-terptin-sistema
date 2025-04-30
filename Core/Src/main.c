/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Pagrindine programa, skirta matuoti barometrini slegi,
  *                   filtruoti ji, rodyti OLED ekrane ir perduoti i kompiuteri.
  ******************************************************************************
  */
/* USER CODE END Header */
/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Pagrindine programa, skirta matuoti barometrini slegi,
  *                   filtruoti ji, rodyti OLED ekrane ir perduoti i kompiuteri.
  ******************************************************************************
  */
/* USER CODE END Header */
#include "main.h"
#include "fonts.h"
#include "ssd1306.h"
#include "bmp280.h"
#include <stdio.h>
#include <string.h>
#include <math.h>

// === Periferiju valdymo strukturos ===
I2C_HandleTypeDef hi2c1;
UART_HandleTypeDef huart2;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim7;

// === BMP280 jutiklio strukturos ===
BMP280_HandleTypedef bmp280;
bmp280_params_t bmp280_params;

// === Nustatymai ===
#define DISKRETIZAVIMO_PERIODAS_MS 10      // Meginiu emimo periodas 10 ms (100 Hz)
#define ISILIMO_LAIKAS_MS 3000             // Laikas, kol sistema ikaista (3 sek)
#define PI 3.1416f                         // Pi reikšme

// === Matavimo kintamieji ===
float slegis_filtruotas = 0.0f;           // Filtruota slegio reikšme
float slegis_jutiklio = 0.0f;             // Neapdorota jutiklio reikšme

// === Min/max sekimas ekrane ir UART ===
float slegis_min_uart = 10000.0f;
float slegis_max_uart = 0.0f;
float slegis_min_ekranas = 10000.0f;
float slegis_max_ekranas = 0.0f;

// === Veliaveles, naudojamos atnaujinimui ===
volatile uint8_t ekrano_atnaujinimas = 0;
volatile uint8_t uart_siuntimas = 0;

// === Pagalbiniai kintamieji ===
uint32_t paskutinis_matavimas = 0;
uint8_t isilimas_baigtas = 0;

// === Funkciju deklaracijos ===
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM6_Init(void);
static void MX_TIM7_Init(void);

// === Žemadažnis filtras (pirmos eiles) ===
float zemadaznis_filtras(float nauja_reiksme, float ankstesne_reiksme) {
    float fs = 100.0f;  // diskretizavimo dažnis (Hz)
    float fc = 10.0f;   // filtro kertinis dažnis (Hz)
    float alfa = (2.0f * PI * fc) / (2.0f * PI * fc + fs);
    return alfa * nauja_reiksme + (1.0f - alfa) * ankstesne_reiksme;
}

// === Pagrindine programa ===
int main(void) {
    HAL_Init();                     
    SystemClock_Config();           
    MX_GPIO_Init();                 
    MX_I2C1_Init();                 
    MX_USART2_UART_Init();         // UART sasaja duomenu siuntimui i PC
    MX_TIM6_Init();                // Timeris ekranui
    MX_TIM7_Init();                // Timeris UART siuntimui

    // Paleidžiamas timerio pertraukimu veikimas
    HAL_TIM_Base_Start_IT(&htim6);
    HAL_TIM_Base_Start_IT(&htim7);

    // OLED ekrano inicijavimas
    ssd1306_Init(&hi2c1);

    // BMP280 jutiklio inicializavimas
    bmp280.i2c = &hi2c1;
    bmp280.addr = BMP280_I2C_ADDRESS_0;
    bmp280_init_default_params(&bmp280_params);
    bmp280_params.mode = BMP280_MODE_NORMAL;
    bmp280_params.filter = BMP280_FILTER_OFF;
    bmp280_params.oversampling_pressure = BMP280_STANDARD;
    bmp280_params.standby = BMP280_STANDBY_250;

    // Jeigu BMP280 inicializavimas nepavyksta, istringame cikle
    if (!bmp280_init(&bmp280, &bmp280_params)) {
    // Siunciame klaida per UART
    char klaida_uart[] = "Klaida: BMP280 nepavyko inicijuoti\r\n";
    HAL_UART_Transmit(&huart2, (uint8_t*)klaida_uart, strlen(klaida_uart), HAL_MAX_DELAY);

    // Rodome klaida per OLED
    ssd1306_Fill(Black);
    ssd1306_SetCursor(0, 10);
    ssd1306_WriteString("BMP280 klaida", Font_7x10, White);
    ssd1306_SetCursor(0, 25);
    ssd1306_WriteString("Patikrink jungti", Font_7x10, White);
    ssd1306_UpdateScreen(&hi2c1);

    // Amžinas sustabdymas
    while (1);
}
    uint32_t pradzios_laikas = HAL_GetTick(); // Laikas, nuo kada matuoti išilima

    // === Pagrindinis ciklas ===
while (1) {
    uint32_t dabar = HAL_GetTick();

    // Slegio matavimas kas DISKRETIZAVIMO_PERIODAS_MS
    if (dabar - paskutinis_matavimas >= DISKRETIZAVIMO_PERIODAS_MS) {
        paskutinis_matavimas = dabar;

        // Nuskaitomas slegis iš jutiklio
        if (bmp280_read_float(&bmp280, NULL, &slegis_jutiklio, NULL)) {
            slegis_jutiklio /= 1000.0f; // hPa -> kPa
            slegis_filtruotas = zemadaznis_filtras(slegis_jutiklio, slegis_filtruotas);
        }
    }

    // Išilimo tikrinimas
    if (!isilimas_baigtas) {
        if (dabar < pradzios_laikas + ISILIMO_LAIKAS_MS) {
            ssd1306_Fill(Black);
            ssd1306_SetCursor(0, 20);
            ssd1306_WriteString("Uzsikrauna...", Font_7x10, White);
            ssd1306_UpdateScreen(&hi2c1);
        } else {
            isilimas_baigtas = 1;
        }
    }

    // Toliau vykdyti tik jei isilimas baigtas
    if (isilimas_baigtas) {
        // Jei reikia atnaujinti ekrana
        if (ekrano_atnaujinimas) {
            ekrano_atnaujinimas = 0;

            if (slegis_filtruotas < slegis_min_ekranas) slegis_min_ekranas = slegis_filtruotas;
            if (slegis_filtruotas > slegis_max_ekranas) slegis_max_ekranas = slegis_filtruotas;

            char eilute[64];
            ssd1306_Fill(Black);

            sprintf(eilute, "Sl: %.3f kPa", slegis_filtruotas);
            ssd1306_SetCursor(0, 0);
            ssd1306_WriteString(eilute, Font_7x10, White);

            sprintf(eilute, "Min: %.3f kPa", slegis_min_ekranas);
            ssd1306_SetCursor(0, 10);
            ssd1306_WriteString(eilute, Font_7x10, White);

            sprintf(eilute, "Max: %.3f kPa", slegis_max_ekranas);
            ssd1306_SetCursor(0, 20);
            ssd1306_WriteString(eilute, Font_7x10, White);

            ssd1306_SetCursor(0, 30);
            if (slegis_filtruotas >= 90.0f && slegis_filtruotas <= 110.0f)
                ssd1306_WriteString("IN RANGE", Font_7x10, White);
            else
                ssd1306_WriteString("OUT OF RANGE", Font_7x10, White);

            ssd1306_UpdateScreen(&hi2c1);
        }

        // Jei reikia siusti UART
        if (uart_siuntimas) {
            uart_siuntimas = 0;

            if (slegis_filtruotas < slegis_min_uart) slegis_min_uart = slegis_filtruotas;
            if (slegis_filtruotas > slegis_max_uart) slegis_max_uart = slegis_filtruotas;

            char busenos_tekstas[16];
            if (slegis_filtruotas >= 90.0f && slegis_filtruotas <= 110.0f)
                strcpy(busenos_tekstas, "IN RANGE");
            else
                strcpy(busenos_tekstas, "OUT OF RANGE");

            char eilute[128];
            sprintf(eilute, "Sl: %.3f | Min: %.3f | Max: %.3f | %s\r\n",
                    slegis_filtruotas, slegis_min_uart, slegis_max_uart, busenos_tekstas);
            HAL_UART_Transmit_IT(&huart2, (uint8_t*)eilute, strlen(eilute));
        }
    }
	}
}

// === Timeriu pertrauktis – nustato veliaveles ekranui ir UART ===
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM6)
        ekrano_atnaujinimas = 1;
    else if (htim->Instance == TIM7)
        uart_siuntimas = 1;
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLLMUL_3;
  RCC_OscInitStruct.PLL.PLLDIV = RCC_PLLDIV_2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_I2C1;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x0040131C;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 24000;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 3000;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief TIM7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM7_Init(void)
{

  /* USER CODE BEGIN TIM7_Init 0 */

  /* USER CODE END TIM7_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM7_Init 1 */

  /* USER CODE END TIM7_Init 1 */
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 24000;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 2000;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM7_Init 2 */

  /* USER CODE END TIM7_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
