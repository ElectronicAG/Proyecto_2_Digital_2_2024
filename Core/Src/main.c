/* USER CODE BEGIN Header */
/**
   ******************************************************************************
 Universidad Del Valle De Guatemalaa
 IE2023: Electrónica digital 2
 Autor: Samuel Tortola - 22094, Alan Gomez - 22115
 Proyecto: Video juego
 Hardware: STM32
 Creado: 30/09/2024
 Última modificación: 3/10/2024
******************************************************************************
 **/

/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "fatfs.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "ili9341.h"
#include "stm32f4xx_hal.h"      // Para la HAL del STM32
#include "stm32f4xx_hal_spi.h"  // Para el manejo de SPI
#include "string.h"
#include "stdio.h"
#include <stdlib.h>


/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
SPI_HandleTypeDef hspi1;
FATFS fs;
FATFS *pfs;
FIL fil;
FRESULT fres;
DWORD fre_clust;
uint32_t totalSpace,freeSpace;
char buffer[100];

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart5;

/* USER CODE BEGIN PV */
extern uint8_t car1[];
extern uint8_t car2[];
extern uint8_t car3[];
extern uint8_t car4[];
extern uint8_t car5[];
extern uint8_t car6[];
extern uint8_t car7[];
extern uint8_t car8[];
extern uint8_t car9[];
extern uint8_t ca10[];
extern uint8_t ca11[];
extern uint8_t ca12[];
extern uint8_t ca13[];
extern uint8_t ca14[];
extern uint8_t ca15[];
extern uint8_t ca16[];
extern uint8_t ca17[];
extern uint8_t ca18[];

extern uint8_t fondo[];


// Array de punteros a los arrays extern
uint8_t* cars[] = {car1, car2, car3, car4, car5, car6, car7, car8, car9, ca10, ca11, ca12, ca13, ca14, ca15, ca16, ca17, ca18};
uint8_t tamano[]={68,49,47,49,39,36,45,39,40,49,39,41,43,41,40,40,41,41};

uint8_t rx_data[1]; // Solo para recibir byte por byte
uint8_t activa = 0;

#define MAX_FILES 70    //Cantidad máxima de archivos permitidos
#define FILENAME_MAX_LEN 50    //Cantidad máxima de carcateres permitidos por nombre

char file_list[MAX_FILES][FILENAME_MAX_LEN];  // Lista de archivos
int file_count = 0;  // Número de archivos

FIL fil;  // Archivo
char file_name[50];  // Nombre del archivo
int receiving_file = 0;  // Flag para indicar si estamos recibiendo un archivo
FRESULT fres;




#define CHUNK_SIZE 512  // Leer la imagen en fragmentos de 512 bytes
#define IMAGE_WIDTH 240
#define IMAGE_HEIGHT 320
#define TOTAL_BYTES (IMAGE_WIDTH * IMAGE_HEIGHT * 2)  // 153600 bytes en total para RGB565




/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_UART5_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void transmit_uart(char *data);

void load_image_from_sd_in_chunks(const char* filename);

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI1_Init();
  MX_FATFS_Init();
  MX_UART5_Init();
  /* USER CODE BEGIN 2 */

  // Montar SD

   fres = f_mount(&fs,"/" , 0);
   if(fres == FR_OK){
       transmit_uart("Micro SD is mounted successfully\n\n\n\n");
   } else {
       transmit_uart("Micro SD is mounted bad\n\n\n\n");
   }


   HAL_UART_Receive_IT(&huart5,rx_data, 1);  // Empezar la recepción por UART5 en modo interrupción


       // Mostrar la imagen en la pantalla
    LCD_Init();
    LCD_Clear(0x00);


    //car1:20x68
    //car2:20x49
    //car3:20x47
    //car4:20x49
    //car5:20x39
    //car6:2036
    //car7:20x45
    //car8:20x39
    //car9:20x40
    //car10:20x49
    //car11:20x39
    //car12:20x41
    //car13:20x43
    //car14:20x41
    //car15:20x41
    //car16:20x40
    //car16:20x40
    //car17:20x41
    //car18:20x41




//    for(int i=0; i<=17; i++){
//    LCD_Bitmap(50,150,20,tamano[i], cars[i]);
 //   HAL_Delay(1000);
   // }


//    LCD_Bitmap(0, 0, 240, 320, fondo);


    load_image_from_sd_in_chunks("fondo.bin");

    for(int i=0; i<=320; i++){
       LCD_Bitmap(50,i,20,tamano[0], cars[0]);
       HAL_Delay(500);
        }



  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */


		if(activa == 1){
			LCD_Clear(0x00);
			FillRect(50, 60, 20, 20, 0xF800);
			FillRect(70, 60, 20, 20, 0x07E0);
			FillRect(90, 60, 20, 20, 0x001F);
			activa = 0;
		}

		else if(activa == 2){
			LCD_Clear(0x00);

			//LCD_Bitmap(0, 0, 320, 240, fondo);
		//	LCD_Bitmap(0, 0, 240, 320, fondo);
			activa = 0;
			}
	}
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief UART5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART5_Init(void)
{

  /* USER CODE BEGIN UART5_Init 0 */

  /* USER CODE END UART5_Init 0 */

  /* USER CODE BEGIN UART5_Init 1 */

  /* USER CODE END UART5_Init 1 */
  huart5.Instance = UART5;
  huart5.Init.BaudRate = 115200;
  huart5.Init.WordLength = UART_WORDLENGTH_8B;
  huart5.Init.StopBits = UART_STOPBITS_1;
  huart5.Init.Parity = UART_PARITY_NONE;
  huart5.Init.Mode = UART_MODE_TX_RX;
  huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart5.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart5) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART5_Init 2 */

  /* USER CODE END UART5_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, LCD_RST_Pin|LCD_D1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LCD_RD_Pin|LCD_WR_Pin|LCD_RS_Pin|LCD_D7_Pin
                          |LCD_D0_Pin|LCD_D2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LCD_CS_Pin|LCD_D6_Pin|LCD_D3_Pin|LCD_D5_Pin
                          |LCD_D4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SD_SS_GPIO_Port, SD_SS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : LCD_RST_Pin LCD_D1_Pin */
  GPIO_InitStruct.Pin = LCD_RST_Pin|LCD_D1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : LCD_RD_Pin LCD_WR_Pin LCD_RS_Pin LCD_D7_Pin
                           LCD_D0_Pin LCD_D2_Pin */
  GPIO_InitStruct.Pin = LCD_RD_Pin|LCD_WR_Pin|LCD_RS_Pin|LCD_D7_Pin
                          |LCD_D0_Pin|LCD_D2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LCD_CS_Pin LCD_D6_Pin LCD_D3_Pin LCD_D5_Pin
                           LCD_D4_Pin */
  GPIO_InitStruct.Pin = LCD_CS_Pin|LCD_D6_Pin|LCD_D3_Pin|LCD_D5_Pin
                          |LCD_D4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : SD_SS_Pin */
  GPIO_InitStruct.Pin = SD_SS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(SD_SS_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void transmit_uart(char *data) {
    HAL_UART_Transmit(&huart5, (uint8_t *)data, strlen(data), HAL_MAX_DELAY);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{

	 if (huart->Instance == UART5) // Verificar si la interrupción es de UART1
	  {

		 if (rx_data[0] == '1'){
			 activa = 1;
		}

		 else if (rx_data[0] == '2'){
		 		 activa = 2;
		 }



	  }

	 // Volver a habilitar la recepción por UART1
	 	   HAL_UART_Receive_IT(&huart5, rx_data, 1);
}



void load_image_from_sd_in_chunks(const char* filename) {
    FIL fil;
    UINT bytes_read;
    FRESULT fres;
    uint8_t chunk_buffer[CHUNK_SIZE];  // Pequeño buffer para leer fragmentos de la SD
    uint32_t total_bytes_read = 0;

    // Abrir el archivo desde la SD
    fres = f_open(&fil, filename, FA_READ);
    if (fres != FR_OK) {
        transmit_uart("Error al abrir el archivo en la SD\n");
        return;
    }

    // Configurar la ventana de la pantalla para toda el área (240x320)
    SetWindows(0, 0, IMAGE_WIDTH - 1, IMAGE_HEIGHT - 1);

    // Leer la imagen en bloques y transmitir a la pantalla
    while (total_bytes_read < TOTAL_BYTES) {
        fres = f_read(&fil, chunk_buffer, CHUNK_SIZE, &bytes_read);
        if (fres != FR_OK || bytes_read == 0) {
            transmit_uart("Error al leer la imagen desde la SD\n");
            break;
        }

        // Enviar los datos leídos a la pantalla
        for (int i = 0; i < bytes_read; i += 2) {
            LCD_DATA(chunk_buffer[i]);      // Primer byte del color (RGB565)
            LCD_DATA(chunk_buffer[i + 1]);  // Segundo byte del color
        }

        total_bytes_read += bytes_read;
    }

    // Cerrar el archivo
    f_close(&fil);
    transmit_uart("Imagen cargada desde la SD en fragmentos\n");
}





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
	while (1) {
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
