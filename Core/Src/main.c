/* USER CODE BEGIN Header */

/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

//Struct that will be saved in EEPROM
typedef struct {
	// The checkpoint's information will be on top of the EEPROM
	uint32_t valid_checkpoint;	// Contains 0xACCEDED if there is a valid checkpoint
	uint32_t latest_checkpoint;	// Index of the most recent checkpoint
	uint32_t stack_size;	// Stack dimension (bytes)

	// Main Stack Pointer value at checkpoint time
	uint32_t msp;

	//Hardware exception frame registers
	uint32_t r0;
	uint32_t r1;
	uint32_t r2;
	uint32_t r3;
	uint32_t r12;
	uint32_t lr;
	uint32_t pc;
	uint32_t psr;

} SystemState;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define MATRIX_SIZE 15

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CRC_HandleTypeDef hcrc;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

//Initialization of the struct for future checkpoints
SystemState saved_context = {0};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_CRC_Init(void);
/* USER CODE BEGIN PFP */

// Function declarations
void Capture_Registers(uint32_t *stack_pointer); // Saves the MSP, the Exception frame and registers R4-R11
void Save_State_EEPROM(SystemState *stato); // Saves in EEPROM local variables and the elements above
void Restore_Context(void); // Restores EEPROM's data in RAM
void LU_Decomposition(void);
void Print_Matrix(float mat[MATRIX_SIZE][MATRIX_SIZE], const char* name); // Function used to show the actual process (debug print)
void Print_Reset_Cause(void);//Function used to show the cause of the reset

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
extern uint32_t _estack;

// Calculates the dimension of the Stack
void Analyze_Stack(void)
{
    uint32_t starting_address = (uint32_t)&_estack; // Stack top (provided by linker)
    uint32_t current_address = __get_MSP();  // Read current MSP via CMSIS
    uint32_t stack_size_bytes = starting_address - current_address;

    printf(" Starting address: 0x%08lX \r\n", starting_address);
    printf(" Current MSP:      0x%08lX \r\n", current_address);
    printf(" Stack dimension:  %lu byte \r\n\r\n", stack_size_bytes);
}

void Software_Delay(void)
{
    for (volatile uint32_t j = 0; j < 16000000; j++) {	//5 seconds delay to see the progress
    }
}

void Print_Reset_Cause(void)
{
    printf("\r\n\r\n Reset cause: ");

        if (__HAL_RCC_GET_FLAG(RCC_FLAG_PORRST)) {
            printf("Power-On / Power-Down Reset (POR/PDR) \r\n");
        }
        else if (__HAL_RCC_GET_FLAG(RCC_FLAG_PINRST)) {
            printf("Hardware Pin Reset (NRST Button) \r\n");
        }
        else if (__HAL_RCC_GET_FLAG(RCC_FLAG_SFTRST)) {
            printf("Software Reset (NVIC_SystemReset) \r\n");
        }
        else if (__HAL_RCC_GET_FLAG(RCC_FLAG_IWDGRST)) {
            printf("Independent Watchdog Timeout \r\n");
        }
        else if (__HAL_RCC_GET_FLAG(RCC_FLAG_WWDGRST)) {
            printf("Window Watchdog Timeout \r\n");
        }
        else if (__HAL_RCC_GET_FLAG(RCC_FLAG_LPWRRST)) {
            printf("Low-Power Reset \r\n");
        }
        else if (__HAL_RCC_GET_FLAG(RCC_FLAG_OBLRST)) {
            printf("Option Byte Load Reset \r\n");
        }
        else {
            printf("Unidentifiable flags \r\n");
        }

    __HAL_RCC_CLEAR_RESET_FLAGS();
}

void Print_Matrix(float mat[MATRIX_SIZE][MATRIX_SIZE], const char* name)
{
    printf("\r\n %s matrix: \r\n", name);
    for (int r = 0; r < MATRIX_SIZE; r++) {
        for (int c = 0; c < MATRIX_SIZE; c++) {
            printf("%7.2f ", mat[r][c]);	//%7.2f to align columns
        }
        printf("\r\n");
    }
}

void LU_Decomposition(void)
{
	//FIRST PHASE: LU DECOMPOSITION
	printf("\r\n====== STARTING LU DECOMPOSITION ======\r\n");

    // 1. Local variables declarations
    float A[MATRIX_SIZE][MATRIX_SIZE];
    float L[MATRIX_SIZE][MATRIX_SIZE] = {0};
    float U[MATRIX_SIZE][MATRIX_SIZE] = {0};

    // 2. Matrix creation, we use incremental values to have a deterministic process
    printf("\r\n Initializing matrix %d x %d \r\n", MATRIX_SIZE, MATRIX_SIZE);

    for (int i = 0; i < MATRIX_SIZE; i++) {
        for (int j = 0; j < MATRIX_SIZE; j++) {
            A[i][j] = (float)(i * MATRIX_SIZE + j + 1);
            if (i == j) A[i][j] += 100.0f;	// To avoid divisions by zero we increment the values on the diagonal
        }
    }


    // 3. Calculation process (complexity O(n^3))
    printf("\r\n Starting calculation: \r\n");

    for (int i = 0; i < MATRIX_SIZE; i++)
    {
        printf("\r\n Calculating [%d / %d] \r\n", i + 1, MATRIX_SIZE);

        // Upper matrix
        for (int k = i; k < MATRIX_SIZE; k++) {
            float sum = 0.0f;
            for (int j = 0; j < i; j++) {
                sum += (L[i][j] * U[j][k]);
            }
            U[i][k] = A[i][k] - sum;
        }

        // Lower matrix
        for (int k = i; k < MATRIX_SIZE; k++) {
            if (i == k) {
                L[i][i] = 1.0f;
            } else {
                float sum = 0.0f;
                for (int j = 0; j < i; j++) {
                    sum += (L[k][j] * U[j][i]);
                }
                L[k][i] = (A[k][i] - sum) / U[i][i];
            }
        }
        printf("\r\n Matrix at the end of the phase %d: \r\n", i + 1);
        Print_Matrix(L, "Lower");
        Print_Matrix(U, "Upper");

        // Checkpoint after each phase (Calls interrupt by PendSV)
        printf("\r\n Saving state in EEPROM (Checkpoint %d) \r\n", i + 1);
        SCB->ICSR |= SCB_ICSR_PENDSVSET_Msk;
        Software_Delay();
    }



    //SECOND PHASE: SYSTEM RESOLUTION
	printf("\r\n====== SYSTEM RESOLUTION ======\r\n");

	// 1. Vectors declaration (as local variables in stack)
	float b[MATRIX_SIZE];	// Right-hand side vector
	float y[MATRIX_SIZE] = {0};	// Intermediate vector
	float x[MATRIX_SIZE] = {0};	// Solution vector

	printf("\r\n Right-hand side vector (b): \r\n");
	for (int i = 0; i < MATRIX_SIZE; i++) {
		b[i] = (float)(i + 1) * 10.0f; // Random numbers (always multiple of 10)
		printf("%7.2f \r\n", b[i]);
	}

	printf("\r\n Last Checkpoint before solving the system \r\n");
	SCB->ICSR |= SCB_ICSR_PENDSVSET_Msk;
	Software_Delay();


	// 2. Forward Substitution (L * y = b)
	printf("\r\n Forward Substitution \r\n");
	for (int i = 0; i < MATRIX_SIZE; i++) {
		float sum = 0.0f;
		for (int j = 0; j < i; j++) {
			sum += L[i][j] * y[j];
		}
		y[i] = (b[i] - sum) / L[i][i];
	}

	printf("\r\n Saving state before Backward substitution \r\n");
	SCB->ICSR |= SCB_ICSR_PENDSVSET_Msk;
	Software_Delay();


	// 3. Backward Substitution (U * x = y)
	printf("\r\n Backward Substitution \r\n");
	for (int i = MATRIX_SIZE - 1; i >= 0; i--) {
		float sum = 0.0f;
		for (int j = i + 1; j < MATRIX_SIZE; j++) {
			sum += U[i][j] * x[j];
		}
		x[i] = (y[i] - sum) / U[i][i];
	}


	// 4. System solutions
	printf("\r\n====== SYSTEM SOLUTION ======\r\n");
	for (int i = 0; i < MATRIX_SIZE; i++) {
		printf("\r\n x[%d] = %7.4f \r\n ", i, x[i]);
	}


	printf("\r\n====== FUNCTION COMPLETED ======\r\n");


	// Cleaning the magic word in EEPROM to invalid any checkpoint
	HAL_FLASHEx_DATAEEPROM_Unlock();
	HAL_FLASHEx_DATAEEPROM_Program(FLASH_TYPEPROGRAMDATA_WORD, 0x08080000, 0x00000000);
	HAL_FLASHEx_DATAEEPROM_Lock();
	printf("\r\n Magic word deleted \r\n");
}
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
  MX_USART2_UART_Init();
  MX_CRC_Init();
  /* USER CODE BEGIN 2 */
  /* USER CODE BEGIN 2 */

  HAL_Delay(2000);
  Print_Reset_Cause();
  printf("\r\n====== SYSTEM INITIALIZATION ======\r\n");

  //HAL_FLASHEx_DATAEEPROM_Unlock();
  //HAL_FLASHEx_DATAEEPROM_Program(FLASH_TYPEPROGRAMDATA_WORD, 0x08080000, 0x00000000);
  //HAL_FLASHEx_DATAEEPROM_Lock();

  printf("\r\n Stack status before the function: \r\n");
  Analyze_Stack();

    // Searching for a valid checkpoint in EEPROM
    uint32_t check_validity = *(__IO uint32_t *)(0x08080000);
    if (check_validity == 0xACCEDED) {
            printf(" Valid checkpoint found in EEPROM, starting recovery process \r\n");
            HAL_Delay(1000);

            // Static variables to avoid overwriting registers during the recovery process.`
            static uint32_t eeprom_addr;
            static uint32_t *ptr_struct;
            static uint32_t struct_size;
            static uint32_t *ptr_ram;
            static uint32_t ram_words;
            static uint32_t i;

            // Copy of the registers in RAM
            eeprom_addr = 0x08080000;
            ptr_struct = (uint32_t*)&saved_context;
            struct_size = sizeof(SystemState) / 4;
            for (i = 0; i < struct_size; i++) {
                ptr_struct[i] = *(__IO uint32_t *)eeprom_addr;
                eeprom_addr += 4;
            }

            // Copy of the remaining values
            ptr_ram = (uint32_t*)saved_context.msp;
            ram_words = saved_context.stack_size / 4;

            // Restoring R4-R11
            for (i = 0; i < 8; i++) {
                ptr_ram[i] = *(__IO uint32_t *)eeprom_addr;
                eeprom_addr += 4;
            }

            // Restoring Local variables
            for (i = 16; i < ram_words; i++) {
                ptr_ram[i] = *(__IO uint32_t *)eeprom_addr;
                eeprom_addr += 4;
            }


            Restore_Context();
    }
    else
    {
        printf(" No valid checkpoint found \r\n");
    }

    //Come back to function with context and memory restored
    LU_Decomposition();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  RCC_OscInitStruct.PLL.PLLDIV = RCC_PLL_DIV3;
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
}

/**
  * @brief CRC Initialization Function
  * @param None
  * @retval None
  */
static void MX_CRC_Init(void)
{

  /* USER CODE BEGIN CRC_Init 0 */

  /* USER CODE END CRC_Init 0 */

  /* USER CODE BEGIN CRC_Init 1 */

  /* USER CODE END CRC_Init 1 */
  hcrc.Instance = CRC;
  if (HAL_CRC_Init(&hcrc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CRC_Init 2 */

  /* USER CODE END CRC_Init 2 */

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
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void PendSV_Handler(void) __attribute__((naked));
void PendSV_Handler(void)
{
    __asm volatile(
        "push {r4-r11} \n"           // Saving register R4-R11
        "mrs r0, msp \n"             // Sending MSP to function using R0

        "push {r3, lr} \n"           // Saving EXC_RETURN in R3
        "bl Capture_Registers \n"    // Branch to function (with link), original LR value is saved in R3
        "pop {r3, lr} \n"            // Restoring EXC_RETURN

        "pop {r4-r11} \n"            // Restoring other registers
        "bx lr \n"                   // Reading EXC_RETURN and come back to function
    );
}

//Saves registers values in the struct
void Capture_Registers(uint32_t *stack_pointer)
{

    saved_context.valid_checkpoint = 0xACCEDED;
    saved_context.latest_checkpoint = 1;		//will be used for future checks (CRC, DualSlot A-B)
    saved_context.msp = (uint32_t)stack_pointer;

    //Start from index 8 cause positions from 0 to 7 are occupied by registers R4-R11 (manually pushed)
    saved_context.r0 = stack_pointer[8];
    saved_context.r1 = stack_pointer[9];
    saved_context.r2 = stack_pointer[10];
    saved_context.r3 = stack_pointer[11];
    saved_context.r12 = stack_pointer[12];
    saved_context.lr = stack_pointer[13];
    saved_context.pc = stack_pointer[14];
    saved_context.psr= stack_pointer[15];

    Save_State_EEPROM(&saved_context);
}

//Saves the stack values in EEPROM
void Save_State_EEPROM(SystemState *saved_state)
{
	printf("\r\n Starting EEPROM dump \r\n");

    // MSP validity check
    uint32_t eeprom_address = 0x08080000;
    uint32_t starting_address = (uint32_t)&_estack;
    uint32_t current_address = saved_state->msp;
    if (current_address > starting_address || current_address < 0x20000000) {
        printf(" Error: Invalid MSP (%08lX) \r\n", current_address);
        return;
    }

    saved_state->stack_size = starting_address - current_address;
    printf(" Bytes to copy in EEPROM: %lu \r\n", saved_state->stack_size);

	//Starting EEPROM dump timer
	CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
	DWT->CYCCNT = 0;
	DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
	uint32_t start_cycles = DWT->CYCCNT;

    //Writing values in EEPROM
    HAL_FLASHEx_DATAEEPROM_Unlock();

    // Saving SystemState starting from address 0x08080000
    uint32_t *ptr_struct = (uint32_t*)saved_state;
    uint32_t struct_size_words = sizeof(SystemState) / 4;
    for (uint32_t i = 0; i < struct_size_words; i++) {
        HAL_FLASHEx_DATAEEPROM_Program(FLASH_TYPEPROGRAMDATA_WORD, eeprom_address, ptr_struct[i]);
        eeprom_address += 4;	// Advance by 4 bytes at a time
    }

    // Saving remaining data, excluding the redundant Exception frame (index from 8 to 15)
    uint32_t *ptr_ram = (uint32_t*)current_address;
    uint32_t ram_words = saved_state -> stack_size / 4;

    // Saving R4-R11
    for (uint32_t i = 0; i < 8; i++) {
        HAL_FLASHEx_DATAEEPROM_Program(FLASH_TYPEPROGRAMDATA_WORD, eeprom_address, ptr_ram[i]);
        eeprom_address += 4;
    }

    // Saving Local variables
    for (uint32_t i = 16; i < ram_words; i++) {
            HAL_FLASHEx_DATAEEPROM_Program(FLASH_TYPEPROGRAMDATA_WORD, eeprom_address, ptr_ram[i]);
            eeprom_address += 4;
    }

    HAL_FLASHEx_DATAEEPROM_Lock();

    //Stopping EEPROM dump timer
    uint32_t end_cycles = DWT->CYCCNT;
    uint32_t total_cycles = end_cycles - start_cycles;

    uint32_t clock_per_ms = SystemCoreClock / 1000;
    uint32_t time_ms = total_cycles / clock_per_ms;
    uint32_t time_us = (total_cycles % clock_per_ms) / (SystemCoreClock / 1000000);

    printf("\r\n CPU Clock Cycles Spent: %lu\r\n", total_cycles);
    printf(" Effective Dump Time: %lu ms and %lu us\r\n", time_ms, time_us);
    printf("\r\n EEPROM dump completed \r\n");
}


void Restore_Context(void) __attribute__((naked));
void Restore_Context(void)
{
    __asm volatile(
	// Store in R0 the address of the global variable saved_context,
	// which contains the data needed for restoration (loaded from EEPROM).
    "ldr r0, = saved_context \n"

	// Restore MSP
	"ldr r1, [r0, #12] \n"  // MSP is stored at offset 12 in the saved struct; load it into R1
	"msr msp, r1 \n"        // Force the processor to use that MSP value
	"pop {r4-r11} \n"       // Restore R4-R11; with the correct MSP, a single pop is enough

	// Use R1 to move MSP to the saved local variables
	"mrs r1, msp \n"        // Load the current MSP value into R1
	"add r1, r1, #32 \n"    // Skip the exception frame (8 registers x 4 bytes each)
	"msr msp, r1 \n"        // Update MSP

	// Use R2 to restore PC
	"ldr r2, [r0, #40] \n"  // PC is at offset 40 in the saved struct; load it into R2
	"orr r2, r2, #1 \n"     // Set the Thumb bit to avoid a UsageFault (ARM state is not supported)
	"push {r2} \n"          // Push PC (with LSB = 1) onto the top of the stack

	// Use R2 to restore the program status register (arithmetic flags)
	"ldr r2, [r0, #44] \n"      // xPSR value is stored at offset 44 in the struct
	"msr apsr_nzcvq, r2 \n"     // Restore arithmetic flags (NZCVQ)

	// Restore the remaining registers
	"ldr lr,  [r0, #36] \n"
	"ldr r12, [r0, #32] \n"
	"ldr r3,  [r0, #28] \n"
	"ldr r2,  [r0, #24] \n"
	"ldr r1,  [r0, #20] \n"
	"ldr r0,  [r0, #16] \n"  // Restore R0 last, since it is used as the base pointer above

	// Pop PC (which was pushed to the top of the stack) and return
	// to the instruction where the checkpoint was taken
	"pop {pc} \n"
    );
}

int _write(int file, char *ptr, int len) {
    HAL_UART_Transmit(&huart2, (uint8_t*)ptr, len, HAL_MAX_DELAY);
    return len;
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
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
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
