/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "fatfs.h"
#include "sdio.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stn1110.h"
#include "UartRingbuffer.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

float f_pitch;
float f_roll;
float f_accelX;
float f_accelY;

int i_accelX;
int i_accelY;

////////////////////////////////////////////SDIO///////////////////////////////////////////////////////////////

int i;

char Filename[] = "Log00000.CSV";

////////////////////////////////////////////SDIO///////////////////////////////////////////////////////////////

////////////////////////////////////////////stn1110///////////////////////////////////////////////////////////

uint32_t Uart6_currentTime;
uint32_t Uart6_previousTime;
const uint32_t Uart6_timeout_ms = 1000;

#define cur_myELM327_STATUS curS;
#define pre_myELM327_STATUS preS;

uint32_t rpm = 0;
uint32_t motorrpm = 0;
uint32_t kph = 0;
uint32_t engineLoad = 0;
uint16_t runTime = 0;
uint8_t fuelType = 0;
int32_t oilTemp = 0;
uint32_t relativePedalPos = 0;
uint32_t throttle = 0;
uint32_t relativeThrottle = 0;
int32_t intakeAirTemp = 0;
uint32_t fuelLevel = 0;
float mafRate = 0; // uint32_t
uint8_t obdStandards = 0;
uint16_t distTravelWithMIL = 0;
uint16_t distSinceCodesCleared = 0;
uint32_t ctrlModVoltage = 0;
int16_t ambientAirTemp = 0;
uint32_t manifoldPressure = 0;
int32_t engineCoolantTemp = 0;
float commandedThrottleActuator = 0;
float commandedAirFuelRatio = 0.0; //stn1110 Data Variables

static uint32_t counter_AccelYp;
static uint32_t counter_Fuel;
//static uint32_t counter_Energy;
static uint32_t counter_Rpm;
//static uint32_t counter_MotorRpm;
static float efficiencyScore;
static uint32_t counter_AccelX;
static uint32_t counter_AccelYm;
static uint32_t counter_AccelYm2;
static uint32_t counter_Time;
static uint32_t counter_KphRpm;
//static uint32_t counter_KphMotorRpm;
static uint32_t counter_Throttle;
static float safetyScore;
static float drivingScore = 10.0;
static float averagedrivingScore = 10.0;
static int counter_N1 = 1;
static int counter_N2 = 1;
static int counter_N3 = 1;
//static int counter_N4 = 1;
static int counter_Idle = 0;
static float AFR = 14.70;

float pre_f_accelX;
float pre_f_accelY;
float pre_instantFuelConsume;
float pre_instantEnergyConsume;
uint32_t pre_rpm;
uint32_t pre_motorrpm;
uint32_t pre_runTime;
int32_t pre_kph;
float pre_commandedThrottleActuator;
float instantFuelConsume = 12.0;
float averageFuelConsume = 12.0;
float averagecommandedThrottleActuator = 8;
float instantEnergyConsume = 12.0;
float averageEnergyConsume = 12.0;



bool flag_aws_publish = false; //flag

char car_state[30] = "";
int pwrCheck;
int rstCheck;
bool errorvaluecheck = true;
bool mafRatecheck = false;

static uint32_t elmstatusCheck = 0;

unsigned long c_time = 0;
unsigned long e_time = 0;

int HOUR = 0;
int MIN = 0;
int SEC = 0; // ?���? ?���? ?��,�?,�?

int E_HOUR = 0;
int E_MIN = 0;
int E_SEC = 0; // ?���? ?���? ?��,�?,�?

////////////////////////////////////////////stn1110///////////////////////////////////////////////////////////

STN1110 stn;

FATFS myFatFS;
FATFS *fs;
FIL myFile;
UINT myBytes;
FRESULT fr;
FILINFO fno;

DWORD fre_clust, fre_sect, tot_sect;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void printError(int8_t curS);
void LogToSDcard();
void LogToSDcard2();
void LogToSDcardError(int8_t curS);
void createNewfile();
void createNewfile2();
void accelYpAlgorithm();
void fuelAlgorithm();
void commandedThrottleActuatorAlgorithm();
void rpmAlgorithm();
void efficiencyScoreAlgorithm();
void efficiencyScoreAlgorithm2();
void accelXAlgorithm();
void accelYmAlgorithm();
void accelYmAlgorithm2();
void timeAlgorithm();
void kphrpmAlgorithm();
void safetyScoreAlgorithm();
void drivingScoreAlgorithm();
void averagedrivingScoreAlgorithm();
void pre_DataUpdate();
void counterReset();
bool errorValueReset();
bool mafRateCheck(float mafRate_value);
void clearData();
bool Uart6_timeout();
void flushInputBuff();
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
  MX_SDIO_SD_Init();
  MX_USART6_UART_Init();
  MX_FATFS_Init();
  MX_DMA_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  Ringbuf_init();
  Ringbuf_init2();

  /////////////////////////////////////////////MPU6050//////////////////////////////////////////////////////////

 //자이로 데이터 받아오기

  /////////////////////////////////////////////MPU6050//////////////////////////////////////////////////////////

 ///////////////////////////////////////////SDIO/////////////////////////////////////////////////////////////

    	    if(f_mount(&myFatFS, SDPath, 1) == FR_OK)
    	     {
    	   	  printf("SD Mount 'Success'\r\n");
    	   	  fr = f_getfree("", &fre_clust, &fs);
    	   	    /* Get total sectors and free sectors */
    	   	    tot_sect = (fs->n_fatent - 2) * fs->csize;
    	   	    fre_sect = fre_clust * fs->csize;
    	   	    /* Print the free space (assuming 512 bytes/sector) */
    	   	    printf("SD card stauts : %lu MB total drive space.\r\n", (tot_sect / 2)/997);
    	   	    printf("Remaining space : %lu MB available.\r\n", (fre_sect / 2)/997);

    	   //	    if (((fre_sect/2)/997) < 10)
    	   //	    {
    	   //			?��?�� �?족시 ?��?��?��?�� 루틴 만들�?
    	   //	    }

    	   	  for (unsigned int k = 0; k < 100000; k++)
    	   	  {
    	   		Filename[3] = k / 10000 + '0'; //만자�?
    	   		Filename[4] = ((k % 10000) / 1000) + '0';//천자�?
    	   		Filename[5] = ((k % 1000) / 100) + '0';//백자�?
    	   		Filename[6] = ((k % 100) / 10) + '0'; //?��?���?
    	   		Filename[7] = k % 10 + '0'; //?��?���?
    	   		fr = f_open(&myFile, Filename, FA_WRITE | FA_CREATE_NEW);
    	   		if (fr == FR_OK)
    	   		{
    	   			break;
    	   		}
    	   	  }
    	   	  printf("Logging to : ");
    	   	  printf("%s\r\n", Filename);
    	   	  f_printf(&myFile, "%s", "First Boot");
    	   	  f_printf(&myFile, "%s", "\n");
    	   	  f_printf(&myFile, "%s", "Time,ELM327_Status,RunTime,Rpm,Kph,EngineLoad,throttle,InstantFuelConsume,counter_AccelYp,counter_Fuel,counter_Rpm,efficiencyScore,counter_AccelX,counter_AccelYm,AccelYm2,counter_Time,counter_KphRpm,safetyScore,drivingScore,averagedrivingScore,f_pitch,f_roll,f_accelX,f_accelY");
    	   	  f_printf(&myFile, "%s", "\n");
    	   	  f_close(&myFile);
    	     }
    	     else { printf("SD Mount 'Failed'\r\n"); NVIC_SystemReset(); }

    	  /////////////////////////////////////////////SDIO/////////////////////////////////////////////////////////////


  /////////////////////////////////////////////stn1110//////////////////////////////////////////////////////////

    if (!stn.begin(&huart6, false, 2000))
    	{
    		printf("Couldn't connect to STN1110");
    		while(1);
    	}
    else printf("Connected to STN1110\r\n");

	uint8_t Uart6_counter = 0;
	uint16_t Uart6_PAYLOAD_LEN;
	char* Uart6_payload;
	bool Uart6_debugMode = false;
	const uint16_t& Uart6_payloadLen = 40;
	Uart6_PAYLOAD_LEN = Uart6_payloadLen;
	Uart6_payload = (char*)malloc(Uart6_PAYLOAD_LEN + 1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
/////////////////////////////////////////////mpu6050//////////////////////////////////////////////////////////
		// clear payload buffer
		memset(Uart6_payload, '\0', Uart6_PAYLOAD_LEN + 1);
		Uart6_counter = 0;
		flushInputBuff();
		Uart_flush();

//		uint8_t str[] = "at+qd\r\n";
//		HAL_UART_Transmit(&huart1, (uint8_t*)str, 8, 1000); //uart번호, 출력할 문자열, 문자 개수, timeout

		Uart_sendstring("at+qd\r\n");

		// prime the timeout timer
		Uart6_previousTime = millis();
		Uart6_currentTime  = Uart6_previousTime;

		while ((Uart6_counter < Uart6_PAYLOAD_LEN) && !Uart6_timeout())
		{
	//		printf("this is sendcommand while loop");

			if (IsDataAvailable())
			{

				char recGyro = Uart_read();

				if (Uart6_debugMode)
				{
					printf("\tReceived char: ");

					if (recGyro == '\f')
						printf("\\f\r\n");
					else if (recGyro == '\n')
						printf("\\n\r\n");
					else if (recGyro == '\r')
						printf("\\r\r\n");
					else if (recGyro == '\t')
						printf("\\t\r\n");
					else if (recGyro == '\v')
						printf("\\v\r\n");
					else
						printf("%c\r\n", recGyro);
				}

				if (recGyro == '<')
				{
					if (Uart6_debugMode)
						printf("Delimiter found\r\n");

					break;
				}
//				else if (!isalnum(recGyro) && (recGyro != ':'))
				else if ((recGyro == '.'))
					continue;

				Uart6_payload[Uart6_counter] = recGyro;
				Uart6_counter++;

			}
		}

		//printf("%s\r\n", Uart6_payload);
		char Gyro_payload[10];
		char* token;
		char del[] = ",";
		memcpy(Gyro_payload, Uart6_payload, sizeof(Gyro_payload));
//		printf("%s\r\n", Gyro_payload);

		token = strtok(Gyro_payload, del);
//		printf("first one : %s\r\n", token);
		i_accelX = atoi(token);
		f_accelX = i_accelX / (float)100;

		token = strtok(NULL, del);
//		printf("second one : %s\r\n", token);
		i_accelY = atoi(token);
		f_accelY = i_accelY / (float)100;

/////////////////////////////////////////////mpu6050//////////////////////////////////////////////////////////


/////////////////////////////////////////////stn1110//////////////////////////////////////////////////////////

  	  	 	 	    static int8_t cur_myELM327_STATUS; // ?��?��?��?�� ?��?���??��
		  	  	    static int8_t pre_myELM327_STATUS; // ?��?��?��?�� ?��?���??��

		  	  	    c_time = millis() / 1000; //?���?

		  	  	    SEC = c_time % 60;
		  	  	    MIN = (c_time / 60) % 60;
		  	  	    HOUR = (c_time / (60 * 60)) % 24;

		  	  	    float tempRPM = stn.rpm();
		  	  	    uint32_t tempVEHICLE_SPEED = stn.kph();
		  	  	    float tempENGINE_LOAD = stn.engineLoad();
		  	  	    uint16_t tempRUN_TIME_SINCE_ENGINE_START = stn.runTime();
		  	  	    uint8_t tempFUEL_TYPE = stn.fuelType();
		  	  	    float tempENGINE_OIL_TEMP = stn.oilTemp();
		  	  	    float tempENGINE_COOLANT_TEMP = stn.engineCoolantTemp();
		  	  	    float tempRELATIVE_ACCELERATOR_PEDAL_POS = stn.relativePedalPos();
		  	  	    float tempTHROTTLE_POSITION = stn.throttle();
		  	  	    float tempCOMMANDED_THROTTLE_ACTUATOR = stn.commandedThrottleActuator();
		  	  	    float tempRELATIVE_THROTTLE_POSITION = stn.relativeThrottle();
		  	  	    float tempINTAKE_AIR_TEMP = stn.intakeAirTemp();
		  	  	    uint8_t tempINTAKE_MANIFOLD_ABS_PRESSURE = stn.manifoldPressure();
		  	  	    float tempFUEL_TANK_LEVEL_INPUT = stn.fuelLevel();
		  	  	    uint8_t tempOBD_STANDARDS = stn.obdStandards();
		  	  	    float tempCONTROL_MODULE_VOLTAGE = stn.ctrlModVoltage();
		  	  	    float tempAMBIENT_AIR_TEMP = stn.ambientAirTemp();
		  	  	    uint16_t tempDISTANCE_TRAVELED_WITH_MIL_ON = stn.distTravelWithMIL();
		  	  	    uint16_t tempDIST_TRAV_SINCE_CODES_CLEARED = stn.distSinceCodesCleared();
		  	  	    float tempMAF_FLOW_RATE = stn.mafRate();
		  	  	    float tempFUEL_AIR_MANDED_EQUIV_RATIO = stn.commandedAirFuelRatio();

		  	  	    mafRateCheck(tempMAF_FLOW_RATE);

		  	  	    curS = stn.status; //?��?��?��?�� 갱신

		  	  	    switch (curS) {
		  	  	         case ELM_SUCCESS:
		  	  	         strcpy(car_state, "ELM_SUCCESS");
		  	  	         break;

		  	  	         case ELM_NO_RESPONSE:
		  	  	         strcpy(car_state, "ELM_NO_RESPONSE");
		  	  	         break;

		  	  	         case ELM_BUFFER_OVERFLOW:
		  	  	         strcpy(car_state, "ELM_BUFFER_OVERFLOW");
		  	  	         break;

		  	  	         case ELM_UNABLE_TO_CONNECT:
		  	  	         strcpy(car_state, "ELM_UNABLE_TO_CONNECT");
		  	  	         break;

		  	  	         case ELM_NO_DATA:
		  	  	         strcpy(car_state, "ELM_NO_DATA");
		  	  	         break;

		  	  	         case ELM_STOPPED:
		  	  	         strcpy(car_state, "ELM_STOPPED");
		  	  	         break;

		  	  	         case ELM_TIMEOUT:
		  	  	         strcpy(car_state, "ELM_TIMEOUT");
		  	  	         break;

		  	  	         default:
		  	  	         strcpy(car_state, "UNKNOWN_ERROR");
		  	  	       }

		  	      if ( (curS != preS) && (mafRatecheck == true)) // 이전상태와 현재상태가 다를시 새파일 생성 (mafRate 값 받아와질시 저장)
		  	          createNewfile();
		  	      else if ( (curS != preS) && (mafRatecheck == false)) // 이전상태와 현재상태가 다를시 새파일 생성 (commandedThrottleActuatorAlgorithm 실행시 저장)
		  	          createNewfile2();

		  	  	    preS = curS; // ?��?��?��?�� ?��?��?��?���? 갱신

		  	  	  if (curS == ELM_SUCCESS)
		  	  	  {
		  	  		 elmstatusCheck = 0; //elmstatusCheck Reset!

		  	  		  runTime = (uint16_t)tempRUN_TIME_SINCE_ENGINE_START; //?��진켜진시?�� ?��?�� ?��?��?���?
		  	  		  fuelType = (int32_t)tempFUEL_TYPE; //?��?�� ?��료정�?
		  	  		  rpm = (uint32_t)tempRPM; //차량 RPM
		  	  		  kph = (uint32_t)tempVEHICLE_SPEED; //차량?��?��
		  	  		  engineLoad = (uint32_t)tempENGINE_LOAD; //?��진�??��
		  	  		  oilTemp = (int32_t)tempENGINE_OIL_TEMP; //?��?��?��?��
		  	  		  engineCoolantTemp = (int32_t)tempENGINE_COOLANT_TEMP; //?���? ?��각수 ?��?��
		  	  		  relativePedalPos = (uint32_t)tempRELATIVE_ACCELERATOR_PEDAL_POS;
		  	  		  throttle = (uint32_t)tempTHROTTLE_POSITION; //?��로�? ?���??��
		  	  		  relativeThrottle = (uint32_t)tempRELATIVE_THROTTLE_POSITION; //?��?? ?��로�? ?���??��
		  	  		  commandedThrottleActuator = (float)tempCOMMANDED_THROTTLE_ACTUATOR; //?��로�? ?��츄에?��?��
		  	  		  intakeAirTemp = (int32_t)tempINTAKE_AIR_TEMP; //?��?��공기 ?��?��
		  	  		  mafRate = (float)tempMAF_FLOW_RATE; //공기?��?��
		  	  		  manifoldPressure = (uint8_t)tempINTAKE_MANIFOLD_ABS_PRESSURE; //?��기매?��?��?�� ?��???��?��
		  	  		  ambientAirTemp = (int16_t)tempAMBIENT_AIR_TEMP; //?��기온?��
		  	  		  distTravelWithMIL = (uint16_t)tempDISTANCE_TRAVELED_WITH_MIL_ON; //경고?�� ?��?��?��?�� 주행거리
		  	  		  distSinceCodesCleared = (uint16_t)tempDIST_TRAV_SINCE_CODES_CLEARED; //DTC ?��거후 주행거리
		  	  		  fuelLevel = (uint32_t)tempFUEL_TANK_LEVEL_INPUT; //?��료레�?
		  	  		  ctrlModVoltage = (uint32_t)tempCONTROL_MODULE_VOLTAGE; //컨트�? 모듈 ?��?��
		  	  		  obdStandards = (uint8_t)tempOBD_STANDARDS; //OBD ?���? - ?��치값 wikipedia �??��
		  	  		  commandedAirFuelRatio = (float)tempFUEL_AIR_MANDED_EQUIV_RATIO;

		  	  	    if (errorValueReset() == true) {printf("Error Value Reset!\r\n"); } //errorvalue 확인후 조건수정
		  	  	    if ( (kph == 0) && (rpm >= 550) ) counter_Idle++;

		  	  	    if (mafRatecheck == true)
		  	  	    {
		  	  	      switch (fuelType) {
		  	  	      case 1 : AFR = 14.7;
		  	  	      break;
		  	  	      case 2 : AFR = 6.4;
		  	  	      break;
		  	  	      case 3 : AFR = 9.0;
		  	  	      break;
		  	  	      case 4 : AFR = 14.6;
		  	  	      break;
		  	  	      case 5 : AFR = 15.5;
		  	  	      break;
		  	  	      case 6 : AFR = 17.2;
		  	  	      break;
		  	  	      default : AFR = 14.7;
		  	  	      break; }

		  	  	      instantFuelConsume = ((kph*AFR*commandedAirFuelRatio*710.0)/(mafRate*3600)); //Km/L
		  	  	      // **instantFuelConsume = kph*1/3600*1/mafRate*AFR*770*commandedAirFuelRatio;    //km/l 단위변환
		  	  	      // **instantFuelConsume = ((mafRate*36000)/(kph*commandedAirFuelRatio*AFR*770)); //1L/100km 단위변환

		  	  	      if ( instantFuelConsume >= 30.0 )
		  	  	      {instantFuelConsume = 30.0;}

		  	  	      printf("instantFuelConsume : ");
		  	  	      printf("%f\r\n", instantFuelConsume);
		  	  	      printf("commandedAirFuelRatio : ");
		  	  	      printf("%f\r\n", commandedAirFuelRatio);
		  	  	      printf("AFR : ");
		  	  	      printf("%f\r\n", AFR);
		  	  	     }

		  	  	    if (mafRatecheck == true) {LogToSDcard();} // 시리얼모니터 확인 및 sd 카드 저장함수
		  	  	    else if (mafRatecheck == false) {LogToSDcard2();}
		  	   	  }
		  	  	  else {
		  	  		  printError(curS);
		  	  	  }

		  	  	if ( (rpm >100) && (pre_rpm > 100) && (curS == ELM_SUCCESS ) ) //알고리즘 실행 조건 수정필요
		  	  	      {
		  	  	        if (mafRatecheck == false)
		  	  	        {
		  	  	          commandedThrottleActuatorAlgorithm();
		  	  	          accelYpAlgorithm();
		  	  	          rpmAlgorithm();
		  	  	          efficiencyScoreAlgorithm2();
		  	  	        }

		  	  	        else if (mafRatecheck == true)
		  	  	        {
		  	  	          fuelAlgorithm();
		  	  	          accelYpAlgorithm();
		  	  	          rpmAlgorithm();
		  	  	          efficiencyScoreAlgorithm();
		  	  	        }

		  	  	        accelXAlgorithm();
		  	  	        accelYmAlgorithm();
		  	  	        accelYmAlgorithm2();
		  	  	        timeAlgorithm();
		  	  	        kphrpmAlgorithm();
		  	  	        safetyScoreAlgorithm();

		  	  	        drivingScoreAlgorithm();
		  	  	        averagedrivingScoreAlgorithm();
		  	  	      }
		  	  	      pre_DataUpdate();

		  	  	      if (elmstatusCheck > 5)
		  	  	      { counterReset(); }

/////////////////////////////////////////////stn1110//////////////////////////////////////////////////////////
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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 100;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 5;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

////////////////////////////////////////////stn1110///////////////////////////////////////////////////////////

void printError(int8_t curS)
{
    elmstatusCheck++;

    if (curS == ELM_NO_RESPONSE)
    {
        printf("ERROR: ELM_NO_RESPONSE");
    }
    else if (curS == ELM_BUFFER_OVERFLOW)
    {
        printf("ERROR: ELM_BUFFER_OVERFLOW");
    }
    else if (curS == ELM_UNABLE_TO_CONNECT)
    {
    	printf("ERROR: ELM_UNABLE_TO_CONNECT");
    }
    else if (curS == ELM_NO_DATA)
    {
    	printf("ERROR: ELM_NO_DATA");
    }
    else if (curS == ELM_STOPPED)
    {
    	printf("ERROR: ELM_STOPPED");
    }
    else if (curS == ELM_TIMEOUT)
    {
    	printf("ERROR: ELM_TIMEOUT");
    }
    else if (curS == ELM_GENERAL_ERROR)
    {
    	printf("ERROR: ELM_GENERAL_ERROR");
    }
    printf("\r\n");


    LogToSDcardError(curS);
}

void LogToSDcard()
   {
	char logBuf[64];
	int loglen;

    // open the file. note that only one file can be open at a time,
    // so you have to close this one before opening another.
    fr = f_open(&myFile, Filename,  FA_WRITE | FA_OPEN_APPEND);

    // if the file is available, write to it:
    if (fr == FR_OK)
    {
        printf("ELM327 Data Logging...\r\n");
        f_printf(&myFile, "%d", HOUR);
        f_printf(&myFile, "%s", "h_");
        f_printf(&myFile, "%d", MIN);
        f_printf(&myFile, "%s", "m_");
        f_printf(&myFile, "%d", SEC);
        f_printf(&myFile, "%s", "s");
    	f_printf(&myFile, "%c", ',');

        f_printf(&myFile, "%s", "ELM_SUCCESS");
    	f_printf(&myFile, "%c", ',');

        f_printf(&myFile, "%u", runTime/60);
    	f_printf(&myFile, "%c", ',');

        f_printf(&myFile, "%u", rpm);
    	f_printf(&myFile, "%c", ',');

        f_printf(&myFile, "%u", kph);
    	f_printf(&myFile, "%c", ',');

        f_printf(&myFile, "%u", engineLoad);
    	f_printf(&myFile, "%c", ',');

        f_printf(&myFile, "%u", throttle);
    	f_printf(&myFile, "%c", ',');

    	f_lseek(&myFile, f_size(&myFile));
    	loglen = sprintf(logBuf, "%f", instantFuelConsume);
    	f_write(&myFile, logBuf, loglen, &myBytes);
        //f_printf(&myFile, "%f", instantFuelConsume);
    	f_printf(&myFile, "%c", ',');

        f_printf(&myFile, "%u", counter_AccelYp);
    	f_printf(&myFile, "%c", ',');

        f_printf(&myFile, "%u", counter_Fuel);
    	f_printf(&myFile, "%c", ',');

        f_printf(&myFile, "%u", counter_Rpm);
    	f_printf(&myFile, "%c", ',');

    	logBuf[0] = {0,};
    	f_lseek(&myFile, f_size(&myFile));
    	loglen = sprintf(logBuf, "%f", efficiencyScore);
    	f_write(&myFile, logBuf, loglen, &myBytes);
        //f_printf(&myFile, "%f", efficiencyScore);
    	f_printf(&myFile, "%c", ',');

        f_printf(&myFile, "%u", counter_AccelX);
    	f_printf(&myFile, "%c", ',');

        f_printf(&myFile, "%u", counter_AccelYm);
    	f_printf(&myFile, "%c", ',');

        f_printf(&myFile, "%u", counter_AccelYm2);
    	f_printf(&myFile, "%c", ',');

        f_printf(&myFile, "%u", counter_Time);
    	f_printf(&myFile, "%c", ',');

        f_printf(&myFile, "%u", counter_KphRpm);
    	f_printf(&myFile, "%c", ',');

    	logBuf[0] = {0,};
    	f_lseek(&myFile, f_size(&myFile));
    	loglen = sprintf(logBuf, "%f", safetyScore);
    	f_write(&myFile, logBuf, loglen, &myBytes);
        //f_printf(&myFile, "%f", safetyScore);
    	f_printf(&myFile, "%c", ',');

    	logBuf[0] = {0,};
    	f_lseek(&myFile, f_size(&myFile));
    	loglen = sprintf(logBuf, "%f", drivingScore);
    	f_write(&myFile, logBuf, loglen, &myBytes);
        //f_printf(&myFile, "%f", drivingScore);
    	f_printf(&myFile, "%c", ',');

    	logBuf[0] = {0,};
    	f_lseek(&myFile, f_size(&myFile));
    	loglen = sprintf(logBuf, "%f", averagedrivingScore);
    	f_write(&myFile, logBuf, loglen, &myBytes);
        //f_printf(&myFile, "%f", averagedrivingScore);
    	f_printf(&myFile, "%c", ',');

    	logBuf[0] = {0,};
    	f_lseek(&myFile, f_size(&myFile));
    	loglen = sprintf(logBuf, "%.2f", f_pitch);
    	f_write(&myFile, logBuf, loglen, &myBytes);
        //f_printf(&myFile, "%f", f_pitch);
    	f_printf(&myFile, "%c", ',');

    	logBuf[0] = {0,};
    	f_lseek(&myFile, f_size(&myFile));
    	loglen = sprintf(logBuf, "%.2f", f_roll);
    	f_write(&myFile, logBuf, loglen, &myBytes);
        //f_printf(&myFile, "%f", f_roll);
    	f_printf(&myFile, "%c", ',');

    	logBuf[0] = {0,};
    	f_lseek(&myFile, f_size(&myFile));
    	loglen = sprintf(logBuf, "%.2f", f_accelX);
    	f_write(&myFile, logBuf, loglen, &myBytes);
        //f_printf(&myFile, "%f", f_accelX);
    	f_printf(&myFile, "%c", ',');

    	logBuf[0] = {0,};
    	f_lseek(&myFile, f_size(&myFile));
    	loglen = sprintf(logBuf, "%.2f", f_accelY);
    	f_write(&myFile, logBuf, loglen, &myBytes);
        //f_printf(&myFile, "%f", f_accelY);
    	f_printf(&myFile, "%c", ',');
    	f_printf(&myFile, "%s", "\n");

        f_close(&myFile);
    }
    else
    	printf("Failed to open %s \r\n", Filename);

    //if the file isn't open, pop up an error:
    /*else
    {
        Serial.println("error opening LOG_XXXX.txt");
        delay(100);
    }*/
}

void LogToSDcard2()
   {
	char logBuf[64];
	int loglen;

    // open the file. note that only one file can be open at a time,
    // so you have to close this one before opening another.
    fr = f_open(&myFile, Filename,  FA_WRITE | FA_OPEN_APPEND);

    // if the file is available, write to it:
    if (fr == FR_OK)
    {
    	f_printf(&myFile, "%d", HOUR);
    	f_printf(&myFile, "%s", "h_");
    	f_printf(&myFile, "%d", MIN);
    	f_printf(&myFile, "%s", "m_");
    	f_printf(&myFile, "%d", SEC);
    	f_printf(&myFile, "%s", "s");
    	f_printf(&myFile, "%c", ',');

    	f_printf(&myFile, "%s", "ELM_SUCCESS");
    	f_printf(&myFile, "%c", ',');

    	f_printf(&myFile, "%u", runTime/60);
    	f_printf(&myFile, "%c", ',');

 	    f_printf(&myFile, "%u", rpm);
    	f_printf(&myFile, "%c", ',');

    	f_printf(&myFile, "%u", kph);
    	f_printf(&myFile, "%c", ',');

    	f_printf(&myFile, "%u", engineLoad);
    	f_printf(&myFile, "%c", ',');

    	f_printf(&myFile, "%u", throttle);
    	f_printf(&myFile, "%c", ',');


    	f_lseek(&myFile, f_size(&myFile));
    	loglen = sprintf(logBuf, "%f", commandedThrottleActuator);
    	f_write(&myFile, logBuf, loglen, &myBytes);
    	//f_printf(&myFile, "%f", commandedThrottleActuator);
    	f_printf(&myFile, "%c", ',');

 	    f_printf(&myFile, "%u", counter_AccelYp);
    	f_printf(&myFile, "%c", ',');

 	    f_printf(&myFile, "%u", counter_Throttle);
    	f_printf(&myFile, "%c", ',');

 	    f_printf(&myFile, "%u", counter_Rpm);
    	f_printf(&myFile, "%c", ',');

    	logBuf[0] = {0,};
    	f_lseek(&myFile, f_size(&myFile));
    	loglen = sprintf(logBuf, "%f", efficiencyScore);
    	f_write(&myFile, logBuf, loglen, &myBytes);
 	    //f_printf(&myFile, "%f", efficiencyScore);
    	f_printf(&myFile, "%c", ',');

 	    f_printf(&myFile, "%u", counter_AccelX);
    	f_printf(&myFile, "%c", ',');

 	    f_printf(&myFile, "%u", counter_AccelYm);
    	f_printf(&myFile, "%c", ',');

 	    f_printf(&myFile, "%u", counter_AccelYm2);
    	f_printf(&myFile, "%c", ',');

 	    f_printf(&myFile, "%u", counter_Time);
    	f_printf(&myFile, "%c", ',');

 	    f_printf(&myFile, "%u", counter_KphRpm);
    	f_printf(&myFile, "%c", ',');

    	logBuf[0] = {0,};
    	f_lseek(&myFile, f_size(&myFile));
    	loglen = sprintf(logBuf, "%f", safetyScore);
    	f_write(&myFile, logBuf, loglen, &myBytes);
 	    //f_printf(&myFile, "%f", safetyScore);
    	f_printf(&myFile, "%c", ',');

    	logBuf[0] = {0,};
    	f_lseek(&myFile, f_size(&myFile));
    	loglen = sprintf(logBuf, "%f", drivingScore);
    	f_write(&myFile, logBuf, loglen, &myBytes);
 	    //f_printf(&myFile, "%f", drivingScore);
    	f_printf(&myFile, "%c", ',');

    	logBuf[0] = {0,};
    	f_lseek(&myFile, f_size(&myFile));
    	loglen = sprintf(logBuf, "%f", averagedrivingScore);
    	f_write(&myFile, logBuf, loglen, &myBytes);
 	    //f_printf(&myFile, "%f", averagedrivingScore);
    	f_printf(&myFile, "%c", ',');

    	logBuf[0] = {0,};
    	f_lseek(&myFile, f_size(&myFile));
    	loglen = sprintf(logBuf, "%.2f", f_pitch);
    	f_write(&myFile, logBuf, loglen, &myBytes);
 	    //f_printf(&myFile, "%f", f_pitch);
    	f_printf(&myFile, "%c", ',');

    	logBuf[0] = {0,};
    	f_lseek(&myFile, f_size(&myFile));
    	loglen = sprintf(logBuf, "%.2f", f_roll);
    	f_write(&myFile, logBuf, loglen, &myBytes);
 	    //f_printf(&myFile, "%f", f_roll);
    	f_printf(&myFile, "%c", ',');

    	logBuf[0] = {0,};
    	f_lseek(&myFile, f_size(&myFile));
    	loglen = sprintf(logBuf, "%.2f", f_accelX);
    	f_write(&myFile, logBuf, loglen, &myBytes);
 	    //f_printf(&myFile, "%f", f_accelX);
    	f_printf(&myFile, "%c", ',');

    	logBuf[0] = {0,};
    	f_lseek(&myFile, f_size(&myFile));
    	loglen = sprintf(logBuf, "%.2f", f_accelY);
    	f_write(&myFile, logBuf, loglen, &myBytes);
 	    //f_printf(&myFile, "%f", f_accelY);
    	f_printf(&myFile, "%c", ',');
    	f_printf(&myFile, "%s", "\n");

        f_close(&myFile);
    }
    else
    	printf("Failed to open %s \r\n", Filename);

    //if the file isn't open, pop up an error:
    /*else
    {
        Serial.println("error opening LOG_XXXX.txt");
        delay(100);
    }*/
}

void LogToSDcardError(int8_t curS)
{
	printf("ELM_ERROR LOGGED\r\n");

    e_time = millis() / 1000; //에러타임

    E_SEC = e_time % 60;
    E_MIN = (e_time / 60) % 60;
    E_HOUR = (e_time / (60 * 60)) % 24;

    // open the file. note that only one file can be open at a time,
    // so you have to close this one before opening another.
    fr = f_open(&myFile, Filename, FA_WRITE | FA_OPEN_APPEND );

  if (fr == FR_OK)
   {
   	f_printf(&myFile, "%d", E_HOUR);
   	f_printf(&myFile, "%s", "h_");
   	f_printf(&myFile, "%d", E_MIN);
   	f_printf(&myFile, "%s", "m_");
   	f_printf(&myFile, "%d", E_SEC);
   	f_printf(&myFile, "%s", "s");
  	f_printf(&myFile, "%c", ',');

     if (curS == ELM_NO_RESPONSE) {

     f_printf(&myFile, "%s", "ERROR: ELM_NO_RESPONSE");
     f_printf(&myFile, "%s", "\n"); }

     else if (curS == ELM_BUFFER_OVERFLOW) {

     f_printf(&myFile, "%s", "ERROR: ELM_BUFFER_OVERFLOW");
 	 f_printf(&myFile, "%s", "\n"); }

     else if (curS == ELM_UNABLE_TO_CONNECT) {

     f_printf(&myFile, "%s", "ERROR: ELM_UNABLE_TO_CONNECT");
     f_printf(&myFile, "%s", "\n"); }

     else if (curS == ELM_NO_DATA) {

     f_printf(&myFile, "%s", "ERROR: ELM_NO_DATA");
 	 f_printf(&myFile, "%s", "\n"); }

     else if (curS == ELM_STOPPED) {

     f_printf(&myFile, "%s", "ERROR: ELM_STOPPED");
 	 f_printf(&myFile, "%s", "\n"); }

     else if (curS == ELM_CANERROR) {

     f_printf(&myFile, "%s", "ERROR: ELM_CANERROR");
     f_printf(&myFile, "%s", "\n");
     }

     else if (curS == ELM_OUTOFMEM) {

     f_printf(&myFile, "%s", "ERROR: ELM_OUTOFMEM");
     f_printf(&myFile, "%s", "\n");
     }

     else if (curS == ELM_TIMEOUT) {

     f_printf(&myFile, "%s", "ERROR: ELM_TIMEOUT");
 	 f_printf(&myFile, "%s", "\n"); }

     else if (curS == ELM_TIMEOUT) {

     f_printf(&myFile, "%s", "ERROR: ELM_GENERAL_ERROR");
 	 f_printf(&myFile, "%s", "\n"); }

     f_close(&myFile);
         }
  else
  	printf("Failed to open %s \r\n", Filename);


}

void createNewfile()
{
    //f_close(&myFile);

	  for (unsigned int k = 0; k < 100000; k++)
	  {
		Filename[3] = k / 10000 + '0'; //만자리
		Filename[4] = ((k % 10000) / 1000) + '0';//천자리
		Filename[5] = ((k % 1000) / 100) + '0';//백자리
		Filename[6] = ((k % 100) / 10) + '0'; //십자리
		Filename[7] = k % 10 + '0'; //일자리

		fr = f_open(&myFile, Filename, FA_WRITE | FA_CREATE_NEW);
		if (fr == FR_OK)
		{
			break;
		}
	  }
    printf("Logging to: ");
    printf("%s", Filename);
    printf("<<---------------------------");
    printf("Created New File...\r\n");

       //dataFile.println("Time,ELM327_Status,RunTime,Rpm,Kph,EngineLoad,throttle,InstantFuelConsume,counter_AccelYp,counter_Fuel,counter_Rpm,efficiencyScore,counter_AccelX,counter_AccelYm,AccelYm2,counter_Time,counter_KphRpm,safetyScore,drivingScore,averagedrivingScore,f_pitch,f_roll,f_accelX,f_accelY");
    	f_printf(&myFile, "%s", "Time,ELM327_Status,RunTime,Rpm,Kph,EngineLoad,throttle,InstantFuelConsume,counter_AccelYp,counter_Fuel,counter_Rpm,efficiencyScore,counter_AccelX,counter_AccelYm,AccelYm2,counter_Time,counter_KphRpm,safetyScore,drivingScore,averagedrivingScore,f_pitch,f_roll,f_accelX,f_accelY");
    	f_printf(&myFile, "%s", "\n");
       //csv 엑셀 윗라인 항목 표시띠
        f_close(&myFile);
}

void createNewfile2()
{
	 //f_close(&myFile);
		  for (unsigned int k = 0; k < 100000; k++)
		  {
			Filename[3] = k / 10000 + '0'; //만자리
			Filename[4] = ((k % 10000) / 1000) + '0';//천자리
			Filename[5] = ((k % 1000) / 100) + '0';//백자리
			Filename[6] = ((k % 100) / 10) + '0'; //십자리
			Filename[7] = k % 10 + '0'; //일자리
			fr = f_open(&myFile, Filename, FA_WRITE | FA_CREATE_NEW);
			if (fr == FR_OK)
			{
				break;
			}
		  }
	    printf("Logging to: ");
	    printf("%s", Filename);
	    printf("<<---------------------------");
	    printf("Created New File...\r\n");

       //dataFile.println("Time,ELM327_Status,RunTime,Rpm,Kph,EngineLoad,throttle,commandedThrottleActuator,counter_AccelYp,counter_Throttle,counter_Rpm,efficiencyScore,counter_AccelX,counter_AccelYm,AccelYm2,counter_Time,counter_KphRpm,safetyScore,drivingScore,averagedrivingScore,f_pitch,f_roll,f_accelX,f_accelY");
       f_printf(&myFile, "%s", "Time,ELM327_Status,RunTime,Rpm,Kph,EngineLoad,throttle,commandedThrottleActuator,counter_AccelYp,counter_Throttle,counter_Rpm,efficiencyScore,counter_AccelX,counter_AccelYm,AccelYm2,counter_Time,counter_KphRpm,safetyScore,drivingScore,averagedrivingScore,f_pitch,f_roll,f_accelX,f_accelY");
       f_printf(&myFile, "%s", "\n");
       //csv 엑셀 윗라인 항목 표시띠
       f_close(&myFile);
}

/////////////////////////////////////////////Algorithm////////////////////////////////////////////////////////

//efficiency algorithm//

void accelYpAlgorithm() //forward acceleration algorithm
{
  const float limitAccelY = 0.4;
  //static float pre_f_accelY = 0.0;
  //static int counter_AccelYp = 0;

  if ( (f_accelY >= limitAccelY) && (pre_f_accelY < limitAccelY) )
  {
    counter_AccelYp++;
    printf("counter_AccelYp : ");
    printf("%lu\r\n", counter_AccelYp);
  }

}

void fuelAlgorithm() //fuel consumption algorithm
{
  const int limitFuel = 8; // km/l
  //static int counter_Fuel = 0;
  const int limitFuel_max = 12;
  //static float pre_instantFuelConsume = 0.0;

  averageFuelConsume = (float)((averageFuelConsume*(counter_N1-1)+instantFuelConsume)/counter_N1);

if (averageFuelConsume <= limitFuel_max)
{
  if ( (instantFuelConsume <= limitFuel) && (pre_instantFuelConsume > limitFuel) )
  {
    counter_Fuel++;
    printf("counter_Fuel : ");
    printf("%lu\r\n", counter_Fuel);
  }
  else if ( (instantFuelConsume >= limitFuel_max) && (pre_instantFuelConsume < limitFuel_max) )
  {
    counter_Fuel--;
    if (counter_Fuel <= 0 )
    {
      counter_Fuel = 0;
    printf("counter_Fuel : ");
    printf("%lu\r\n", counter_Fuel);
    }
  }
}
counter_N1++;
}

void commandedThrottleActuatorAlgorithm()
{
    //static int counter_Throttle = 0;

    const int limit_Throttle_max = 65;
    const int limit_Throttle = 20;
    const int limit_Throttle_min = 10;

     averagecommandedThrottleActuator = (float)((averagecommandedThrottleActuator*(counter_N2-1)+commandedThrottleActuator)/counter_N2);

if (averagecommandedThrottleActuator >= limit_Throttle_min) {
  if ( (commandedThrottleActuator >= limit_Throttle) && (pre_commandedThrottleActuator < limit_Throttle) )
  {
    counter_Throttle++;
    printf("counter_Throttle : ");
    printf("%lu\r\n", counter_Throttle);
  }
    if ( (commandedThrottleActuator >= limit_Throttle_max) && (pre_commandedThrottleActuator < limit_Throttle_max) )
    {
    counter_Throttle++;
    printf("counter_Throttle+ : ");
    printf("%lu\r\n", counter_Throttle);
    }
    if ( (commandedThrottleActuator > limit_Throttle_max) && (pre_commandedThrottleActuator > limit_Throttle) )
    {
    counter_Throttle++;
    printf("counter_Throttle+ : ");
    printf("%lu\r\n", counter_Throttle);
    }
    if ( (commandedThrottleActuator <= limit_Throttle_min) && (pre_commandedThrottleActuator > limit_Throttle_min) )
  {
    counter_Throttle--;
    if (counter_Throttle <= 0 )
    {
      counter_Throttle = 0;
    printf("counter_Throttle : ");
    printf("%lu\r\n", counter_Throttle);
    }
  }
}

//
//if (averagecommandedThrottleActuator < limit_Throttle_min)
//{
//    counter_Throttle = 0;
//    printf("counter_Throttle Zeroed ");
//    printf("%lu\r\n", counter_Throttle);
//}

if (counter_Idle >= 30)
{
  counter_Throttle++;
  counter_Idle = 0;
  printf("Idle counter_Throttle+ : ");
  printf("%lu\r\n", counter_Throttle);
}
counter_N2++;
}

void rpmAlgorithm() //enginespeed algorithm
{
  const int limitRpm = 2100;
  const int limitRpm_max = 4000;
  //static uint32_t pre_rpm = 0.0;
  //static int counter_Rpm = 0;

  if ( (rpm >= limitRpm) && (pre_rpm < limitRpm) )
  {
    counter_Rpm++;
    printf("counter_Rpm : ");
    printf("%lu\r\n", counter_Rpm);
  }

  if ( ( (rpm >= limitRpm_max) && (pre_rpm >= limitRpm_max) ) || ( (rpm >= limitRpm_max) && (pre_rpm < limitRpm_max) ) )
  {
    counter_Rpm++;
    printf("counter_Rpm+ : ");
    printf("%lu\r\n", counter_Rpm);
  }
}

void efficiencyScoreAlgorithm() //algorithm with fuel limit condition efficiency formulas
{
  //static float efficiencyScore = 0.0;
  const int limitFuel_max = 12;

  if (averageFuelConsume < limitFuel_max)
  {
    efficiencyScore = (float)(5.0-(40*((float)counter_AccelYp)/runTime)-(30*((float)counter_Rpm)/runTime));
    printf("efficiencyScore : ");
    printf("%f", efficiencyScore);
    printf(" / 10\r\n");
  }
  else if (averageFuelConsume >= limitFuel_max)
  {
    efficiencyScore = (float)(10.0-(40*((float)counter_AccelYp)/runTime)-(180*((float)counter_Fuel)/runTime)-(30*((float)counter_Rpm)/runTime));
    printf("efficiencyScore : ");
    printf("%f", efficiencyScore);
    printf(" / 10\r\n");
  }
  if (efficiencyScore <= 0)
  efficiencyScore = 0.0;
  if (isnan(efficiencyScore))
  efficiencyScore = 5.0;

  printf("averageFuelConsume : ");
  printf("%f\r\n", averageFuelConsume);
}

void efficiencyScoreAlgorithm2() //algorithm with fuel limit condition efficiency formulas
{
  //static float efficiencyScore = 0.0;
  const int limit_Throttle_min = 10;

  if (averagecommandedThrottleActuator >= limit_Throttle_min)
  {
    efficiencyScore = (float)(5.0-(40*((float)counter_AccelYp)/runTime)-(30*((float)counter_Rpm)/runTime));
    printf("efficiencyScore : ");
    printf("%f", efficiencyScore);
    printf(" / 10\r\n");
  }
  else if (averagecommandedThrottleActuator < limit_Throttle_min)
  {
    efficiencyScore = (float)(10.0-(50*((float)counter_AccelYp)/runTime)-(180*((float)counter_Throttle)/runTime)-(30*((float)counter_Rpm)/runTime));
    printf("efficiencyScore : ");
    printf("%f", efficiencyScore);
    printf(" / 10\r\n");
  }
  if (efficiencyScore <= 0)
  efficiencyScore = 0.0;
  if (isnan(efficiencyScore))
  efficiencyScore = 5.0;

  printf("averagecommandedThrottleActuator : ");
  printf("%f\r\n", averagecommandedThrottleActuator);
}

//safety algorithm//

void accelXAlgorithm() //lateral acceleration algorithm
{
  const float limitAccelXp = 0.4;
  const float limitAccelXm = -0.4;
  //static float pre_f_accelX = 0.0;
  //static int counter_AccelX = 0;

  if ( (f_accelX >= limitAccelXp) && (pre_f_accelX < limitAccelXp))
  {
    counter_AccelX++;
    printf("counter_AccelX : ");
    printf("%lu\r\n", counter_AccelX);
  }
  else if ( (f_accelX <= limitAccelXm) && (pre_f_accelX > limitAccelXm))
  {
    counter_AccelX++;
    printf("counter_AccelX : ");
    printf("%lu\r\n", counter_AccelX);
  }

}

void accelYmAlgorithm() //dangerous acceleration algorithm
{
  const float limitAccelYm = -0.4;
  //static float pre_f_accelY = 0;
  //static int counter_AccelYm = 0;

if ( (f_accelY <= limitAccelYm) && (pre_f_accelY > limitAccelYm) )
{
    counter_AccelYm++;
    printf("counter_AccelYm : ");
    printf("%lu\r\n", counter_AccelYm);
}
}

void accelYmAlgorithm2() // negative forward acceleration algorithm
{
  const float limitAccelYm = -0.3;
  //static float pre_f_accelY = 0.0;
  //static int counter_AccelYm2 = 0;

  if ( (f_accelY <= limitAccelYm) && (pre_f_accelY > limitAccelYm) )
  {
    counter_AccelYm2++;
    printf("counter_AccelYm2 : ");
    printf("%lu\r\n", counter_AccelYm2);
  }
}

void timeAlgorithm() //time algorithm
{
  const int limitTime = 7200; //2hr (7200sec)
  //static uint16_t pre_runTime = 0;
  //static int counter_Time = 0;

  if ( (runTime >= limitTime) && (pre_runTime < limitTime) )
  {
    counter_Time = 1;
    printf("counter_Time : ");
    printf("%lu\r\n", counter_Time);
  }
}

void kphrpmAlgorithm() //vehicle Speed & Rpm algorithm
{
  const int limitKph = 51;
  const int limitKph_max = 135;
  //const int limitRpm = 2700;
  const int limitRpm_max = 4000;
  const int limitRpm_max2 = 5500;

  //static int32_t pre_kph = 0;
  //static int counter_KphRpm = 0;

  if ( ( (kph >= limitKph) && (pre_kph < limitKph) ) || /*( (kph >= limitKph_max) && (pre_kph >= limitKph_max) ) ||*/ ( (kph >= limitKph_max) && (pre_kph < limitKph_max) ) )
  {
    counter_KphRpm++;
    printf("counter_KphRpm : ");
    printf("%lu\r\n", counter_KphRpm);
  }

  if ( ( (rpm >= limitRpm_max2) && (pre_rpm < limitRpm_max2) ) || ( (rpm >= limitRpm_max) && (pre_rpm < limitRpm_max) ) /*|| ( (rpm >= limitRpm) && (pre_rpm < limitRpm) )*/ )
  {
      counter_KphRpm++;
      printf("counter_KphRpm+ : ");
      printf("%lu\r\n", counter_KphRpm);
  }
}

void safetyScoreAlgorithm() //algorithm with time condition formulas
{
  //static float safetyScore = 0.0;

  if (counter_Time == 0)
  {
    safetyScore = (float)(10.0-(200*((float)counter_AccelYm)/runTime)-(80*((float)counter_KphRpm)/runTime)-(50*((float)counter_AccelYm2)/runTime)-(30*((float)counter_AccelX)/runTime));
    printf("safetyScore : ");
    printf("%f", safetyScore);
    printf(" / 10\r\n");
  }
  else if (counter_Time > 0)
  {
    safetyScore = (float)(9.0-(200*((float)counter_AccelYm)/runTime)-(80*((float)counter_KphRpm)/runTime)-(50*((float)counter_AccelYm2)/runTime)-(30*((float)counter_AccelX)/runTime));
    printf("safetyScore : ");
    printf("%f", safetyScore);
    printf(" / 10\r\n");
  }

  if (safetyScore <= 0)
  safetyScore = 0.0;
  if (isnan(safetyScore))
  safetyScore = 8.0;

  printf("Counter_Time : ");
  printf("%lu\r\n", counter_Time);
}

void drivingScoreAlgorithm() //global algorithm
{
  //static float drivingScore = 10.0;

  drivingScore = ((safetyScore+efficiencyScore) / 2.0);
  char safetyScoreBuf[30] = "";

  if (drivingScore <= 2)
  {
    strcpy(safetyScoreBuf, "verybad");
    printf("safetyScoreBuf : ");
    printf("%s\r\n", safetyScoreBuf);
    printf("drivingScore : ");
    printf("%f", drivingScore);
    printf(" / 10\r\n");
  }

  else if ( (drivingScore <= 4) && (drivingScore > 2) )
  {
    strcpy(safetyScoreBuf, "bad");
    printf("safetyScoreBuf : ");
    printf("%s\r\n", safetyScoreBuf);
    printf("drivingScore : ");
    printf("%f", drivingScore);
    printf(" / 10\r\n");
  }

  else if ( (drivingScore <= 6) && (drivingScore > 4) )
  {
    strcpy(safetyScoreBuf, "normal");
    printf("safetyScoreBuf : ");
    printf("%s\r\n", safetyScoreBuf);
    printf("drivingScore : ");
    printf("%f", drivingScore);
    printf(" / 10\r\n");
  }

  else if ( (drivingScore <= 8) && (drivingScore > 6) )
  {
    strcpy(safetyScoreBuf, "good");
    printf("safetyScoreBuf : ");
    printf("%s\r\n", safetyScoreBuf);
    printf("drivingScore : ");
    printf("%f", drivingScore);
    printf(" / 10\r\n");
  }

  else if (drivingScore > 8)
  {
    strcpy(safetyScoreBuf, "verygood");
    printf("safetyScoreBuf : ");
    printf("%s\r\n", safetyScoreBuf);
    printf("drivingScore : ");
    printf("%f", drivingScore);
    printf(" / 10\r\n");
  }

}

void averagedrivingScoreAlgorithm()
{
  //static float averagedrivingScore = 10.0;
  //if (drivingScore >= 0)

  averagedrivingScore = (float)((averagedrivingScore*(counter_N3-1)+drivingScore)/counter_N3);
  if (isnan(averagedrivingScore))
  {averagedrivingScore = 7.5;}

  printf("averagedrivingScore : ");
  printf("%f", averagedrivingScore);
  printf(" / 10\r\n");

  counter_N3++;
}

void pre_DataUpdate()
{
    pre_f_accelX = f_accelX;
    pre_f_accelY = f_accelY;
    pre_rpm = rpm;
    pre_runTime = runTime;
    pre_kph = kph;
    if (mafRatecheck == true)
    {
      pre_instantFuelConsume = instantFuelConsume;
      printf("pre_instantFuelConsume = ");
      printf("%f\r\n", pre_instantFuelConsume);
      }
    else if (mafRatecheck == false)
    {
      pre_commandedThrottleActuator = commandedThrottleActuator;
      printf("pre_commandedThrottleActuator : ");
      printf("%f\r\n", pre_commandedThrottleActuator);
    }

    printf("pre_f_accelX = ");
    printf("%f\r\n", pre_f_accelX);
    printf("pre_f_accelY = ");
    printf("%f\r\n", pre_f_accelY);
    printf("pre_rpm = ");
    printf("%lu\r\n", pre_rpm);
    printf("pre_runTime = ");
    printf("%lu\r\n", pre_runTime);
    printf("pre_kph = ");
    printf("%ld\r\n", pre_kph);


    printf("pre_Data Updated\r\n");
}

void counterReset()
{
  counter_AccelYp = 0;
  counter_Fuel = 0;
  counter_Throttle = 0;
  counter_Rpm = 0;
  efficiencyScore = 0.0;
  counter_AccelX = 0;
  counter_AccelYm = 0;
  counter_AccelYm2 = 0;
  counter_Time = 0;
  counter_KphRpm = 0;
  safetyScore = 0.0;
  drivingScore = 7.5;
  averagedrivingScore = 10.0;
  counter_N1 = 1;
  counter_N2 = 1;
  counter_N3 = 1;

  averageFuelConsume= 0.0;
  averagecommandedThrottleActuator = 0.0;

  printf("Counter Reset\r\n");
}

bool errorValueReset()
{
  errorvaluecheck = false;
   if ( (rpm == -1) || (engineLoad == -1) || (kph == -1) || (runTime == 65535) )
   {
    runTime = pre_runTime;
    rpm = 700;
    kph = 1;
    engineLoad = 19;
    oilTemp = 85;
    engineCoolantTemp = 85;
    relativePedalPos = 10;
    throttle = 13;
    relativeThrottle = 3;
    commandedThrottleActuator = 5.0;
    intakeAirTemp = 15;
    manifoldPressure = 32;
    mafRate = 2.01;
    //fuelLevel = 50;
    ctrlModVoltage = 14;
    distTravelWithMIL = 0;
    ambientAirTemp = 15;

    errorvaluecheck = true;
   }
   return errorvaluecheck;
}

bool mafRateCheck(float mafRate_value)
{

  if (mafRate_value <= 0)
	  mafRatecheck = false;

  else if (mafRate_value > 0)
	  mafRatecheck = true;

  return mafRatecheck;
}

bool Uart6_timeout()
{
	Uart6_currentTime = millis();
	if ((Uart6_currentTime - Uart6_previousTime) >= Uart6_timeout_ms)
		return true;
	return false;
}

void flushInputBuff()
{
    unsigned char ch;

    while (IsDataAvailable())
    {
        ch = Uart_read();
    }
    // sekim 20200514 Remove Warning
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

