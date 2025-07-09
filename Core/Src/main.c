/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "string.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "mongoose_glue.h"
#include "mongoose.h"
#include "frozen.h"
#include "jsmn.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

typedef struct {
    uint16_t status_bits[4];     // UINT[0], UINT[1], UINT[2], UINT[3] - 64 bits
    uint16_t data_words[106];
} PLC_Snapshot;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define API_URL "http://192.168.1.233:8080/api/stats?start=-1h&stop=now()"
#define MAX_RESPONSE_SIZE 4096
#define MAX_EXTRACTED_NUMBERS 64
#define MAX_PAYLOAD_SIZE 512
#define MAX_NDEF_SIZE 540
#define MAX_NUMBERS 64
#define data "{\"boolean_percentages\":{\"FeederStatusBits.AirTrackBlowerEnabled\":53.49740932642487,\"FeederStatusBits.BulkElevatorConveyorEnabledFWD\":53.49740932642487,\"FeederStatusBits.BulkElevatorConveyorEnabledREV\":0,\"FeederStatusBits.BulkElevatorLowLevel\":0,\"FeederStatusBits.OrientationSectionDriveEnabled\":0,\"FeederStatusBits.Spare\":0,\"FeederStatusBits.StepperEnabled\":0,\"LevelStatusBits.HighLevel.Lane1\":46.50259067357513,\"LevelStatusBits.HighLevel.Lane2\":53.43709468223087,\"LevelStatusBits.HighLevel.Lane3\":0,\"LevelStatusBits.HighLevel.Lane4\":0,\"LevelStatusBits.HighLevel.Lane5\":0,\"LevelStatusBits.HighLevel.Lane6\":0,\"LevelStatusBits.HighLevel.Lane7\":0,\"LevelStatusBits.HighLevel.Lane8\":0,\"LevelStatusBits.NotAtLowLevel.Lane1\":46.50259067357513,\"LevelStatusBits.NotAtLowLevel.Lane2\":53.43709468223087,\"LevelStatusBits.NotAtLowLevel.Lane3\":0,\"LevelStatusBits.NotAtLowLevel.Lane4\":0,\"LevelStatusBits.NotAtLowLevel.Lane5\":0,\"LevelStatusBits.NotAtLowLevel.Lane6\":0,\"LevelStatusBits.NotAtLowLevel.Lane7\":0,\"LevelStatusBits.NotAtLowLevel.Lane8\":0,\"SystemStatusBits.AirPressureOk\":100,\"SystemStatusBits.AutoMode\":100,\"SystemStatusBits.ControlPowerOn\":100,\"SystemStatusBits.PurgeMode\":0,\"SystemStatusBits.SystemFaulted\":0,\"SystemStatusBits.SystemIdle\":0},\"fault_counts\":{\"FaultBits.AirPressureNotOkFault\":0,\"FaultBits.AirTrackBlowerFault\":0,\"FaultBits.BulkElevatorConveyorFault\":0,\"FaultBits.JamInOrientation.Lane1\":332,\"FaultBits.JamInOrientation.Lane2\":0,\"FaultBits.JamInOrientation.Lane3\":332,\"FaultBits.JamInOrientation.Lane4\":0,\"FaultBits.JamInOrientation.Lane5\":0,\"FaultBits.JamInOrientation.Lane6\":0,\"FaultBits.JamInOrientation.Lane7\":0,\"FaultBits.JamInOrientation.Lane8\":0,\"FaultBits.JamInStorage.Lane1\":0,\"FaultBits.JamInStorage.Lane2\":0,\"FaultBits.JamInStorage.Lane3\":0,\"FaultBits.JamInStorage.Lane4\":0,\"FaultBits.JamInStorage.Lane5\":0,\"FaultBits.JamInStorage.Lane6\":0,\"FaultBits.JamInStorage.Lane7\":0,\"FaultBits.JamInStorage.Lane8\":0,\"FaultBits.OrientationSectionDriveFault\":386,\"FaultBits.PlcToPlcFault\":61,\"FaultBits.StepperFault\":0},\"float_averages\":{\"Floats.AirTrackBlower.Speed\":198.66642381569204,\"Floats.OrientationSectionDrive.Temperature\":198.6398539339026,\"Floats.OrientationSectionDrive.VibrationX\":198.49442135066167,\"Floats.OrientationSectionDrive.VibrationY\":198.49728547238945,\"Floats.OrientationSectionDrive.VibrationZ\":801.4288238665528,\"Floats.Performance.PartsPerMinute\":40.16731803510442}}"

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
#if defined ( __ICCARM__ ) /*!< IAR Compiler */
#pragma location=0x30000000
ETH_DMADescTypeDef  DMARxDscrTab[ETH_RX_DESC_CNT]; /* Ethernet Rx DMA Descriptors */
#pragma location=0x30000080
ETH_DMADescTypeDef  DMATxDscrTab[ETH_TX_DESC_CNT]; /* Ethernet Tx DMA Descriptors */

#elif defined ( __CC_ARM )  /* MDK ARM Compiler */

__attribute__((at(0x30000000))) ETH_DMADescTypeDef  DMARxDscrTab[ETH_RX_DESC_CNT]; /* Ethernet Rx DMA Descriptors */
__attribute__((at(0x30000080))) ETH_DMADescTypeDef  DMATxDscrTab[ETH_TX_DESC_CNT]; /* Ethernet Tx DMA Descriptors */

#elif defined ( __GNUC__ ) /* GNU Compiler */

ETH_DMADescTypeDef DMARxDscrTab[ETH_RX_DESC_CNT] __attribute__((section(".RxDescripSection"))); /* Ethernet Rx DMA Descriptors */
ETH_DMADescTypeDef DMATxDscrTab[ETH_TX_DESC_CNT] __attribute__((section(".TxDescripSection")));   /* Ethernet Tx DMA Descriptors */
#endif

ETH_TxPacketConfig TxConfig;

ETH_HandleTypeDef heth;

I2C_HandleTypeDef hi2c1;

RNG_HandleTypeDef hrng;

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
static char response_buf[MAX_RESPONSE_SIZE];
static size_t response_len = 0;
static bool request_done = false;
float extracted_numbers[MAX_EXTRACTED_NUMBERS];
int number_count = 0;

#define numbercount 0
#define MAX_VALUES 64

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ETH_Init(void);
static void MX_RNG_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
bool mg_random(void *buf, size_t len) {  // Use on-board RNG
  for (size_t n = 0; n < len; n += sizeof(uint32_t)) {
    uint32_t r;
    HAL_RNG_GenerateRandomNumber(&hrng, &r);
    memcpy((char *) buf + n, &r, n + sizeof(r) > len ? len - n : sizeof(r));
  }
  return true; // TODO(): ensure successful RNG init, then return on false above
}

uint64_t mg_millis(void) {
  return HAL_GetTick();
}

int _write(int fd, unsigned char *buf, int len) {
  if (fd == 1 || fd == 2) {                     // stdout or stderr ?
    HAL_UART_Transmit(&huart3, buf, len, 999);  // Print to the UART
  }
  return len;
}

static void http_event_handler(struct mg_connection *c, int ev, void *ev_data, void *fn_data) {
	if (ev == MG_EV_HTTP_MSG) {
    struct mg_http_message *hm = (struct mg_http_message *) ev_data;
    size_t copy_len = hm->body.len < MAX_RESPONSE_SIZE - 1 ? hm->body.len : MAX_RESPONSE_SIZE - 1;
    memcpy(response_buf, hm->body.buf, copy_len);
    response_buf[copy_len] = '\0';
    response_len = copy_len;
    printf("HTTP Response: %.*s\r\n", (int) response_len, response_buf);
    request_done = true;
    c->is_closing = 1;
  } else if (ev == MG_EV_CLOSE && !request_done) {
    printf("HTTP request failed or connection closed early\r\n");
  }
}


void perform_http_data_read(void) {
  request_done = false;

  // Use the global manager directly
  struct mg_connection *c = mg_http_connect(&g_mgr, API_URL, http_event_handler, NULL);
  if (c == NULL) {
    printf("HTTP connect failed\r\n");
    return;
  }

  // Send GET request
  mg_printf(c, "GET %s HTTP/1.0\r\nHost: 192.168.1.233\r\n\r\n", "/api/stats?start=-1h&stop=now()");

  // Poll until request is done or timeout hits
  uint32_t start = HAL_GetTick();
  while (!request_done && HAL_GetTick() - start < 5000) {
    mg_mgr_poll(&g_mgr, 1);  // Poll for events
    HAL_Delay(1);            // Give other threads (like LWIP) time to breathe
  }

  if (!request_done) {
    printf("Request timed out\r\n");
  }
}


void wait_for_network_ready(void) {
  uint32_t start = HAL_GetTick();
  while (HAL_GetTick() - start < 500) {
    mongoose_poll();   // gives time for ARP retry and TCP/IP setup
    HAL_Delay(1);
  }
}

void collect_plc_data(PLC_Snapshot *snapshot) {
    uint16_t val;

    // Read status_bits[0..3] from registers 1000–1003
    for (int i = 0; i < 4; i++) {
        glue_modbus_read_reg(1000 + i, &val);
        snapshot->status_bits[i] = val;
        MG_INFO(("status_bits[%d] (reg %d) = %04X", i, 1000 + i, val));
    }

    // Read data_words[0..105] from registers 1004–1109
    for (int i = 0; i < 106; i++) {
        glue_modbus_read_reg(1004 + i, &val);
        snapshot->data_words[i] = val;
        if (i < 20 || i > 100) {  // Limit log volume
            MG_INFO(("data_words[%d] (reg %d) = %04X", i, 1004 + i, val));
        }
    }

    MG_INFO(("Snapshot struct size: %d", (int) sizeof(PLC_Snapshot)));
}



void write_snapshot_to_nfc(void) {
    uint16_t i2c_addr = 0x53 << 1;        // 0xA6
    uint16_t mem_addr = 0x0008;           // NDEF message start offset
    HAL_StatusTypeDef status;

    PLC_Snapshot snapshot;
    collect_plc_data(&snapshot); // Assume this populates the snapshot struct

    printf("Snapshot size: %u\n", (unsigned int) sizeof(snapshot));
    printf("Float count: %d\n", number_count);

    if (number_count == 0 || number_count > 64) {
        printf("Invalid float count: %d\n", number_count);
        return;
    }

    // Prepare combined payload: [snapshot][count][floats]
    size_t snapshot_size = sizeof(snapshot);
    size_t float_bytes = 1 + number_count * sizeof(float);
    size_t payload_length = snapshot_size + float_bytes;

    uint8_t payload[MAX_PAYLOAD_SIZE];
    if (payload_length > MAX_PAYLOAD_SIZE) {
        printf("Error: Payload is too large for the buffer!\n");
        return;
    }

    memcpy(payload, &snapshot, snapshot_size);
    payload[snapshot_size] = (uint8_t) number_count;
    memcpy(&payload[snapshot_size + 1], extracted_numbers, number_count * sizeof(float));

    const char* mime_type = "application/octet-stream";
    uint8_t type_length = strlen(mime_type);
    uint8_t ndef_header_size = 1 + 1 + 4;
    uint32_t ndef_record_length = ndef_header_size + type_length + payload_length;

    uint8_t ndef_buffer[MAX_NDEF_SIZE];
    uint16_t total_ndef_message_size = 0;

    ndef_buffer[total_ndef_message_size++] = 0x03;  // TLV type

    if (ndef_record_length < 0xFF) {
        ndef_buffer[total_ndef_message_size++] = (uint8_t) ndef_record_length;
    } else {
        ndef_buffer[total_ndef_message_size++] = 0xFF;
        ndef_buffer[total_ndef_message_size++] = (ndef_record_length >> 8) & 0xFF;
        ndef_buffer[total_ndef_message_size++] = ndef_record_length & 0xFF;
    }

    ndef_buffer[total_ndef_message_size++] = 0xC2; // MB=1, ME=1, SR=0, TNF=MIME
    ndef_buffer[total_ndef_message_size++] = type_length;

    ndef_buffer[total_ndef_message_size++] = (payload_length >> 24) & 0xFF;
    ndef_buffer[total_ndef_message_size++] = (payload_length >> 16) & 0xFF;
    ndef_buffer[total_ndef_message_size++] = (payload_length >> 8) & 0xFF;
    ndef_buffer[total_ndef_message_size++] = payload_length & 0xFF;

    memcpy(&ndef_buffer[total_ndef_message_size], mime_type, type_length);
    total_ndef_message_size += type_length;

    memcpy(&ndef_buffer[total_ndef_message_size], payload, payload_length);
    total_ndef_message_size += payload_length;

    ndef_buffer[total_ndef_message_size++] = 0xFE;

    printf("Payload + floats total: %lu\n", (unsigned long) payload_length);
    printf("NDEF Record length: %u\n", ndef_record_length);
    printf("Total NDEF Message to Write (including TLV and Terminator): %u bytes\n", total_ndef_message_size);

    printf("NFC TLV Dump (%d bytes):\n", total_ndef_message_size);
    for (int j = 0; j < total_ndef_message_size; j++) {
        if (j % 16 == 0) printf("\n%04X  ", j);
        printf("%02X ", ndef_buffer[j]);
    }
    printf("\n");

    // Chunked write to NFC
    const uint16_t MAX_CHUNK_SIZE = 128;
    uint16_t bytes_remaining = total_ndef_message_size;
    uint16_t offset = 0;

    while (bytes_remaining > 0) {
        uint16_t chunk_size = (bytes_remaining > MAX_CHUNK_SIZE) ? MAX_CHUNK_SIZE : bytes_remaining;

        status = HAL_I2C_Mem_Write(&hi2c1, i2c_addr, mem_addr + offset,
                                   I2C_MEMADD_SIZE_16BIT,
                                   &ndef_buffer[offset], chunk_size,
                                   HAL_MAX_DELAY);

        if (status != HAL_OK) {
            printf("I2C chunk write failed at offset %d, status: %d\n", offset, status);
            return;
        }
        HAL_Delay(100);
        offset += chunk_size;
        bytes_remaining -= chunk_size;
    }

    printf("I2C chunked write successful.\n");
}






double mg_str_to_d(struct mg_str s) {
    char buf[64];
    size_t n = s.len < sizeof(buf) - 1 ? s.len : sizeof(buf) - 1;
    memcpy(buf, s.buf, n);
    buf[n] = '\0';
    return atof(buf);  // Or strtod(buf, NULL) for more control
}


void parse_numbers_recursive(struct mg_str json, float *dest_array, int max_count, int *out_count) {
    size_t offset = 0;
    struct mg_str key, val;

    while ((offset = mg_json_next(json, offset, &key, &val)) != 0) {
        // Check if it's a number by trying to convert it
        if (val.len > 0 && (isdigit(val.buf[0]) || val.buf[0] == '-' || val.buf[0] == '+')) {
            double num = mg_str_to_d(val);  // ✅ Convert raw mg_str to double
            if (*out_count < max_count) {
                dest_array[*out_count] = (float) num;
                (*out_count)++;
            }
        } else if (val.len > 0 && val.buf[0] == '{') {
            // Recurse into nested object
            parse_numbers_recursive(val, dest_array, max_count, out_count);
        }
    }
}


void parse_and_store_numbers(const char *json_data, size_t len,
                             float *dest_array, int max_count, int *out_count) {
    struct mg_str json = mg_str_n(json_data, len);
    *out_count = 0;
    parse_numbers_recursive(json, dest_array, max_count, out_count);

    if (*out_count == 0) {
        printf("No numbers found in JSON.\n");
    } else if (*out_count >= max_count) {
        printf("Warning: hit max count, some values may be skipped.\n");
    }
}

static int is_numeric_token(const char *json, jsmntok_t *tok) {
    for (int i = tok->start; i < tok->end; i++) {
        if ((json[i] >= '0' && json[i] <= '9') || json[i] == '.' || json[i] == '-') continue;
        return 0;
    }
    return 1;
}

void extract_numbers(const char *json) {
    jsmn_parser p;
    jsmntok_t tokens[512];
    jsmn_init(&p);
    int r = jsmn_parse(&p, json, strlen(json), tokens, sizeof(tokens)/sizeof(tokens[0]));

    if (r < 0) {
        printf("Failed to parse JSON: %d\n", r);
        return;
    }

    number_count = 0;

    for (int i = 1; i < r && number_count < MAX_NUMBERS; i++) {
        if (tokens[i].type == JSMN_PRIMITIVE && is_numeric_token(json, &tokens[i])) {
            char temp[32];
            int len = tokens[i].end - tokens[i].start;
            if (len < sizeof(temp)) {
                memcpy(temp, json + tokens[i].start, len);
                temp[len] = '\0';
                extracted_numbers[number_count++] = strtof(temp, NULL);
            }
        }
    }


     printf("Extracted %d numbers:\r\n", number_count);
     for (int i = 0; i < number_count; i++) {
         printf("  [%2d] %.6f\r\n", i, extracted_numbers[i]);
     }
//     printf("[");
//    for (int i = 0; i < number_count; i++) {
//        printf("%f", extracted_numbers[i]);
//        if (i < number_count - 1) printf(", ");
//    }
//    printf("]\n");

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
  MX_ETH_Init();
  MX_RNG_Init();
  MX_USART3_UART_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  mongoose_init();
  uint32_t lastWriteTime = 0;
  uint32_t lastHttpPollTime = 0;
  for (;;) {
    mongoose_poll();

    uint32_t now = HAL_GetTick();
    if (now - lastWriteTime >= 10000) {
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET); // Turn ON LED to indicate write
      uint32_t start = HAL_GetTick();
      write_snapshot_to_nfc(); // Takes ~800ms

      uint32_t duration = HAL_GetTick() - start;
      printf("NFC write duration: %lu ms\r\n", duration);

      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET); // Turn OFF LED after write
      lastWriteTime = now;
    }

    if(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13)){
    	HAL_Delay(500);
    	glue_update_state();
        wait_for_network_ready();
    	perform_http_data_read();
    	HAL_Delay(1000);
    	parse_and_store_numbers(response_buf, strlen(response_buf), extracted_numbers, MAX_EXTRACTED_NUMBERS, &number_count);
    	printf("Parsed %d numbers:\n", number_count);
    	for (int i = 0; i < number_count; i++) {
    	    printf("Value %d: %.3f\r\n", i, extracted_numbers[i]);
    	}

//    	printf("%s\r\n\r\n", data);
//    	HAL_Delay(500);
//    	extract_numbers(data);

    }
  }

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

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48|RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = 64;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 12;
  RCC_OscInitStruct.PLL.PLLP = 1;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ETH Initialization Function
  * @param None
  * @retval None
  */
static void MX_ETH_Init(void)
{

  /* USER CODE BEGIN ETH_Init 0 */

  /* USER CODE END ETH_Init 0 */

   static uint8_t MACAddr[6];

  /* USER CODE BEGIN ETH_Init 1 */

  /* USER CODE END ETH_Init 1 */
  heth.Instance = ETH;
  MACAddr[0] = 0x00;
  MACAddr[1] = 0x80;
  MACAddr[2] = 0xE1;
  MACAddr[3] = 0x00;
  MACAddr[4] = 0x00;
  MACAddr[5] = 0x00;
  heth.Init.MACAddr = &MACAddr[0];
  heth.Init.MediaInterface = HAL_ETH_RMII_MODE;
  heth.Init.TxDesc = DMATxDscrTab;
  heth.Init.RxDesc = DMARxDscrTab;
  heth.Init.RxBuffLen = 1536;

  /* USER CODE BEGIN MACADDRESS */

  /* USER CODE END MACADDRESS */

  if (HAL_ETH_Init(&heth) != HAL_OK)
  {
    Error_Handler();
  }

  memset(&TxConfig, 0 , sizeof(ETH_TxPacketConfig));
  TxConfig.Attributes = ETH_TX_PACKETS_FEATURES_CSUM | ETH_TX_PACKETS_FEATURES_CRCPAD;
  TxConfig.ChecksumCtrl = ETH_CHECKSUM_IPHDR_PAYLOAD_INSERT_PHDR_CALC;
  TxConfig.CRCPadCtrl = ETH_CRC_PAD_INSERT;
  /* USER CODE BEGIN ETH_Init 2 */

  /* USER CODE END ETH_Init 2 */

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
  hi2c1.Init.Timing = 0x20303E5D;
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
  * @brief RNG Initialization Function
  * @param None
  * @retval None
  */
static void MX_RNG_Init(void)
{

  /* USER CODE BEGIN RNG_Init 0 */

  /* USER CODE END RNG_Init 0 */

  /* USER CODE BEGIN RNG_Init 1 */

  /* USER CODE END RNG_Init 1 */
  hrng.Instance = RNG;
  hrng.Init.ClockErrorDetection = RNG_CED_ENABLE;
  if (HAL_RNG_Init(&hrng) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RNG_Init 2 */

  /* USER CODE END RNG_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart3, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart3, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LED_GREEN_Pin|LED_RED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(USB_FS_PWR_EN_GPIO_Port, USB_FS_PWR_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_YELLOW_GPIO_Port, LED_YELLOW_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_GREEN_Pin LED_RED_Pin */
  GPIO_InitStruct.Pin = LED_GREEN_Pin|LED_RED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_FS_PWR_EN_Pin */
  GPIO_InitStruct.Pin = USB_FS_PWR_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(USB_FS_PWR_EN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_FS_OVCR_Pin */
  GPIO_InitStruct.Pin = USB_FS_OVCR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_FS_OVCR_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_FS_ID_Pin */
  GPIO_InitStruct.Pin = USB_FS_ID_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG1_HS;
  HAL_GPIO_Init(USB_FS_ID_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_YELLOW_Pin */
  GPIO_InitStruct.Pin = LED_YELLOW_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_YELLOW_GPIO_Port, &GPIO_InitStruct);

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
