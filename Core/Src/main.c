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
#include <stdlib.h>
#include <stdio.h>
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
#define API_URL "http://192.168.1.133:8080/api/stats?start=-1h&stop=now()"
#define MAX_RESPONSE_SIZE 100000
#define MAX_EXTRACTED_NUMBERS 256
#define MAX_PAYLOAD_SIZE 512
#define MAX_NDEF_SIZE 8000
#define MAX_NUMBERS 56
#define MAX_VALUES 1024
#define OUTPUT_BUF_SIZE 6000

typedef struct {
  char start_time[32];
  int interval_sec;
  float values[MAX_VALUES];
  int count;
} ParsedData;

typedef struct {
  char *buffer;         // Will point to g_json_response_buffer
  size_t buffer_size;   // Will be MAX_JSON_SIZE
  size_t current_len;   // How much data we've copied so far
  bool transfer_complete; // Flag to signal the main loop
  bool error_occurred;    // Flag for errors
} http_transfer_state_t;

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

    // Fix: use pointer for header
    const struct mg_str *cl_hdr = mg_http_get_header(hm, "Content-Length");
    size_t expected_len = 0;
    if (cl_hdr != NULL && cl_hdr->len > 0) {
      expected_len = (size_t) atoi(cl_hdr->buf);
    }

    printf("Received body length: %d\r\n", (int) hm->body.len);
    if (expected_len > 0 && hm->body.len < expected_len) {
      // Incomplete body, wait for more
      return;
    }

    size_t copy_len = hm->body.len < MAX_RESPONSE_SIZE - 1 ? hm->body.len : MAX_RESPONSE_SIZE - 1;
    memcpy(response_buf, hm->body.buf, copy_len);
    response_buf[copy_len] = '\0';
    response_len = copy_len;

    printf("HTTP Response: %.*s\r\n", (int) response_len, response_buf);

    request_done = true;
  }
  else if (ev == MG_EV_CLOSE && !request_done) {
    printf("HTTP request failed or connection closed early\r\n");
  }
}




void perform_http_data_read(void) {
  request_done = false;

  struct mg_connection *c = mg_http_connect(&g_mgr, API_URL, http_event_handler, NULL);
  if (c == NULL) {
    printf("HTTP connect failed\r\n");
    return;
  }

  mg_printf(c, "GET %s HTTP/1.0\r\nHost: 192.168.1.133\r\n\r\n", "/api/stats?start=-1h&stop=now()");

  uint32_t start = HAL_GetTick();
  while (!request_done && HAL_GetTick() - start < 7000) {
    mg_mgr_poll(&g_mgr, 10);  // Poll with more responsiveness
    HAL_Delay(1);
  }

  if (!request_done) {
    printf("Request timed out or incomplete\r\n");
  }
}




void wait_for_network_ready(void) {
  uint32_t start = HAL_GetTick();
  while (HAL_GetTick() - start < 2000) {
    mongoose_poll();
    HAL_Delay(1);
  }

  // Force ARP resolution with a dummy connection
  struct mg_connection *warmup = mg_connect(&g_mgr, "tcp://192.168.1.133:80", NULL, NULL);
  if (warmup != NULL) {
    mg_mgr_poll(&g_mgr, 100);
    warmup->is_closing = 1;
  }
}




double mg_str_to_d(struct mg_str s) {
    char buf[64];
    size_t n = s.len < sizeof(buf) - 1 ? s.len : sizeof(buf) - 1;
    memcpy(buf, s.buf, n);
    buf[n] = '\0';
    return atof(buf);  // Or strtod(buf, NULL) for more control
}



void write_json_to_nfc(const char *json_data, size_t json_len) {
    uint16_t i2c_addr = 0x53 << 1;   // ST25DV I2C address
    uint16_t mem_addr = 0x0008;      // NDEF start offset
    HAL_StatusTypeDef status;

    if (json_len > 8000) {
        printf("Error: JSON too large for NFC memory (%lu bytes)\n", (unsigned long)json_len);
        return;
    }

    const char* mime_type = "application/json";
    uint8_t type_length = strlen(mime_type);
    uint8_t ndef_header_size = 1 + 1 + 4;

    uint32_t payload_length = json_len;
    uint32_t ndef_record_length = ndef_header_size + type_length + payload_length;

    uint8_t ndef_buffer[MAX_NDEF_SIZE];
    uint16_t total_ndef_message_size = 0;

    // TLV block
    ndef_buffer[total_ndef_message_size++] = 0x03;  // TLV type
    if (ndef_record_length < 0xFF) {
        ndef_buffer[total_ndef_message_size++] = (uint8_t)ndef_record_length;
    } else {
        ndef_buffer[total_ndef_message_size++] = 0xFF;
        ndef_buffer[total_ndef_message_size++] = (ndef_record_length >> 8) & 0xFF;
        ndef_buffer[total_ndef_message_size++] = ndef_record_length & 0xFF;
    }

    // NDEF Record header
    ndef_buffer[total_ndef_message_size++] = 0xC2;  // MIME type record
    ndef_buffer[total_ndef_message_size++] = type_length;
    ndef_buffer[total_ndef_message_size++] = (payload_length >> 24) & 0xFF;
    ndef_buffer[total_ndef_message_size++] = (payload_length >> 16) & 0xFF;
    ndef_buffer[total_ndef_message_size++] = (payload_length >> 8) & 0xFF;
    ndef_buffer[total_ndef_message_size++] = payload_length & 0xFF;

    memcpy(&ndef_buffer[total_ndef_message_size], mime_type, type_length);
    total_ndef_message_size += type_length;

    memcpy(&ndef_buffer[total_ndef_message_size], json_data, payload_length);
    total_ndef_message_size += payload_length;

    ndef_buffer[total_ndef_message_size++] = 0xFE;  // Terminator TLV

    printf("JSON length: %lu\n", (unsigned long)json_len);
    printf("Total NDEF size: %u bytes\n", total_ndef_message_size);

    // I2C chunked write to NFC
    const uint16_t MAX_CHUNK_SIZE = 64;
    uint16_t bytes_remaining = total_ndef_message_size;
    uint16_t offset = 0;

    while (bytes_remaining > 0) {
        uint16_t chunk_size = (bytes_remaining > MAX_CHUNK_SIZE) ? MAX_CHUNK_SIZE : bytes_remaining;

        status = HAL_I2C_Mem_Write(&hi2c1, i2c_addr, mem_addr + offset,
                                   I2C_MEMADD_SIZE_16BIT,
                                   &ndef_buffer[offset], chunk_size,
                                   HAL_MAX_DELAY);

        if (status != HAL_OK) {
            printf("I2C chunk write failed at offset %d, status: %d\r\n", offset, status);
            return;
        }

        HAL_Delay(100);  // Wait for memory to settle
        offset += chunk_size;
        bytes_remaining -= chunk_size;
    }

    printf("I2C chunked write successful.\r\n");
}

bool parse_float_request_fields(const char *json_data, size_t len,
                                struct mg_str *out_field,
                                struct mg_str *out_start,
                                struct mg_str *out_stop) {
    struct mg_str json = mg_str_n(json_data, len);
    struct mg_str key, val;
    size_t offset = 0;

    *out_field = mg_str_n(NULL, 0);
    *out_start = mg_str_n(NULL, 0);
    *out_stop  = mg_str_n(NULL, 0);

    printf("🧪 Parsing JSON: %.*s\r\n", (int) json.len, json.buf);
    // Check if JSON is double-encoded (starts with quote)
    if (json_data[0] == '"' && json_data[len - 1] == '"') {
        printf("⚠️  JSON appears to be string-encoded, unescaping...\r\n");

        static char unescaped[1024];
        size_t j = 0;

        for (size_t i = 1; i < len - 1 && j < sizeof(unescaped) - 1; i++) {
            if (json_data[i] == '\\' && i + 1 < len - 1) {
                i++;  // Skip backslash
            }
            unescaped[j++] = json_data[i];
        }

        unescaped[j] = '\0';
        json_data = unescaped;
        len = strlen(unescaped);

        printf("🔓 Unescaped JSON: %s\n\r", json_data);
    }


    while ((offset = mg_json_next(json, offset, &key, &val)) > 0) {
    	printf("🔍 Found Key: %.*s | Value: %.*s\n", (int)key.len, key.buf, (int)val.len, val.buf);


    	if (mg_match(key, mg_str("\"field\""), NULL)) {
    	    *out_field = val;
    	    printf("✅ Matched 'field': %.*s\r\n", (int) val.len, val.buf);
    	} else if (mg_match(key, mg_str("\"start\""), NULL)) {
    	    *out_start = val;
    	    printf("✅ Matched 'start': %.*s\r\n", (int) val.len, val.buf);
    	} else if (mg_match(key, mg_str("\"stop\""), NULL)) {
    	    *out_stop = val;
    	    printf("✅ Matched 'stop': %.*s\r\n", (int) val.len, val.buf);
    	} else {
    	    // This will now correctly show "cmd" as the non-match
    	    printf("❌ No match for key: %.*s\r\n", (int) key.len, key.buf);
    	}
    }

    // Final report
    printf("📦 Parsed values:\r\n");
    printf("   field: %.*s\r\n", (int) out_field->len, out_field->buf);
    printf("   start: %.*s\r\n", (int) out_start->len, out_start->buf);
    printf("   stop : %.*s\r\n", (int) out_stop->len, out_stop->buf);

    if (out_field->len > 0 && out_start->len > 0 && out_stop->len > 0) {
        printf("✅ All required fields extracted.\r\n");
        return true;
    } else {
        printf("❌ Missing one or more required fields.\r\n");
        return false;
    }
}

void copy_and_strip_quotes(struct mg_str src, char *dest, size_t dest_size) {
    size_t j = 0;
    for (size_t i = 0; i < src.len && j < dest_size - 1; i++) {
        if (src.buf[i] != '"') {
            dest[j++] = src.buf[i];
        }
    }
    dest[j] = '\0';
}



void read_phone_request_json(void) {
    uint16_t i2c_addr = 0x53 << 1;
    uint16_t mem_addr = 0x0008;
    uint8_t buf[256];

    // Read first 256 bytes of EEPROM
    if (HAL_I2C_Mem_Read(&hi2c1, i2c_addr, mem_addr, I2C_MEMADD_SIZE_16BIT, buf, sizeof(buf), HAL_MAX_DELAY) != HAL_OK) {
        printf("I2C read failed\n");
        return;
    }

    uint16_t i = 0;

    if (buf[i++] != 0x03) {
        printf("Not an NDEF TLV.\n");
        return;
    }

    uint8_t tlv_length = buf[i++];  // assuming < 255 bytes

    // NDEF record header
    if (buf[i++] != 0xD1) {
        printf("Not a text record from phone.\n");
        return;
    }

    uint8_t type_len = buf[i++];
    uint8_t payload_len = buf[i++];

    // skip type string ("T")
    i += type_len;

    // first byte of payload is encoding/language code (usually 0x02)
    i++;

    // JSON string starts here
    char json[240];
    memcpy(json, &buf[i], payload_len - 1);
    json[payload_len - 1] = '\0';

    printf("📥 Received JSON from phone: %s\n", json);


    char *json_data = strchr(json, '{');  // Find the first '{' character
    if (!json_data) {
        printf("❌ No JSON object found in payload\n");
        return;
    }

    // Step 2: Declare output variables
    struct mg_str field, start, stop;

    printf("Raw JSON: %s\n", json_data);


    // Step 3: Parse for cmd/field/start/stop
    if (parse_float_request_fields(json_data, strlen(json_data), &field, &start, &stop)) {
        // Step 4: Use them to build the HTTP URL
        char url[256];
        snprintf(url, sizeof(url),
                 "http://192.168.1.133:8080/api/float-range?field=%.*s&start=%.*s&stop=%.*s",
                 (int) field.len, field.buf,
                 (int) start.len, start.buf,
                 (int) stop.len,  stop.buf);

        printf("🌐 Generated HTTP URL:\n%s\n", url);

        char field_buf[128], start_buf[64], stop_buf[64];

        copy_and_strip_quotes(field, field_buf, sizeof(field_buf));
        copy_and_strip_quotes(start, start_buf, sizeof(start_buf));
        copy_and_strip_quotes(stop,  stop_buf,  sizeof(stop_buf));

        printf("✅ Clean URL:\n");
        printf("http://192.168.1.133:8080/api/float-range?field=%s&start=%s&stop=%s\n",
               field_buf, start_buf, stop_buf);

        char full_path[256];
        snprintf(full_path, sizeof(full_path),
                 "/api/float-range?field=%s&start=%s&stop=%s",
                 field_buf, start_buf, stop_buf);

        // Optional debug print
        printf("✅ Clean path: %s\n", full_path);
        wait_for_network_ready();
        perform_http_get(full_path);
    	HAL_Delay(1000);

        // Optionally perform the HTTP request here...
        // perform_http_get(url);
    } else {
        printf("❌ Failed to extract field/start/stop from JSON\n");
    }
}

void perform_http_get(const char *path) {
  request_done = false;

  printf("🌐 Starting HTTP GET for path: %s\r\n", path);

  // Connect to server
  struct mg_connection *c = mg_http_connect(&g_mgr, "http://192.168.1.133:8080", http_event_handler, NULL);
  if (c == NULL) {
    printf("❌ HTTP connect failed\r\n");
    return;
  }

  // Send custom GET request
  mg_printf(c,
    "GET %s HTTP/1.1\r\n"
    "Host: 192.168.1.133\r\n"
    "Connection: keep-alive\r\n\r\n",
    path);



  // Poll until request completes or times out
  uint32_t start = HAL_GetTick();
  while (!request_done && HAL_GetTick() - start < 5000) {
    mg_mgr_poll(&g_mgr, 1);
    HAL_Delay(1);
  }

  if (!request_done) {
    printf("❌ HTTP request timed out\r\n");
    return;
  }
  ParsedData parsed;
  char compact_json[OUTPUT_BUF_SIZE];

  if (parse_response_json(response_buf, &parsed)) {
    make_compact_json(&parsed, compact_json, sizeof(compact_json));
    printf("Compact JSON:\n%s\r\n", compact_json);
  } else {
    printf("Failed to parse JSON!\r\n");
  }

  // ✅ HTTP response is now in `response_buf`, call your NFC writer
  write_json_to_nfc(compact_json, strlen(compact_json));
}

int parse_response_json(const char *json, ParsedData *out) {
  const char *ptr = json;
  int i = 0;
  char prev_time_str[32] = {0};
  int found_first = 0;
  out->interval_sec = 0;
  out->count = 0;
  float last_value = 0.0;
  time_t expected_time = 0;

  while ((ptr = strstr(ptr, "{\"time\":\"")) && i < MAX_VALUES) {
    ptr += 9;  // Move past "{\"time\":\""
    const char *time_end = strchr(ptr, '"');
    if (!time_end) break;

    int len = time_end - ptr;
    if (len >= sizeof(out->start_time)) len = sizeof(out->start_time) - 1;

    char current_time_str[32];
    strncpy(current_time_str, ptr, len);
    current_time_str[len] = '\0';

    // Parse timestamp into struct tm
    char trimmed[32];
    strncpy(trimmed, current_time_str, sizeof(trimmed));
    size_t len_trimmed = strlen(trimmed);
    if (trimmed[len_trimmed - 1] == 'Z') trimmed[len_trimmed - 1] = '\0';

    struct tm tm = {0};
    strptime(trimmed, "%Y-%m-%dT%H:%M:%S", &tm);
    time_t current_time = mktime(&tm);

    // Get value
    const char *val_ptr = strstr(time_end, "\"value\":");
    if (!val_ptr) break;
    float v = atof(val_ptr + 8);

    if (!found_first) {
      strcpy(out->start_time, current_time_str);
      expected_time = current_time;
      found_first = 1;
      last_value = v;
      out->values[i++] = v;
    } else {
      if (out->interval_sec == 0) {
        // Compute interval from previous and current timestamps
        char prev_trimmed[32];
        strncpy(prev_trimmed, prev_time_str, sizeof(prev_trimmed));
        size_t len_prev = strlen(prev_trimmed);
        if (prev_trimmed[len_prev - 1] == 'Z') prev_trimmed[len_prev - 1] = '\0';

        struct tm tm_prev = {0};
        strptime(prev_trimmed, "%Y-%m-%dT%H:%M:%S", &tm_prev);
        time_t prev_time = mktime(&tm_prev);
        out->interval_sec = (int)difftime(current_time, prev_time);
        expected_time = prev_time + out->interval_sec;
      }

      // Fill in missing values using last_value
      while (expected_time < current_time && i < MAX_VALUES) {
        out->values[i++] = last_value;
        expected_time += out->interval_sec;
      }

      // Insert current value
      out->values[i++] = v;
      last_value = v;
      expected_time += out->interval_sec;
    }

    strcpy(prev_time_str, current_time_str);
  }

  out->count = i;
  return (i > 0) ? 1 : 0;
}


// Creates compact JSON output
void make_compact_json(const ParsedData *data, char *out_buf, size_t out_buf_size) {
  int offset = snprintf(out_buf, out_buf_size,
    "{\n  \"start\": \"%s\",\n  \"interval\": %d,\n  \"values\": [",
    data->start_time, data->interval_sec);

  for (int i = 0; i < data->count; i++) {
    offset += snprintf(out_buf + offset, out_buf_size - offset,
      (i < data->count - 1) ? "%.2f," : "%.2f", data->values[i]);
  }

  snprintf(out_buf + offset, out_buf_size - offset, "]\n}\n");
  printf("Number of values: %d\n", data->count);
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
  for (;;) {
    mongoose_poll();

    if(!HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_6)){
    	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
    	HAL_Delay(5000);
    	read_phone_request_json();
    } else{
    	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
    }

    if(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13)){
    	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
    	HAL_Delay(500);
    	glue_update_state();
        wait_for_network_ready();
    	perform_http_data_read();
    	HAL_Delay(1000);
    	write_json_to_nfc(response_buf, response_len);
    } else{
    	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
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

  /*Configure GPIO pin : PA6 */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

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
