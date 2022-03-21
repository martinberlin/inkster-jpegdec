// Requirements: Needs to have an EPD Class defined. Needs PSRAM.
#include <Arduino.h>

// Needs Lovyan as a component:
// git submodule add https://github.com/lovyan03/LovyanGFX.git components/LovyanGFX
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_types.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_sleep.h"
// WiFi related
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "lwip/err.h"
#include "lwip/sys.h"
// HTTP Client + time
#include "esp_netif.h"
#include "esp_http_client.h"
#include "esp_sntp.h"
#include "soc/rtc_wdt.h"

// JPG decoder
#if ESP_IDF_VERSION_MAJOR >= 4 // IDF 4+
  #include "esp32/rom/tjpgd.h"
#else // ESP32 Before IDF 4.0
  #include "rom/tjpgd.h"
#endif

#include "esp_task_wdt.h"
#include <stdio.h>
#include <string.h>
#include <math.h> // round + pow

// - - - - Display configuration - - - - - - - - -
extern "C"
{
    #include "epd_driver.h"
    #include "epd_highlevel.h"
}
EpdiyHighlevelState hl;
int temperature = 25;

// JPG decoder from @bitbank2
#include "JPEGDEC.h"
JPEGDEC jpeg;
// Returns already EPD_WIDTH / HEIGHT

// Affects the gamma to calculate gray (lower is darker/higher contrast) Note: That on true is slower
// Nice test values: 0.9 1.2 1.4 higher and is too bright
#define USE_GAMMA_CORRECTION true
double gamma_value = 0.8;

// Real pem cert of loremflickr.com
const char * server_cert_pem = "-----BEGIN CERTIFICATE-----\nMIIDdzCCAl+gAwIBAgIEAgAAuTANBgkqhkiG9w0BAQUFADBaMQswCQYDVQQGEwJJ\nRTESMBAGA1UEChMJQmFsdGltb3JlMRMwEQYDVQQLEwpDeWJlclRydXN0MSIwIAYD\nVQQDExlCYWx0aW1vcmUgQ3liZXJUcnVzdCBSb290MB4XDTAwMDUxMjE4NDYwMFoX\nDTI1MDUxMjIzNTkwMFowWjELMAkGA1UEBhMCSUUxEjAQBgNVBAoTCUJhbHRpbW9y\nZTETMBEGA1UECxMKQ3liZXJUcnVzdDEiMCAGA1UEAxMZQmFsdGltb3JlIEN5YmVy\nVHJ1c3QgUm9vdDCCASIwDQYJKoZIhvcNAQEBBQADggEPADCCAQoCggEBAKMEuyKr\nmD1X6CZymrV51Cni4eiVgLGw41uOKymaZN+hXe2wCQVt2yguzmKiYv60iNoS6zjr\nIZ3AQSsBUnuId9Mcj8e6uYi1agnnc+gRQKfRzMpijS3ljwumUNKoUMMo6vWrJYeK\nmpYcqWe4PwzV9/lSEy/CG9VwcPCPwBLKBsua4dnKM3p31vjsufFoREJIE9LAwqSu\nXmD+tqYF/LTdB1kC1FkYmGP1pWPgkAx9XbIGevOF6uvUA65ehD5f/xXtabz5OTZy\ndc93Uk3zyZAsuT3lySNTPx8kmCFcB5kpvcY67Oduhjprl3RjM71oGDHweI12v/ye\njl0qhqdNkNwnGjkCAwEAAaNFMEMwHQYDVR0OBBYEFOWdWTCCR1jMrPoIVDaGezq1\nBE3wMBIGA1UdEwEB/wQIMAYBAf8CAQMwDgYDVR0PAQH/BAQDAgEGMA0GCSqGSIb3\nDQEBBQUAA4IBAQCFDF2O5G9RaEIFoN27TyclhAO992T9Ldcw46QQF+vaKSm2eT92\n9hkTI7gQCvlYpNRhcL0EYWoSihfVCr3FvDB81ukMJY2GQE/szKN+OMY3EU/t3Wgx\njkzSswF07r51XgdIGn9w/xZchMB5hbgF/X++ZRGjD8ACtPhSNzkE1akxehi/oCr0\nEpn3o0WC4zxe9Z2etciefC7IpJ5OCBRLbf1wbWsaY71k5h+3zvDyny67G7fyUIhz\nksLi4xaNmjICq44Y3ekQEe5+NauQrz4wlHrQMz2nZQ/1/I6eYs9HRCwBXbsdtTLS\nR9I4LtD+gdwyah617jzV/OeBHRnDJELqYzmp\n-----END CERTIFICATE-----";

#define LGFX_USE_V1
#include <LovyanGFX.hpp>

class LGFX : public lgfx::LGFX_Device
{
  lgfx::Panel_IT8951   _panel_instance;
  lgfx::Bus_SPI       _bus_instance; 
  lgfx::Light_PWM     _light_instance;

public:

  // コンストラクタを作成し、ここで各種設定を行います。
  // クラス名を変更した場合はコンストラクタも同じ名前を指定してください。
  LGFX(void)
  {
    { // バス制御の設定を行います。
      auto cfg = _bus_instance.config();    // バス設定用の構造体を取得します。

// SPIバスの設定
      cfg.spi_host = VSPI_HOST;     // 使用するSPIを選択  (VSPI_HOST or HSPI_HOST)
      cfg.spi_mode = 0;             // SPI通信モードを設定 (0 ~ 3)
      cfg.freq_write = 40000000;    // 送信時のSPIクロック (最大80MHz, 80MHzを整数で割った値に丸められます)
      cfg.freq_read  = 16000000;    // 受信時のSPIクロック
      cfg.spi_3wire  = true;        // 受信をMOSIピンで行う場合はtrueを設定
      cfg.use_lock   = true;        // トランザクションロックを使用する場合はtrueを設定
      cfg.dma_channel = 2;          // Set the DMA channel (1 or 2. 0=disable)   使用するDMAチャンネルを設定 (0=DMA不使用)
      cfg.pin_sclk = 1;             // dummy since we dont use SPI
      cfg.pin_mosi = 3;             // dummy
      cfg.pin_miso = -1;            // SPIのMISOピン番号を設定 (-1 = disable)
      cfg.pin_dc   = -1;            // SPIのD/Cピン番号を設定  (-1 = disable)
 
        // Disabling BUS Config
      _bus_instance.config(cfg);    // 設定値をバスに反映します。
      _panel_instance.setBus(&_bus_instance);      // バスをパネルにセットします。
    }

    { // 表示パネル制御の設定を行います。
      auto cfg = _panel_instance.config();    // 表示パネル設定用の構造体を取得します。

      cfg.pin_cs           =    -1;  // CSが接続されているピン番号   (-1 = disable)
      cfg.pin_rst          =    -1;     // RSTが接続されているピン番号  (-1 = disable)
      cfg.pin_busy         =    -1;    // BUSYが接続されているピン番号 (-1 = disable)

      // ※ 以下の設定値はパネル毎に一般的な初期値が設定されていますので、不明な項目はコメントアウトして試してみてください。

      cfg.memory_width     =   EPD_WIDTH;  // ドライバICがサポートしている最大の幅
      cfg.memory_height    =   EPD_HEIGHT;  // ドライバICがサポートしている最大の高さ
      cfg.panel_width      =   EPD_WIDTH;  // 実際に表示可能な幅
      cfg.panel_height     =   EPD_HEIGHT;  // 実際に表示可能な高さ
      cfg.offset_x         =     0;  // パネルのX方向オフセット量
      cfg.offset_y         =     0;  // パネルのY方向オフセット量
      cfg.offset_rotation  =     0;  // 回転方向の値のオフセット 0~7 (4~7は上下反転)
      cfg.dummy_read_pixel =     8;  // ピクセル読出し前のダミーリードのビット数
      cfg.dummy_read_bits  =     1;  // ピクセル以外のデータ読出し前のダミーリードのビット数
      cfg.readable         =  true;  // データ読出しが可能な場合 trueに設定
      cfg.invert           =  true;  // パネルの明暗が反転してしまう場合 trueに設定
      cfg.rgb_order        = false;  // パネルの赤と青が入れ替わってしまう場合 trueに設定
      cfg.dlen_16bit       = false;  // データ長を16bit単位で送信するパネルの場合 trueに設定
      cfg.bus_shared       =  true;  // SDカードとバスを共有している場合 trueに設定(drawJpgFile等でバス制御を行います)

      _panel_instance.config(cfg);
    }
    printf("Not doing setPanel for this example\n");
    //setPanel(&_panel_instance); // 使用するパネルをセットします。
  }
};

// 準備したクラスのインスタンスを作成します。
LGFX love;
// - - - - end of Display configuration  - - - - -

// JPEG decoder
JDEC jd; 
JRESULT rc;
// Buffers
uint8_t *fb;            // EPD 2bpp buffer
uint8_t *source_buf;    // JPG download buffer
uint8_t *decoded_image; // RAW decoded image
static uint8_t tjpgd_work[3096]; // tjpgd buffer. min size: 3096 bytes

uint32_t buffer_pos = 0;
uint32_t time_download = 0;
uint32_t time_decomp = 0;
uint32_t time_render = 0;
static const char * jd_errors[] = {
    "Succeeded",
    "Interrupted by output function",
    "Device error or wrong termination of input stream",
    "Insufficient memory pool for the image",
    "Insufficient stream input buffer",
    "Parameter error",
    "Data format error",
    "Right format but not supported",
    "Not supported JPEG standard"
};

uint16_t ep_width = EPD_WIDTH;
uint16_t ep_height = EPD_HEIGHT;
uint8_t gamme_curve[256];

static const char *TAG = "TJpeg";
uint16_t countDataEventCalls = 0;
uint32_t countDataBytes = 0;
uint32_t img_buf_pos = 0;
uint32_t dataLenTotal = 0;
uint64_t startTime = 0;

#if VALIDATE_SSL_CERTIFICATE == true
  /* Time aware for ESP32: Important to check SSL certs validity */
  void time_sync_notification_cb(struct timeval *tv)
  {
      ESP_LOGI(TAG, "Notification of a time synchronization event");
  }

  static void initialize_sntp(void)
  {
      ESP_LOGI(TAG, "Initializing SNTP");
      sntp_setoperatingmode(SNTP_OPMODE_POLL);
      sntp_setservername(0, "pool.ntp.org");
      sntp_set_time_sync_notification_cb(time_sync_notification_cb);
  #ifdef CONFIG_SNTP_TIME_SYNC_METHOD_SMOOTH
      sntp_set_sync_mode(SNTP_SYNC_MODE_SMOOTH);
  #endif
      sntp_init();
  }

  static void obtain_time(void)
  {
      initialize_sntp();

      // wait for time to be set
      time_t now = 0;
      struct tm timeinfo = { 0 };
      int retry = 0;
      const int retry_count = 10;
      while (sntp_get_sync_status() == SNTP_SYNC_STATUS_RESET && ++retry < retry_count) {
          ESP_LOGI(TAG, "Waiting for system time to be set... (%d/%d)", retry, retry_count);
          vTaskDelay(2000 / portTICK_PERIOD_MS);
      }
      time(&now);
      localtime_r(&now, &timeinfo);
  }
#endif
//====================================================================================
// This sketch contains support functions to render the Jpeg images
//
// Created by Bodmer 15th Jan 2017
// Refactored by @martinberlin for EPDiy as a Jpeg download and render example
//====================================================================================

// Return the minimum of two values a and b
#define minimum(a,b)     (((a) < (b)) ? (a) : (b))

uint8_t find_closest_palette_color(uint8_t oldpixel)
{
  return oldpixel & 0xF0;
}

void deepsleep(){
    esp_deep_sleep(1000000LL * 60 * DEEPSLEEP_MINUTES_AFTER_RENDER);
}

static uint32_t feed_buffer(JDEC *jd,      
               uint8_t *buff, // Pointer to the read buffer (NULL:skip) 
               uint32_t nd 
) {
    uint32_t count = 0;

    while (count < nd) {
      if (buff != NULL) {
            *buff++ = source_buf[buffer_pos];
        }
        count ++;
        buffer_pos++;
    }

  return count;
}

/* User defined call-back function to output decoded RGB bitmap in decoded_image buffer 
   In this v2 example the pixels are set directly on tjd_output without using a decoded buffer
*/
static uint32_t
tjd_output(JDEC *jd,     /* Decompressor object of current session */
           void *bitmap, /* Bitmap data to be output */
           JRECT *rect   /* Rectangular region to output */
) {
  uint32_t render_start = esp_timer_get_time();
  
  uint32_t w = rect->right - rect->left + 1;
  uint32_t h = rect->bottom - rect->top + 1;
  uint32_t image_width = jd->width;
  uint8_t *bitmap_ptr = (uint8_t*)bitmap;
  
  // This pushes Image directly via SPI, which is not what we want here
  //love.pushImage(rect->left, rect->top, w, h, (lgfx::rgb888_t*)bitmap);
 
  for (uint32_t i = 0; i < w * h; i++) {

    uint8_t r = *(bitmap_ptr++);
    uint8_t g = *(bitmap_ptr++);
    uint8_t b = *(bitmap_ptr++);

    // Calculate weighted grayscale (Not used now)
    //uint32_t val = ((r * 30 + g * 59 + b * 11) / 100); // original formula
    //uint32_t val = (r*38 + g*75 + b*15) >> 7; // @vroland recommended formula

    int xx = rect->left + i % w;
    if (xx < 0 || xx >= image_width) {
      continue;
    }
    int yy = rect->top + i / w;
    if (yy < 0 || yy >= jd->height) {
      continue;
    }

    // Here would be great to hold back the pre-processed LovyanGFX image but now it simply uses the internal gray converted
    // Write the pixel using color888
    #if USE_GAMMA_CORRECTION == false
      epd_draw_pixel(xx, yy, love.color888(r, g, b) , fb);
    #else
      epd_draw_pixel(xx, yy,  love.color888(gamme_curve[r], gamme_curve[g], gamme_curve[b]), fb);
    #endif
  }

  // Copy the square processed by Lovyan into the EPD framebuffer (Could not find a way to return image)
  /* EpdRect sq = {
      .x = rect->left,
      .y = rect->top,
      .width = w,
      .height = h
  };
  epd_copy_to_framebuffer(sq, (uint8_t*) bitmap, fb); */

  rtc_wdt_feed();
  time_render += (esp_timer_get_time() - render_start)/1000;

  return 1;
}

//====================================================================================
//   This function opens source_buf Jpeg image file and primes the decoder
//====================================================================================
int drawBufJpeg(uint8_t *source_buf, int xpos, int ypos) {
  rc = jd_prepare(&jd, feed_buffer, tjpgd_work, sizeof(tjpgd_work), &source_buf);
  if (rc != JDR_OK) {    
    ESP_LOGE(TAG, "JPG jd_prepare error: %s", jd_errors[rc]);
    return ESP_FAIL;
  }

  uint32_t decode_start = esp_timer_get_time();

  // Last parameter scales        v 1 will reduce the image
  rc = jd_decomp(&jd, tjd_output, 0);
  if (rc != JDR_OK) {
    ESP_LOGE(TAG, "JPG jd_decomp error: %s", jd_errors[rc]);
    return ESP_FAIL;
  }

  time_decomp = (esp_timer_get_time() - decode_start)/1000;

  ESP_LOGI("JPG", "width: %d height: %d\n", jd.width, jd.height);
  ESP_LOGI("decode", "%d ms . image decompression", time_decomp);

  // Render the image onto the screen at given coordinates
  //jpegRender(xpos, ypos, jd.width, jd.height);

  return 1;
}

// Handles Htpp events and is in charge of buffering source_buf (jpg compressed image)
esp_err_t _http_event_handler(esp_http_client_event_t *evt)
{
    switch (evt->event_id)
    {
    case HTTP_EVENT_ERROR:
        ESP_LOGE(TAG, "HTTP_EVENT_ERROR");
        break;
    case HTTP_EVENT_ON_CONNECTED:
        ESP_LOGI(TAG, "HTTP_EVENT_ON_CONNECTED");
        break;
    case HTTP_EVENT_HEADER_SENT:
        ESP_LOGI(TAG, "HTTP_EVENT_HEADER_SENT");
        break;
    case HTTP_EVENT_ON_HEADER:
        #if DEBUG_VERBOSE
          ESP_LOGI(TAG, "HTTP_EVENT_ON_HEADER, key=%s, value=%s", evt->header_key, evt->header_value);
        #endif
        break;
    case HTTP_EVENT_ON_DATA:
        ++countDataEventCalls;
        #if DEBUG_VERBOSE
          if (countDataEventCalls%10==0) {
            ESP_LOGI(TAG, "%d len:%d\n", countDataEventCalls, evt->data_len);
          }
        #endif
        dataLenTotal += evt->data_len;

        if (countDataEventCalls == 1) startTime = esp_timer_get_time();
        // Append received data into source_buf
        memcpy(&source_buf[img_buf_pos], evt->data, evt->data_len);
        img_buf_pos += evt->data_len;

        // Optional hexa dump
        //ESP_LOG_BUFFER_HEX(TAG, source_buf, 100);
        break;

    case HTTP_EVENT_ON_FINISH:
        // Do not draw if it's a redirect (302)
        if (esp_http_client_get_status_code(evt->client) == 200) {

          love.startWrite();
          printf("%d bytes read from %s\nIMG_BUF size: %d\n", img_buf_pos, IMG_URL, img_buf_pos);
          drawBufJpeg(source_buf, 0, 0);
          time_download = (esp_timer_get_time()-startTime)/1000;
          printf("%d ms - download", time_download);
          
          // Refresh display
          love.endWrite();
          
          // Refresh display
          printf("\nepd_hl_update_screen -> Refresh EPD\n");
          //epd_fill_circle(100, 100, 10, 0 , fb);
          epd_hl_update_screen(&hl, MODE_GC16, 25);

          printf("%d ms . image render\n", time_render);
          printf("%d ms - total time spent\n", time_download+time_decomp+time_render);
        } else {
          printf("HTTP on finish got status code: %d\n", esp_http_client_get_status_code(evt->client));
        }
        break;

    case HTTP_EVENT_DISCONNECTED:
        ESP_LOGI(TAG, "HTTP_EVENT_DISCONNECTED\n");
        break;
    }
    return ESP_OK;
}

// Handles http request
static void http_post(void)
{    
    epd_fullclear(&hl, temperature);
    /**
     * NOTE: All the configuration parameters for http_client must be specified
     * either in URL or as host and path parameters.
     * FIX: Uncommenting cert_pem restarts even if providing the right certificate
     */
    esp_http_client_config_t config = {
        .url = IMG_URL,
        #if VALIDATE_SSL_CERTIFICATE == 1
          // Research how to do this in Arduino-ESP32
          .cert_pem = server_cert_pem,
        #endif
        .disable_auto_redirect = false,
        .event_handler = _http_event_handler,
        .buffer_size = HTTP_RECEIVE_BUFFER_SIZE,
        };

    esp_http_client_handle_t client = esp_http_client_init(&config);

    #if DEBUG_VERBOSE
      printf("Free heap before HTTP download: %d\n", xPortGetFreeHeapSize());
      if (esp_http_client_get_transport_type(client) == HTTP_TRANSPORT_OVER_SSL && config.cert_pem) {
        printf("SSL CERT:\n%s\n\n", server_cert_pem);
      }
    #endif
    
    esp_err_t err = esp_http_client_perform(client);
    if (err == ESP_OK)
    {
        printf("\nIMAGE URL: %s\n\nHTTP GET Status = %d, content_length = %d\n",
                 IMG_URL,
                 esp_http_client_get_status_code(client),
                 esp_http_client_get_content_length(client));
    }
    else
    {
        printf("\nHTTP GET request failed: %s", esp_err_to_name(err));
    }

    printf("Go to sleep %d minutes (Disabled)\n", DEEPSLEEP_MINUTES_AFTER_RENDER);
    esp_http_client_cleanup(client);
    vTaskDelay(10);
    //deepsleep();
}

/* FreeRTOS event group to signal when we are connected*/
static EventGroupHandle_t s_wifi_event_group;

/* The event group allows multiple bits for each event, but we only care about two events:
 * - we are connected to the AP with an IP
 * - we failed to connect after the maximum amount of retries */
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT BIT1
static int s_retry_num = 0;

static void event_handler(void *arg, esp_event_base_t event_base,
                          int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START)
    {
        esp_wifi_connect();
    }
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED)
    {
        if (s_retry_num < 5)
        {
            esp_wifi_connect();
            s_retry_num++;
            printf("Retry to connect to the AP\n");
        }
        else
        {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
            printf("Connect to the AP failed %d times. Going to deepsleep %d minutes\n", 5, DEEPSLEEP_MINUTES_AFTER_RENDER);
            deepsleep();
        }
    }
    else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP)
    {
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

// Initializes WiFi the ESP-IDF way
void wifi_init_sta(void)
{
    s_wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());

    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_got_ip));
    // C++ wifi config
    wifi_config_t wifi_config;
    memset(&wifi_config, 0, sizeof(wifi_config));
    wifi_config.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;
    sprintf(reinterpret_cast<char *>(wifi_config.sta.ssid), ESP_WIFI_SSID);
    sprintf(reinterpret_cast<char *>(wifi_config.sta.password), ESP_WIFI_PASSWORD);
    wifi_config.sta.pmf_cfg.capable = true;

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config((wifi_interface_t)ESP_IF_WIFI_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    /* Waiting until either the connection is established (WIFI_CONNECTED_BIT) or connection failed for the maximum
     * number of re-tries (WIFI_FAIL_BIT). The bits are set by event_handler() (see above) */
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
                                           WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
                                           pdFALSE,
                                           pdFALSE,
                                           portMAX_DELAY);

    /* xEventGroupWaitBits() returns the bits before the call returned, hence we can test which event actually happened. */
    if (bits & WIFI_CONNECTED_BIT)
    {
        printf("Connected to ap SSID:%s", ESP_WIFI_SSID);
    }
    else if (bits & WIFI_FAIL_BIT)
    {
        printf("Failed to connect to SSID:%s", ESP_WIFI_PASSWORD);
    }
    else
    {
        printf("UNEXPECTED EVENT");
    }

    /* The event will not be processed after unregister */
    ESP_ERROR_CHECK(esp_event_handler_instance_unregister(IP_EVENT, IP_EVENT_STA_GOT_IP, instance_got_ip));
    ESP_ERROR_CHECK(esp_event_handler_instance_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID, instance_any_id));
    vEventGroupDelete(s_wifi_event_group);
}

void loop() {}

void setup() {
  Serial.begin(115200);
  epd_init(EPD_OPTIONS_DEFAULT);
  hl = epd_hl_init(EPD_BUILTIN_WAVEFORM);
  fb = epd_hl_get_framebuffer(&hl);

  // Should be big enough to allocate the JPEG file size, width * height should suffice
  source_buf = (uint8_t *)heap_caps_malloc(ep_width * ep_height/2, MALLOC_CAP_SPIRAM);
  if (source_buf == NULL) {
      printf("ERROR:Initial alloc source_buf failed. Is PSRAM enabled?");
  }
  printf("Free heap after buffers allocation: %d\n", xPortGetFreeHeapSize());

  
  love.init();
  // epd_fast:    LovyanGFX uses a 4×4 16pixel tile pattern to display a pseudo 17level grayscale.
  // epd_quality: Uses 16 levels of grayscale
  love.setEpdMode(epd_mode_t::epd_quality);

  double gammaCorrection = 1.0 / gamma_value;
  for (int gray_value =0; gray_value<256;gray_value++)
    gamme_curve[gray_value]= round (255*pow(gray_value/255.0, gammaCorrection));

  // Initialize NVS
  esp_err_t ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
  {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
  }
  ESP_ERROR_CHECK(ret);

  // WiFi log level set only to Error otherwise outputs too much
  esp_log_level_set("wifi", ESP_LOG_ERROR);
  
  // Initialization: WiFi + clean screen while downloading image
  wifi_init_sta();
  #if VALIDATE_SSL_CERTIFICATE == true
    obtain_time();
  #endif
  
  http_post();
}
