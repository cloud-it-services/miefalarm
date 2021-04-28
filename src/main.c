#include "mgos.h"
#include "mgos_dht.h"
#include "mgos_uart.h"
#include "mgos_http_server.h"
#include "mgos_mqtt.h"
#include "pt/pt.h"

#define UART_NO 2
#define MHZ19_DATA_LEN 9
#define MEASURE_CYCLES 100
#define CALIBRATE_CYCLES 100
#define MAIN_LOOP_TICK_MS 1
#define HIGH 0x1
#define LOW 0x0
//#define CONFIG_FILENAME "mq135.conf"

// mhz19c commands
#define MHZ19C_CMD_ABC_MODE 0x79
#define MHZ19C_CMD_READ_CO2 0x86
#define MHZ19C_CMD_CALIBRATE 0x87
#define MHZ19C_ABC_MODE_ON 0xA0  // ABC mode on
#define MHZ19C_ABC_MODE_OFF 0x00 // ABC mode off

static struct mgos_dht *s_dht = NULL;
static uint64_t last_tick = 0;
static uint8_t temperature = 1;
static uint8_t humidity = 1;
static uint16_t ppm = 0;
static uint8_t calibrating = 0;

// threads
static struct pt pt_dht;
static struct pt pt_display;
static struct pt pt_button;
static struct pt pt_mhz19c_calibrate;
static struct pt pt_mhz19c_measure;
static struct pt pt_http_push;
static struct pt pt_mqtt_push;

/*
static void write_config()
{
  // store attenuation and r0 in filesystem
  FILE *fp = fopen(CONFIG_FILENAME, "w+");
  if (fp == NULL)
    return;

  //LOG(LL_DEBUG, ("write config: %d\n%.6f\n", adc_attenuation, r0));
  fprintf(fp, "%d\n%f\n", adc_attenuation, r0);
  fclose(fp);
}

static void read_config()
{
  FILE *fp = fopen(CONFIG_FILENAME, "r");
  if (fp == NULL)
  {
    LOG(LL_DEBUG, ("%s not found\n", CONFIG_FILENAME));
    return;
  }
  fscanf(fp, "%hhu\n", &adc_attenuation);
  LOG(LL_DEBUG, ("set adc_attenuation: %d\n", adc_attenuation));
  fscanf(fp, "%lf\n", &r0);
  LOG(LL_DEBUG, ("set r0: %f\n", r0));
  fclose(fp);
}
*/

typedef struct
{
  uint8_t temperature;
  uint8_t humidity;
  uint16_t co2;
  uint8_t calibrating;
} sensor_data;

int json(struct json_out *out, va_list *ap)
{
  sensor_data *t = va_arg(*ap, sensor_data *);
  return json_printf(out, "{temperature: %d, humidity: %d, co2: %d, calibrating: %B}", t->temperature, t->humidity, t->co2, t->calibrating);
}

static void sensor_data_to_json(char *buf, uint8_t len)
{
  struct json_out out = JSON_OUT_BUF(buf, len);
  sensor_data sd = {temperature, humidity, ppm, calibrating};
  json_printf(&out, "%M", json, &sd);
}

static void get_sensor_data(struct mg_connection *c, int ev, void *p, void *user_data)
{
  if (ev != MG_EV_HTTP_REQUEST)
    return;

  char buf[200];
  sensor_data_to_json(buf, sizeof(buf));
  mg_send_response_line(c, 200, "Content-Type: application/json\r\n");
  mg_printf(c, "%s\r\n", buf);
  c->flags |= MG_F_SEND_AND_CLOSE;
}

/*
static void ev_handler(struct mg_connection *c, int ev, void *evd, void *cb_arg)
{
  const struct http_message *msg = (const struct http_message *)(evd);
  switch (ev)
  {
  case MG_EV_CONNECT:
    LOG(LL_INFO, ("CONNECT %s:%d - err=%d", __FUNCTION__, __LINE__, *(int *)evd));
    if (*(int *)evd != 0)
    {
      c->flags |= MG_F_CLOSE_IMMEDIATELY;
      return;
    }
    break;
  case MG_EV_HTTP_REPLY:
  {
    LOG(LL_INFO, ("Got reply:\n%.*s\n", (int) msg->body.len, msg->body.p));
    c->flags |= MG_F_SEND_AND_CLOSE;
    
    //LOG(LL_INFO, ("%s:%d - MG_EV_HTTP_REPLY - len=%d", __FUNCTION__, __LINE__, (int)msg->body.len));
    //c->flags |= MG_F_CLOSE_IMMEDIATELY;
    //break;
  }
  case MG_EV_CLOSE:
    break;
  default:
    LOG(LL_INFO, ("HTTP EVENT"));
  }

  (void)cb_arg;
}
*/

static void post_sensor_data()
{
  const char *url = mgos_sys_config_get_push_url();
  if (url == NULL)
    return;

  char buf[200];
  sensor_data_to_json(buf, sizeof(buf));
  struct mg_mgr *mgr = mgos_get_mgr();
  struct mg_connection *conn = mg_connect_http(mgr, NULL, NULL, url, "Content-Type: application/json\r\n", buf);

  if (conn == NULL)
    LOG(LL_ERROR, ("Failed to connect to %s", url));
}

uint8_t getCRC(uint8_t *data)
{
  uint8_t crc = 0;
  for (uint8_t x = 1; x < 8; x++)
    crc += data[x];
  crc = 0xFF - crc;
  crc++;
  return crc;
}

void mhz19c_build_command(uint8_t command, uint16_t data, uint8_t *cmd_buffer)
{
  /* values for conversions */
  uint8_t high = (data >> 8) & 0xFF;
  uint8_t low = data & 0xFF;
  memset(cmd_buffer, 0, MHZ19_DATA_LEN);
  cmd_buffer[0] = 0xFF;    ///(0xFF) 255/FF means 'any' address (where the sensor is located)
  cmd_buffer[1] = 0x01;    // set  register (0x01) arbitrary byte number
  cmd_buffer[2] = command; // assign command value

  switch (command)
  {
  case MHZ19C_CMD_ABC_MODE:
    cmd_buffer[3] = low;
    break;
  case MHZ19C_CMD_CALIBRATE:
    cmd_buffer[6] = high;
    break;
  case MHZ19C_CMD_READ_CO2:
    break;
  }

  /* set checksum */
  cmd_buffer[8] = getCRC(cmd_buffer);
}

void mhz19c_receive_response(uint8_t *data)
{
  memset(data, 0, MHZ19_DATA_LEN);
  mgos_uart_read(UART_NO, data, MHZ19_DATA_LEN);
  LOG(LL_INFO, ("mhz19c receive: %02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X\n", data[0], data[1], data[2], data[3], data[4], data[5], data[6], data[7], data[8]));
  uint8_t crc = getCRC(data);
  if (data[8] != crc)
    LOG(LL_WARN, ("***** CRC error *****\n"));
}

void mhz19c_send_command(uint8_t command, uint16_t data)
{
  uint8_t cmd_buffer[MHZ19_DATA_LEN];
  mhz19c_build_command(command, data, cmd_buffer);

  LOG(LL_INFO, ("mhz19c send: %02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X\n", cmd_buffer[0], cmd_buffer[1], cmd_buffer[2], cmd_buffer[3], cmd_buffer[4], cmd_buffer[5], cmd_buffer[6], cmd_buffer[7], cmd_buffer[8]));
  mgos_uart_flush(UART_NO);
  mgos_uart_write(UART_NO, cmd_buffer, MHZ19_DATA_LEN);
  mgos_uart_flush(UART_NO);
}

bool mhz19c_init()
{
  struct mgos_uart_config ucfg;
  mgos_uart_config_set_defaults(UART_NO, &ucfg);
  ucfg.baud_rate = 9600;
  ucfg.num_data_bits = 8;
  ucfg.parity = MGOS_UART_PARITY_NONE;
  ucfg.stop_bits = MGOS_UART_STOP_BITS_1;
  if (!mgos_uart_configure(UART_NO, &ucfg))
    return false;

  //mgos_uart_set_dispatcher(UART_NO, mhz19c_data_receiver, NULL);
  mgos_uart_set_rx_enabled(UART_NO, true);
  return true;
}

int dht_thread(struct pt *pt, uint16_t ticks)
{
  static uint16_t timer;
  timer += ticks;

  PT_BEGIN(pt);

  while (1)
  {
    float t = mgos_dht_get_temp(s_dht);
    float h = mgos_dht_get_humidity(s_dht);

    if (!isnan(h) && !isnan(t))
    {
      temperature = t;
      humidity = h;
      //LOG(LL_DEBUG, ("temperature: %f *C humidity: %f %%\n", t, h));
    }

    // sleep 1 second
    timer = 0;
    PT_WAIT_UNTIL(pt, timer >= 1000);
  }

  PT_END(pt);
}

int display_thread(struct pt *pt, uint16_t ticks)
{
  static uint16_t timer;
  timer += ticks;

  PT_BEGIN(pt);

  while (1)
  {
    LOG(LL_DEBUG, ("temperature: %d *C humidity: %d %% ppm: %d\n", temperature, humidity, ppm));
    timer = 0;
    PT_WAIT_UNTIL(pt, timer >= 1000);
  }

  PT_END(pt);
}

int mhz19c_measure_thread(struct pt *pt, uint16_t ticks)
{
  static uint16_t timer;
  timer += ticks;

  static uint8_t response[MHZ19_DATA_LEN];

  PT_BEGIN(pt);

  // disable auto calibration
  mhz19c_send_command(MHZ19C_CMD_ABC_MODE, MHZ19C_ABC_MODE_OFF);
  //timer = 0;
  //PT_WAIT_UNTIL(pt, timer >= 100);
  mhz19c_receive_response(response);

  while (1)
  {
    mhz19c_send_command(MHZ19C_CMD_READ_CO2, 0);
    PT_WAIT_UNTIL(pt, mgos_uart_read_avail(UART_NO) == MHZ19_DATA_LEN);
    mhz19c_receive_response(response);

    if (response[1] == MHZ19C_CMD_READ_CO2)
    {
      ppm = (response[2] << 8) | response[3];
      ppm += 20; // co2 correction
    }
    timer = 0;
    PT_WAIT_UNTIL(pt, timer >= 1000);
  }

  PT_END(pt);
}

int mhz19c_calibrate_thread(struct pt *pt, uint16_t ticks)
{
  static uint16_t timer;
  timer += ticks;

  PT_BEGIN(pt);

  calibrating = 1;

  mgos_gpio_write(mgos_sys_config_get_mhz19c_hd_pin(), LOW);
  mgos_gpio_set_mode(mgos_sys_config_get_mhz19c_hd_pin(), MGOS_GPIO_MODE_OUTPUT);

  timer = 0;
  PT_WAIT_UNTIL(pt, timer >= 7000);

  mgos_gpio_write(mgos_sys_config_get_mhz19c_hd_pin(), HIGH);

  calibrating = 0;

  PT_EXIT(pt);
  PT_END(pt);
}

int button_thread(struct pt *pt, uint16_t ticks)
{
  static uint16_t timer;
  timer += ticks;

  PT_BEGIN(pt);

  while (1)
  {
    // wait until button was pressed
    PT_WAIT_UNTIL(pt, !mgos_gpio_read(mgos_sys_config_get_button_pin()));

    // wait until button was released
    PT_WAIT_UNTIL(pt, mgos_gpio_read(mgos_sys_config_get_button_pin()));

    // start calibration
    LOG(LL_DEBUG, ("***** CALIBRATE *****\n"));
    PT_INIT(&pt_mhz19c_calibrate);
    PT_SPAWN(pt, &pt_mhz19c_calibrate, mhz19c_calibrate_thread(&pt_mhz19c_calibrate, ticks));
    LOG(LL_DEBUG, ("***** CALIBRATION DONE *****\n"));
  }

  PT_END(pt);
}

int mqtt_push_thread(struct pt *pt, uint16_t ticks)
{
  static uint16_t timer;
  timer += ticks;

  PT_BEGIN(pt);

  while (1)
  {
    char topic[100];
    snprintf(topic, sizeof(topic), "losant/%s/state", mgos_sys_config_get_mqtt_client_id());
    
    char buf[200];
    sensor_data_to_json(buf, sizeof(buf));
    
    uint16_t res = mgos_mqtt_pubf(topic, 0, false, "{data:%s}", buf);
    LOG(LL_INFO, ("mqtt success: %d\n", res));

    timer = 0;
    PT_WAIT_UNTIL(pt, timer >= mgos_sys_config_get_push_interval() * 1000);
  }

  PT_END(pt);
}

int http_push_thread(struct pt *pt, uint16_t ticks)
{
  static uint16_t timer;
  timer += ticks;

  PT_BEGIN(pt);

  while (1)
  {
    post_sensor_data();
    timer = 0;
    PT_WAIT_UNTIL(pt, timer >= mgos_sys_config_get_push_interval() * 1000);
  }

  PT_END(pt);
}

static void main_loop(void *arg)
{
  uint64_t now = mgos_uptime_micros() / 1000;
  uint64_t ticks = now - last_tick;
  last_tick = now;

  // schedule threads
  dht_thread(&pt_dht, ticks);
  mhz19c_measure_thread(&pt_mhz19c_measure, ticks);
  display_thread(&pt_display, ticks);
  button_thread(&pt_button, ticks);
  if (mgos_sys_config_get_push_url() != NULL)
    http_push_thread(&pt_http_push, ticks);
  if (mgos_sys_config_get_mqtt_server() != NULL)
    mqtt_push_thread(&pt_mqtt_push, ticks);
}

enum mgos_app_init_result mgos_app_init(void)
{
  //read_config();

  // init dht
  if ((s_dht = mgos_dht_create(mgos_sys_config_get_dht_pin(), DHT11)) == NULL)
    return MGOS_APP_INIT_ERROR;

  // init mhz19c
  if (!mhz19c_init())
    return MGOS_APP_INIT_ERROR;

  // init threads
  PT_INIT(&pt_dht);
  PT_INIT(&pt_mhz19c_measure);
  PT_INIT(&pt_button);
  PT_INIT(&pt_http_push);
  PT_INIT(&pt_mqtt_push);

  // http handler
  mgos_register_http_endpoint("/api/sensor/values", get_sensor_data, NULL);

  // mqtt subscription
  /*
  char topic[100];
  snprintf(topic, sizeof(topic), "/miefalarm/%s/values", mgos_sys_config_get_device_id());
  mgos_mqtt_sub(topic, mqtt_handler, NULL);
  */

  // start main
  mgos_set_timer(MAIN_LOOP_TICK_MS, MGOS_TIMER_REPEAT, main_loop, NULL);

  return MGOS_APP_INIT_SUCCESS;
}