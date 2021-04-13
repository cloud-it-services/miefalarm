#include "mgos.h"
#include "mgos_dht.h"
#include "mgos_adc.h"
#include "pt/pt.h"

#define MEASURE_CYCLES 100
#define CALIBRATE_CYCLES 100
#define MAIN_LOOP_TICK_MS 1
#define CONFIG_FILENAME "mq135.conf"

const double a = 121.4517;
const double b = -2.78054;

static struct mgos_dht *s_dht = NULL;
static uint64_t last_tick = 0;
static uint8_t adc_attenuation = 3; // ADC Dämpfungsfaktor
static uint8_t temperature = 1;
static uint8_t humidity = 1;
static uint16_t ppm = 0;
static double r0 = 1;
static double rs = 0;

// threads
static struct pt pt_dht;
static struct pt pt_mq135_measure;
static struct pt pt_mq135_calibrate;
static struct pt pt_display;
static struct pt pt_button;

static void write_config()
{
  // store attenuation and r0 in filesystem
  FILE *fp = fopen(CONFIG_FILENAME, "w+");
  if (fp == NULL)
    return;

  //LOG(LL_INFO, ("write config: %d\n%.6f\n", adc_attenuation, r0));
  fprintf(fp, "%d\n%f\n", adc_attenuation, r0);
  fclose(fp);
}

static void read_config()
{
  FILE *fp = fopen(CONFIG_FILENAME, "r");
  if (fp == NULL)
  {
    LOG(LL_INFO, ("%s not found\n", CONFIG_FILENAME));
    return;
  }
  fscanf(fp, "%hhu\n", &adc_attenuation);
  LOG(LL_INFO, ("set adc_attenuation: %d\n", adc_attenuation));
  fscanf(fp, "%lf\n", &r0);
  LOG(LL_INFO, ("set r0: %f\n", r0));
  fclose(fp);
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
      //LOG(LL_INFO, ("temperature: %f *C humidity: %f %%\n", t, h));
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
    LOG(LL_INFO, ("temperature: %d *C humidity: %d %% attenuation: %d r0: %f rs: %f ppm: %d\n", temperature, humidity, adc_attenuation, r0, rs, ppm));
    timer = 0;
    PT_WAIT_UNTIL(pt, timer >= 1000);
  }

  PT_END(pt);
}

int mq135_measure_thread(struct pt *pt, uint16_t ticks)
{
  static uint16_t timer;
  timer += ticks;

  static uint16_t i;
  static uint8_t timeout;
  static float mean_voltage;
  static uint32_t measurements;

  PT_BEGIN(pt);

  while (1)
  {
    measurements = 0;
    for (i = 0; i < MEASURE_CYCLES; i++)
    {
      measurements += mgos_adc_read_voltage(mgos_sys_config_get_mq135_pin());
      timeout = mgos_rand_range(0, 10);
      timer = 0;
      PT_WAIT_UNTIL(pt, timer >= timeout);
    };
    mean_voltage = measurements / 1000.0 / MEASURE_CYCLES;
    LOG(LL_INFO, ("mean_voltage: %f\n", mean_voltage));
    rs = (5.0 / mean_voltage - 1) * 10000;
    rs /= 1.6979 - 0.012 * temperature - 0.00612 * humidity; // apply scaling factor
    ppm = a * pow((rs / r0), b);
  }

  PT_END(pt);
}

static void init_mq135()
{
  esp32_set_channel_attenuation(mgos_sys_config_get_mq135_pin(), adc_attenuation);
  mgos_adc_enable(mgos_sys_config_get_mq135_pin());
}

int mq135_calibrate_thread(struct pt *pt, uint16_t ticks)
{
  static uint16_t timer;
  timer += ticks;

  static uint16_t adc_raw_value;

  PT_BEGIN(pt);

  adc_attenuation = 3;
  init_mq135();

  while (1)
  {
    adc_raw_value = mgos_adc_read(mgos_sys_config_get_mq135_pin());
    if (adc_raw_value < 1500 && adc_attenuation)
    {
      adc_attenuation--;
      init_mq135();
      timer = 0;
      PT_WAIT_UNTIL(pt, timer >= 1000);
      continue;
    }

    r0 = rs * pow((a / 420.0), (1.0 / b));
    break;
  }

  write_config();

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
    LOG(LL_INFO, ("***** CALIBRATE *****\n"));
    PT_INIT(&pt_mq135_calibrate);
    PT_SPAWN(pt, &pt_mq135_calibrate, mq135_calibrate_thread(&pt_mq135_calibrate, ticks));
    LOG(LL_INFO, ("***** CALIBRATION DONE *****\n"));
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
  mq135_measure_thread(&pt_mq135_measure, ticks);
  display_thread(&pt_display, ticks);
  button_thread(&pt_button, ticks);
}

enum mgos_app_init_result mgos_app_init(void)
{
  // load config
  read_config();

  // init dht
  if ((s_dht = mgos_dht_create(mgos_sys_config_get_dht_pin(), DHT11)) == NULL)
    return MGOS_APP_INIT_ERROR;

  // init mq135
  init_mq135();

  // init threads
  PT_INIT(&pt_dht);
  PT_INIT(&pt_mq135_measure);
  PT_INIT(&pt_mq135_calibrate);
  PT_INIT(&pt_button);

  // start main
  mgos_set_timer(MAIN_LOOP_TICK_MS, MGOS_TIMER_REPEAT, main_loop, NULL);

  return MGOS_APP_INIT_SUCCESS;
}