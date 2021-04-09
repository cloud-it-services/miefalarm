#include "mgos.h"
#include "mgos_dht.h"
#include "mgos_adc.h"
#include "pt/pt.h"

static uint8_t operation_mode = 0; // 0 - Messung | 1 - Calibrierung
static struct mgos_dht *s_dht = NULL;
static unsigned long global_millis = 0;
static uint8_t adc_attenuation = 3; // ADC Dämpfungsfaktor
static uint8_t temperature = 1;
static uint8_t humidity = 1;
static double r0 = 1;
static uint32_t ppm = 0;

// threads
static struct pt pt_dht, pt_mq135, pt_button;

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

static float a = 121.4517;
static float b = -2.78054;
int mq135_thread(struct pt *pt, uint16_t ticks)
{
  static uint16_t timer;
  timer += ticks;

  PT_BEGIN(pt);

  while (1)
  {
    uint8_t i = 0;
    int measurements[10] = {};
    while (i < 10)
    {
      int val = mgos_adc_read(mgos_sys_config_get_mq135_pin());
      // ggf. ADC Dämpfungsfaktor anpassen
      if (val < 1000 || val > 3000)
      {
        if (val < 1000 && adc_attenuation)
          adc_attenuation--;
        else if (val > 3000 && adc_attenuation < 3)
          adc_attenuation++;

        LOG(LL_INFO, ("attenuation: %d measured val: \n", adc_attenuation, val));
        esp32_set_channel_attenuation(mgos_sys_config_get_mq135_pin(), adc_attenuation);
        mgos_adc_enable(mgos_sys_config_get_mq135_pin());
        continue;
      }

      measurements[i++] = val;
    };
    int sum = 0;
    while (i--)
      sum += measurements[i];
    int meanVal = sum / 10;

    float rs = (4095.0 / meanVal - 1);

    //LOG(LL_INFO, ("sum: %d\n", sum));
    //LOG(LL_INFO, ("mean: %d\n", meanVal));
    //LOG(LL_INFO, ("rs: %f\n", rs));

    if (operation_mode == 0)
    {
      float rs_scaling_factor = (1.6979 - 0.012 * temperature - 0.00612 * humidity);
      float rs_corrected = rs / rs_scaling_factor;
      ppm = a * pow((rs_corrected / r0), b);
      LOG(LL_INFO, ("temperature: %d *C humidity: %d %% attenuation: %d ppm: %d\n", temperature, humidity, adc_attenuation, ppm));
    }
    else if (operation_mode == 1)
    {
      r0 = rs * pow((a / 400.0), (1.0 / b));
      LOG(LL_INFO, ("temperature: %d *C humidity: %d %% attenuation: %d r0: %f\n", temperature, humidity, adc_attenuation, r0));
    }

    // sleep 1 second
    timer = 0;
    PT_WAIT_UNTIL(pt, timer >= 1000);
  }

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

    // switch operation mode
    operation_mode = (operation_mode + 1) % 2;
    if (operation_mode)
      LOG(LL_INFO, ("***** CALIBRATE *****\n"));
    else
      LOG(LL_INFO, ("***** MEASURE *****\n"));
  }

  PT_END(pt);
}

static void main_loop(void *arg)
{
  while (1)
  {
    unsigned long ts = mgos_uptime_micros() / 1000;
    unsigned long ticks = ts - global_millis;
    global_millis = ts;

    // start threads
    dht_thread(&pt_dht, ticks);
    mq135_thread(&pt_mq135, ticks);
    button_thread(&pt_button, ticks);

    // feed watchdog
    mgos_wdt_feed();
  }
}

/*
static void on_boot_button_pressed(int pin, void *arg)
{
  LOG(LL_INFO, ("***** BUTTON PRESSED %d\n",pin));
  if (pin == mgos_sys_config_get_button_pin())
  {
    //LOG(LL_INFO, ("***** BUTTON PRESSED\r\n"));
  }
}
*/

enum mgos_app_init_result mgos_app_init(void)
{
  // init dht
  if ((s_dht = mgos_dht_create(mgos_sys_config_get_dht_pin(), DHT11)) == NULL)
  {
    return MGOS_APP_INIT_ERROR;
  }

  // init mq135
  esp32_set_channel_attenuation(mgos_sys_config_get_mq135_pin(), adc_attenuation);
  mgos_adc_enable(mgos_sys_config_get_mq135_pin());

  // init boot button
  //mgos_gpio_set_button_handler(mgos_sys_config_get_button_pin(), MGOS_GPIO_PULL_UP, MGOS_GPIO_INT_EDGE_NEG, 50, on_boot_button_pressed, NULL);

  // init threads
  PT_INIT(&pt_dht);
  PT_INIT(&pt_mq135);
  PT_INIT(&pt_button);

  // start main loop
  mgos_set_timer(0, false, main_loop, NULL);

  return MGOS_APP_INIT_SUCCESS;
}