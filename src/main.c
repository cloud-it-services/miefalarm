#include "mgos.h"
#include "mgos_dht.h"
#include "mgos_uart.h"
#include "mgos_mqtt.h"
#include "pt/pt.h"
#include "uECC/uECC.h"
#include "uECC/uECC.c"
#ifdef MGOS_HAVE_HTTP_SERVER
#include "mgos_http_server.h"
#endif

#include "mbedtls/sha256.h" /* SHA-256 only */
//#include "mbedtls/md.h"     /* generic interface */
//#include "mbedtls/debug.h"
//#include "mbedtls/platform.h"

// device pins
#define PIN_DHT 21
#define PIN_MHZ19C_HD 33
#define PIN_LED_RED 27
#define PIN_LED_YELLOW 26
#define PIN_LED_GREEN 25
#define PIN_BUTTON_CALIBRATE 0
#define UART_NO 2
#if CS_PLATFORM == CS_P_ESP8266
    #define PIN_DHT 5
    #define PIN_MHZ19C_HD 10
    #define PIN_LED_RED 12
    #define PIN_LED_YELLOW 14
    #define PIN_LED_GREEN 10
    #define PIN_BUTTON_CALIBRATE 0
    #define UART_NO 0
#endif

#define MHZ19_DATA_LEN 9
#define MAIN_LOOP_TICK_MS 1
#define HIGH 0x1
#define LOW 0x0

// mhz19c commands
#define MHZ19C_CMD_ABC_MODE 0x79
#define MHZ19C_CMD_READ_CO2 0x86
#define MHZ19C_CMD_CALIBRATE 0x87
#define MHZ19C_ABC_MODE_ON 0xA0  // ABC mode on
#define MHZ19C_ABC_MODE_OFF 0x00 // ABC mode off

// ecdsa parameter
#define HASH_LEN 32
#define CURVE uECC_secp256k1()
#define PRIVATE_KEY_LEN uECC_curve_private_key_size(CURVE)
#define PUBLIC_KEY_LEN uECC_curve_public_key_size(CURVE)
#define SIGNATURE_LEN uECC_curve_public_key_size(CURVE)

// threads
static struct pt pt_dht;
static struct pt pt_display;
static struct pt pt_button;
static struct pt pt_mhz19c_calibrate;
static struct pt pt_mhz19c_measure;
static struct pt pt_data_push;
//static struct pt pt_led;

static struct mgos_dht *s_dht = NULL;
static uint64_t last_tick = 0;
static uint8_t temperature = 0;
static uint8_t humidity = 0;
static uint16_t ppm = 0;
static uint8_t calibrating = 0;

/*
static void saveFile(char *filename, char *data)
{
    FILE *fp = fopen(filename, "w+");
    if (fp == NULL)
        return;
    fprintf(fp, "%s", data);
    fclose(fp);
}

static char *loadFile(char *filename, char *buffer)
{
    FILE *fp = fopen(filename, "r");
    if (fp == NULL)
    {
        LOG(LL_DEBUG, ("%s not found\n", filename));
        return;
    }
    fseek(fp, 0, SEEK_END);
    uint16_t fSize = ftell(fp);
    rewind(fp);
    fread(buffer, 1, fSize, fp);
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
    return json_printf(out, "{temperature:%d,humidity:%d,co2:%d,calibrating:%B}", t->temperature, t->humidity, t->co2, t->calibrating);
}

static void sensor_data_to_json(char *buf, uint8_t len)
{
    struct json_out out = JSON_OUT_BUF(buf, len);
    sensor_data sd = {temperature, humidity, ppm, calibrating};
    json_printf(&out, "%M", json, &sd);
}

static void base64_encode(uint8_t *input_str, int len_str, uint8_t *encoded)
{
    char char_set[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789_-";
    int index, no_of_bits = 0, val = 0, count = 0, temp;
    int i, j, k = 0;
    for (i = 0; i < len_str; i += 3)
    {
        val = 0, count = 0, no_of_bits = 0;
        for (j = i; j < len_str && j <= i + 2; j++)
        {
            val = val << 8;
            val = val | input_str[j];
            count++;
        }
        no_of_bits = count * 8;
        while (no_of_bits != 0)
        {
            if (no_of_bits >= 6)
            {
                temp = no_of_bits - 6;
                index = (val >> temp) & 63;
                no_of_bits -= 6;
            }
            else
            {
                temp = 6 - no_of_bits;
                index = (val << temp) & 63;
                no_of_bits = 0;
            }
            encoded[k++] = char_set[index];
        }
    }
    encoded[k] = '\0';
}

static void base64_decode(uint8_t *encoded, int len_str, uint8_t *decoded)
{
    int i, j, k = 0;
    int num = 0;
    int count_bits = 0;
    for (i = 0; i < len_str; i += 4)
    {
        num = 0, count_bits = 0;
        for (j = 0; j < 4; j++)
        {
            if (encoded[i + j] != '=')
            {
                num = num << 6;
                count_bits += 6;
            }
            if (encoded[i + j] >= 'A' && encoded[i + j] <= 'Z')
                num = num | (encoded[i + j] - 'A');
            else if (encoded[i + j] >= 'a' && encoded[i + j] <= 'z')
                num = num | (encoded[i + j] - 'a' + 26);
            else if (encoded[i + j] >= '0' && encoded[i + j] <= '9')
                num = num | (encoded[i + j] - '0' + 52);
            else if (encoded[i + j] == '_')
                num = num | 62;
            else if (encoded[i + j] == '-')
                num = num | 63;
        }
        while (count_bits != 0)
        {
            count_bits -= 8;
            decoded[k++] = (num >> count_bits) & 255;
        }
    }
    decoded[k] = '\0';
}

int RNG(uint8_t *dest, unsigned size)
{
    while (size)
    {
        *dest = mgos_rand_range(0, 255);
        ++dest;
        --size;
    }
    return 1;
}

static uint8_t create_signature(char *message, uint8_t *sig)
{

    // Load private key
    const unsigned char *message_buffer = (const unsigned char *) message;
    const char *privKeyBase64 = mgos_sys_config_get_auth_private_key();
    uint8_t privKey[PRIVATE_KEY_LEN];
    base64_decode((uint8_t*) privKeyBase64, strlen(privKeyBase64), privKey);

    uint8_t hash[HASH_LEN];
    mbedtls_sha256(message_buffer, strlen(message), hash, 0);

    char debugBuf[2*HASH_LEN];
    char *ptr = &debugBuf[0];
    for (size_t i = 0; i < HASH_LEN; i++) {
        ptr += sprintf(ptr, "%02X", hash[i]);
    }
    LOG(LL_DEBUG, ("*** message: %s", message));
    LOG(LL_DEBUG, ("*** hash: %s", debugBuf));

    // Create signature
    uECC_set_rng(&RNG);
    const struct uECC_Curve_t *curve = CURVE;
    if (!uECC_sign(privKey, hash, HASH_LEN, sig, curve))
    {
        LOG(LL_ERROR, ("signature creation failed"));
        return 1;
    }
    return 0;
}

static void create_keys()
{
    LOG(LL_INFO, ("generate keys"));

    uint8_t priv[PRIVATE_KEY_LEN];
    uint8_t pub[PUBLIC_KEY_LEN];
    uint8_t privKeyBase64[100];
    uint8_t pubKeyBase64[100];

    uECC_set_rng(RNG);
    const struct uECC_Curve_t *curve = CURVE;
    uECC_make_key(pub, priv, curve);

    base64_encode(priv, sizeof(priv), privKeyBase64);
    base64_encode(pub, sizeof(pub), pubKeyBase64);
    LOG(LL_INFO, ("*** private key: %s", privKeyBase64));
    LOG(LL_INFO, ("*** public key: %s", pubKeyBase64));

    mgos_sys_config_set_auth_private_key((const char*) privKeyBase64);
    mgos_sys_config_set_auth_public_key((const char*) pubKeyBase64);

    char *err = NULL;
    save_cfg(&mgos_sys_config, &err); /* Writes conf9.json */
    LOG(LL_DEBUG, ("Saving configuration: %s\n", err ? err : "no error"));
    free(err);
}

static void send_response(struct mg_connection *c, char *data)
{
    mg_send_head(c, 200, strlen(data),
                 "Cache: no-transform, no-cache\r\n"
                 "Content-Type: application/json;charset=utf-8");
    mg_printf(c, "%s", data);
    c->flags |= MG_F_SEND_AND_CLOSE;
}

/**
 * HTTP API Handler
 */
static void http_handler_sensor_data(struct mg_connection *c, int ev, void *p, void *user_data)
{
    if (ev != MG_EV_HTTP_REQUEST)
        return;

    char data[200];
    char buf[sizeof(data) + 100];
    sensor_data_to_json(data, sizeof(data));
    snprintf(buf, sizeof(buf), "{\"data\":%s}", data);
    send_response(c, buf);
}

static void http_handler_auth_keys(struct mg_connection *c, int ev, void *p, void *user_data)
{
    if (ev != MG_EV_HTTP_REQUEST)
        return;

    create_keys();

    char buf[200];
    snprintf(buf, sizeof(buf), "{\"public_key\":\"%s\"}", mgos_sys_config_get_auth_public_key());
    send_response(c, buf);
}
/**
 * HTTP API Handler END
 */

/*
static uint8_t post_sensor_data(const char *url, bool sign_data)
{
    char data[200];
    sensor_data_to_json(data, sizeof(data));

    char pl[sizeof(data) + 100];
    if (sign_data)
    {
        uint8_t sig[SIGNATURE_LEN];
        uint8_t sigBase64[100];
        create_signature(data, sig);
        base64_encode(sig, strlen((char*) sig), sigBase64);
        snprintf(pl, sizeof(pl), "{\"data\":%s,\"signature\":\"%hhn\",\"id\":\"%ld\"}", data, sigBase64, (long)mgos_uptime_micros());
    } else {
        snprintf(pl, sizeof(pl), "{\"data\":%s", data);
    }

    LOG(LL_DEBUG, ("*** http post pl: %s", pl));

    struct mg_mgr *mgr = mgos_get_mgr();
    struct mg_connection *conn = mg_connect_http(mgr, NULL, NULL, url, "Content-Type: application/json\r\n", pl);
    return conn != NULL ? 0 : 1;
}
*/

static uint8_t post_sensor_data(const char *url, bool sign_data)
{
    char data[200];
    sensor_data_to_json(data, sizeof(data));

    char pl[sizeof(data) + 100];
    if (sign_data)
    {
        uint8_t sig[SIGNATURE_LEN];
        create_signature(data, sig);
        char sigBuf[2*sizeof(sig)];
        char *ptr = &sigBuf[0];
        for (size_t i = 0; i < sizeof(sig); i++) {
            ptr += sprintf(ptr, "%02X", sig[i]);
        }
        snprintf(pl, sizeof(pl), "{\"data\":%s,\"signature\":\"%s\",\"id\":\"%ld\"}", data, sigBuf, (long)mgos_uptime_micros());
    } else {
        snprintf(pl, sizeof(pl), "{\"data\":%s", data);
    }

    LOG(LL_DEBUG, ("*** http post pl: %s", pl));

    struct mg_mgr *mgr = mgos_get_mgr();
    struct mg_connection *conn = mg_connect_http(mgr, NULL, NULL, url, "Content-Type: application/json\r\n", pl);
    return conn != NULL ? 0 : 1;
}

void set_leds() {

    if (ppm > mgos_sys_config_get_co2_ppm_critical()) {
        LOG(LL_INFO, ("*** RED LED ***"));
        mgos_gpio_write(PIN_LED_RED, HIGH);
        mgos_gpio_write(PIN_LED_YELLOW, LOW);
        mgos_gpio_write(PIN_LED_GREEN, LOW);
    } else if (ppm > mgos_sys_config_get_co2_ppm_warn()) {
        LOG(LL_INFO, ("*** YELLOW LED ***"));
        mgos_gpio_write(PIN_LED_RED, LOW);
        mgos_gpio_write(PIN_LED_YELLOW, HIGH);
        mgos_gpio_write(PIN_LED_GREEN, LOW);
    } else {
        LOG(LL_INFO, ("*** GREEN LED ***"));
        mgos_gpio_write(PIN_LED_RED, LOW);
        mgos_gpio_write(PIN_LED_YELLOW, LOW);
        mgos_gpio_write(PIN_LED_GREEN, HIGH);
    }
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

uint8_t mhz19c_receive_response(uint8_t *data)
{
    memset(data, 0, MHZ19_DATA_LEN);
    mgos_uart_read(UART_NO, data, MHZ19_DATA_LEN);
    LOG(LL_INFO, ("*** mhz19c receive: %02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X\n", data[0], data[1], data[2], data[3], data[4], data[5], data[6], data[7], data[8]));
    uint8_t crc = getCRC(data);
    if (data[8] != crc) {
        LOG(LL_WARN, ("***** CRC error *****\n"));
        return 1;
    }
    return 0;
}

void mhz19c_send_command(uint8_t command, uint16_t data)
{
    uint8_t cmd_buffer[MHZ19_DATA_LEN];
    mhz19c_build_command(command, data, cmd_buffer);

    LOG(LL_INFO, ("*** mhz19c send: %02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X\n", cmd_buffer[0], cmd_buffer[1], cmd_buffer[2], cmd_buffer[3], cmd_buffer[4], cmd_buffer[5], cmd_buffer[6], cmd_buffer[7], cmd_buffer[8]));
    while (mgos_uart_write_avail(UART_NO) < MHZ19_DATA_LEN)
    mgos_uart_flush(UART_NO);
    mgos_uart_write(UART_NO, cmd_buffer, MHZ19_DATA_LEN);
}

bool mhz19c_init()
{
    LOG(LL_INFO, ("*** init uart %d", UART_NO));
    struct mgos_uart_config ucfg;
    mgos_uart_config_set_defaults(UART_NO, &ucfg);
    ucfg.baud_rate = 9600;
    ucfg.num_data_bits = 8;
    ucfg.parity = MGOS_UART_PARITY_NONE;
    ucfg.stop_bits = MGOS_UART_STOP_BITS_1;
    if (!mgos_uart_configure(UART_NO, &ucfg)) {
        LOG(LL_ERROR, ("configure uart failed"));
        return false;
    }

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
        PT_WAIT_UNTIL(pt, timer >= 10000);
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
        LOG(LL_INFO, ("temperature: %d *C humidity: %d %% ppm: %d\n", temperature, humidity, ppm));
        timer = 0;
        PT_WAIT_UNTIL(pt, timer >= 10000);
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
    timer = 0;
    PT_WAIT_UNTIL(pt, (mgos_uart_read_avail(UART_NO) >= MHZ19_DATA_LEN) || timer >= 1000);
    mhz19c_receive_response(response);

    while (1)
    {
        mhz19c_send_command(MHZ19C_CMD_READ_CO2, 0);
        timer = 0;
        PT_WAIT_UNTIL(pt, (mgos_uart_read_avail(UART_NO) >= MHZ19_DATA_LEN) || timer >= 1000);
        if (!mhz19c_receive_response(response) && response[1] == MHZ19C_CMD_READ_CO2) {
            ppm = (response[2] << 8) | response[3];
            ppm += 20; // co2 correction
            set_leds();
        }
        PT_WAIT_UNTIL(pt, timer >= 10000);
    }

    PT_END(pt);
}

int mhz19c_calibrate_thread(struct pt *pt, uint16_t ticks)
{
    static uint16_t timer;
    timer += ticks;

    PT_BEGIN(pt);

    calibrating = 1;

    mgos_gpio_set_mode(PIN_MHZ19C_HD, MGOS_GPIO_MODE_OUTPUT);
    mgos_gpio_write(PIN_MHZ19C_HD, LOW);

    timer = 0;
    PT_WAIT_UNTIL(pt, timer >= 7000);

    mgos_gpio_set_mode(PIN_MHZ19C_HD, MGOS_GPIO_MODE_INPUT);

    calibrating = 0;

    PT_EXIT(pt);
    PT_END(pt);
}

/*
int led_thread(struct pt *pt, uint16_t ticks)
{
    static uint16_t timer;
    timer += ticks;

    PT_BEGIN(pt);

    while (1)
    {
        if (ppm > mgos_sys_config_get_co2_ppm_critical()) {
            LOG(LL_INFO, ("*** RED LED ***"));
            mgos_gpio_write(mgos_sys_config_get_led_r_pin(), HIGH);
            mgos_gpio_write(mgos_sys_config_get_led_y_pin(), LOW);
            mgos_gpio_write(mgos_sys_config_get_led_g_pin(), LOW);
        } else if (ppm > mgos_sys_config_get_co2_ppm_warn()) {
            LOG(LL_INFO, ("*** YELLOW LED ***"));
            mgos_gpio_write(mgos_sys_config_get_led_r_pin(), LOW);
            mgos_gpio_write(mgos_sys_config_get_led_y_pin(), HIGH);
            mgos_gpio_write(mgos_sys_config_get_led_g_pin(), LOW);
        } else {
            LOG(LL_INFO, ("*** GREEN LED ***"));
            mgos_gpio_write(mgos_sys_config_get_led_r_pin(), LOW);
            mgos_gpio_write(mgos_sys_config_get_led_y_pin(), LOW);
            mgos_gpio_write(mgos_sys_config_get_led_g_pin(), HIGH);
        }

        timer = 0;
        PT_WAIT_UNTIL(pt, timer >= 1000);
    }

    PT_END(pt);
}
*/

int button_thread(struct pt *pt, uint16_t ticks)
{
    static uint16_t timer;
    timer += ticks;

    PT_BEGIN(pt);

    while (1)
    {
        // wait until button was pressed
        PT_WAIT_UNTIL(pt, !mgos_gpio_read(PIN_BUTTON_CALIBRATE));

        // wait until button was released
        PT_WAIT_UNTIL(pt, mgos_gpio_read(PIN_BUTTON_CALIBRATE));

        // start calibration
        LOG(LL_DEBUG, ("***** CALIBRATE *****\n"));
        PT_INIT(&pt_mhz19c_calibrate);
        PT_SPAWN(pt, &pt_mhz19c_calibrate, mhz19c_calibrate_thread(&pt_mhz19c_calibrate, ticks));
        LOG(LL_DEBUG, ("***** CALIBRATION DONE *****\n"));
    }

    PT_END(pt);
}

int data_push_thread(struct pt *pt, uint16_t ticks)
{
    static uint16_t timer;
    timer += ticks;

    PT_BEGIN(pt);

    while (1)
    {
        static char buf[200];
        sensor_data_to_json(buf, sizeof(buf));

        // mqtt
        if (mgos_sys_config_get_mqtt_server() != NULL)
        {
            char topic[100];
            snprintf(topic, sizeof(topic), mgos_sys_config_get_mqtt_topic(), mgos_sys_config_get_mqtt_client_id());
            uint16_t res = mgos_mqtt_pubf(topic, 0, false, "{data:%s}", buf);
            LOG(LL_INFO, ("*** mqtt success: %d", res));
            timer = 0;
            PT_WAIT_UNTIL(pt, timer >= 1);
        }

        // http
        const char *http_push_url = mgos_sys_config_get_push_url();
        if (http_push_url != NULL)
        {
            uint8_t res = post_sensor_data(http_push_url, false);
            LOG(LL_INFO, ("*** http push success: %d", res));
            timer = 0;
            PT_WAIT_UNTIL(pt, timer >= 1);
        }

        // iot cloud
        if (mgos_sys_config_get_cloud_enable())
        {
            char url[256];
            snprintf(url, sizeof(url), "https://miefalarm.de/api/devices/%s/state", mgos_sys_config_get_auth_public_key());
            uint8_t res = post_sensor_data(url, true);
            LOG(LL_INFO, ("*** cloud push success: %d", res));
            timer = 0;
            PT_WAIT_UNTIL(pt, timer >= 1);
        }

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
    mgos_wdt_set_timeout(10);
    dht_thread(&pt_dht, ticks);
    mhz19c_measure_thread(&pt_mhz19c_measure, ticks);
    display_thread(&pt_display, ticks);
    button_thread(&pt_button, ticks);
    //led_thread(&pt_led, ticks);
    data_push_thread(&pt_data_push, ticks);
    mgos_wdt_feed();
}

enum mgos_app_init_result mgos_app_init(void)
{
    LOG(LL_INFO, ("CS_PLATFORM: %d", CS_PLATFORM));
    LOG(LL_INFO, ("PRIVATE_KEY_LEN: %d", PRIVATE_KEY_LEN));
    LOG(LL_INFO, ("PUBLIC_KEY_LEN: %d", PUBLIC_KEY_LEN));
    LOG(LL_INFO, ("SIGNATURE_LEN: %d", SIGNATURE_LEN));

    //read_config();

    // init dht
    if ((s_dht = mgos_dht_create(PIN_DHT, DHT11)) == NULL) {
        LOG(LL_ERROR, ("init dht11 failed"));
        return MGOS_APP_INIT_ERROR;
    }

    //LOG(LL_DEBUG, ("--- 1 ---"));

    // init mhz19c
    if (!mhz19c_init()) {
        LOG(LL_ERROR, ("init mhz19c failed"));
        return MGOS_APP_INIT_ERROR;
    }

    //LOG(LL_DEBUG, ("--- 2 ---"));

    // init LED pins
    mgos_gpio_setup_output(PIN_LED_RED, true);
    mgos_gpio_setup_output(PIN_LED_YELLOW, true);
    mgos_gpio_setup_output(PIN_LED_GREEN, true);
    mgos_gpio_set_mode(PIN_LED_RED, MGOS_GPIO_MODE_OUTPUT);
    mgos_gpio_set_mode(PIN_LED_YELLOW, MGOS_GPIO_MODE_OUTPUT);
    mgos_gpio_set_mode(PIN_LED_GREEN, MGOS_GPIO_MODE_OUTPUT);
    mgos_gpio_write(PIN_LED_RED, HIGH);
    mgos_gpio_write(PIN_LED_YELLOW, HIGH);
    mgos_gpio_write(PIN_LED_GREEN, HIGH);

    //LOG(LL_DEBUG, ("--- 3 ---"));

    // create keys
    if (mgos_sys_config_get_auth_public_key() == NULL)
        create_keys();

    //LOG(LL_DEBUG, ("--- 4 ---"));

    // init threads
    PT_INIT(&pt_dht);
    PT_INIT(&pt_mhz19c_measure);
    PT_INIT(&pt_button);
    PT_INIT(&pt_data_push);
    //PT_INIT(&pt_led);

    //LOG(LL_DEBUG, ("--- 5 ---"));

    #ifdef MGOS_HAVE_HTTP_SERVER
    mgos_register_http_endpoint("/api/sensor/state", http_handler_sensor_data, NULL);
    mgos_register_http_endpoint("/api/auth/keys/create", http_handler_auth_keys, NULL); // TODO: restrict this path to authenticated user via acl (or using digest authentication)
    #endif

    //LOG(LL_DEBUG, ("--- 6 ---"));

    // start main
    mgos_wdt_enable();
    mgos_set_timer(MAIN_LOOP_TICK_MS, MGOS_TIMER_REPEAT, main_loop, NULL);

    //LOG(LL_DEBUG, ("--- 7 ---"));

    return MGOS_APP_INIT_SUCCESS;
}