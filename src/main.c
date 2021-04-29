#include "mgos.h"
#include "mgos_dht.h"
#include "mgos_uart.h"
#include "mgos_http_server.h"
#include "mgos_mqtt.h"
#include "pt/pt.h"
#include "uECC/uECC.h"
#include "uECC/uECC.c"

#define UART_NO 2
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

// signature
#define CHALLENGE_LEN 30
#define PUBLIC_KEY_LEN 40
#define PRIVATE_KEY_LEN 21
#define SIGNATURE_LEN 40 // for curve secp160r1 (twice as curve length)

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
    return json_printf(out, "{temperature: %d, humidity: %d, co2: %d, calibrating: %B}", t->temperature, t->humidity, t->co2, t->calibrating);
}

static void sensor_data_to_json(char *buf, uint8_t len)
{
    struct json_out out = JSON_OUT_BUF(buf, len);
    sensor_data sd = {temperature, humidity, ppm, calibrating};
    json_printf(&out, "%M", json, &sd);
}

/*
static void hex_decode(const char *in, size_t len, uint8_t *out)
{
    unsigned int i, t, hn, ln;
    for (t = 0, i = 0; i < len; i += 2, ++t)
    {
        hn = in[i] > '9' ? in[i] - 'A' + 10 : in[i] - '0';
        ln = in[i + 1] > '9' ? in[i + 1] - 'A' + 10 : in[i + 1] - '0';
        out[t] = (hn << 4) | ln;
    }
}

static void hex_encode(unsigned char *bytes, int bytesLength, char *dest)
{
    char lookup[16] = {'0', '1', '2', '3', '4', '5', '6', '7', '8', '9', 'A', 'B', 'C', 'D', 'E', 'F'};
    for (int i = 0; i < bytesLength; i++)
    {
        dest[2 * i] = lookup[(bytes[i] >> 4) & 0xF];
        dest[2 * i + 1] = lookup[bytes[i] & 0xF];
    }
    dest[2 * bytesLength] = 0;
}
*/

char *base64Encoder(uint8_t input_str[], int len_str)
{
    // Character set of base64 encoding scheme
    char char_set[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789_-";

    // Resultant string
    char *res_str = (char *)malloc(100 * sizeof(char));

    int index, no_of_bits = 0, padding = 0, val = 0, count = 0, temp;
    int i, j, k = 0;

    // Loop takes 3 characters at a time from
    // input_str and stores it in val
    for (i = 0; i < len_str; i += 3)
    {
        val = 0, count = 0, no_of_bits = 0;

        for (j = i; j < len_str && j <= i + 2; j++)
        {
            // binary data of input_str is stored in val
            val = val << 8;

            // (A + 0 = A) stores character in val
            val = val | input_str[j];

            // calculates how many time loop
            // ran if "MEN" -> 3 otherwise "ON" -> 2
            count++;
        }

        no_of_bits = count * 8;

        // calculates how many "=" to append after res_str.
        padding = no_of_bits % 3;

        // extracts all bits from val (6 at a time)
        // and find the value of each block
        while (no_of_bits != 0)
        {
            // retrieve the value of each block
            if (no_of_bits >= 6)
            {
                temp = no_of_bits - 6;

                // binary of 63 is (111111) f
                index = (val >> temp) & 63;
                no_of_bits -= 6;
            }
            else
            {
                temp = 6 - no_of_bits;

                // append zeros to right if bits are less than 6
                index = (val << temp) & 63;
                no_of_bits = 0;
            }
            res_str[k++] = char_set[index];
        }
    }

    // padding is done here
    for (i = 1; i <= padding; i++)
    {
        res_str[k++] = '=';
    }

    res_str[k] = '\0';

    return res_str;
}

char *base64Decoder(char encoded[], int len_str)
{
    char *decoded_string;

    decoded_string = (char *)malloc(sizeof(char) * 100);

    int i, j, k = 0;

    // stores the bitstream.
    int num = 0;

    // count_bits stores current
    // number of bits in num.
    int count_bits = 0;

    // selects 4 characters from
    // encoded string at a time.
    // find the position of each encoded
    // character in char_set and stores in num.
    for (i = 0; i < len_str; i += 4)
    {
        num = 0, count_bits = 0;
        for (j = 0; j < 4; j++)
        {
            // make space for 6 bits.
            if (encoded[i + j] != '=')
            {
                num = num << 6;
                count_bits += 6;
            }

            /* Finding the position of each encoded 
            character in char_set 
            and storing in "num", use OR 
            '|' operator to store bits.*/

            // encoded[i + j] = 'E', 'E' - 'A' = 5
            // 'E' has 5th position in char_set.
            if (encoded[i + j] >= 'A' && encoded[i + j] <= 'Z')
                num = num | (encoded[i + j] - 'A');

            // encoded[i + j] = 'e', 'e' - 'a' = 5,
            // 5 + 26 = 31, 'e' has 31st position in char_set.
            else if (encoded[i + j] >= 'a' && encoded[i + j] <= 'z')
                num = num | (encoded[i + j] - 'a' + 26);

            // encoded[i + j] = '8', '8' - '0' = 8
            // 8 + 52 = 60, '8' has 60th position in char_set.
            else if (encoded[i + j] >= '0' && encoded[i + j] <= '9')
                num = num | (encoded[i + j] - '0' + 52);

            // '+' occurs in 62nd position in char_set.
            else if (encoded[i + j] == '+')
                num = num | 62;

            // '/' occurs in 63rd position in char_set.
            else if (encoded[i + j] == '/')
                num = num | 63;

            // ( str[i + j] == '=' ) remove 2 bits
            // to delete appended bits during encoding.
            else
            {
                num = num >> 2;
                count_bits -= 2;
            }
        }

        while (count_bits != 0)
        {
            count_bits -= 8;

            // 255 in binary is 11111111
            decoded_string[k++] = (num >> count_bits) & 255;
        }
    }

    // place NULL character to mark end of string.
    decoded_string[k] = '\0';

    return decoded_string;
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

static void create_keys()
{
    LOG(LL_INFO, ("generate keys"));

    uint8_t priv[PRIVATE_KEY_LEN];
    uint8_t pub[PUBLIC_KEY_LEN];
    //char privKeyHex[2 * PRIVATE_KEY_LEN * sizeof(char)];
    //char pubKeyHex[2 * PUBLIC_KEY_LEN * sizeof(char)];

    uECC_set_rng(RNG);
    const struct uECC_Curve_t *curve = uECC_secp160r1();
    uECC_make_key(pub, priv, curve);

    //hex_encode(pub, sizeof(pub), pubKeyHex);
    //hex_encode(priv, sizeof(priv), privKeyHex);
    char *privKeyBase64 = base64Encoder(priv, sizeof(priv));
    char *pubKeyBase64 = base64Encoder(pub, sizeof(pub));

    LOG(LL_INFO, ("private key %s", privKeyBase64));
    LOG(LL_INFO, ("public key %s", pubKeyBase64));

    mgos_sys_config_set_auth_private_key(privKeyBase64);
    mgos_sys_config_set_auth_public_key(pubKeyBase64);
    char *err = NULL;
    save_cfg(&mgos_sys_config, &err); /* Writes conf9.json */
    LOG(LL_INFO, ("Saving configuration: %s\n", err ? err : "no error"));
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

    char buf[200];
    sensor_data_to_json(buf, sizeof(buf));
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
        snprintf(topic, sizeof(topic), mgos_sys_config_get_mqtt_topic(), mgos_sys_config_get_mqtt_client_id());

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

    // create keys
    if (mgos_sys_config_get_auth_public_key() == NULL)
        create_keys();

    // init threads
    PT_INIT(&pt_dht);
    PT_INIT(&pt_mhz19c_measure);
    PT_INIT(&pt_button);
    PT_INIT(&pt_http_push);
    PT_INIT(&pt_mqtt_push);

    // http handler
    mgos_register_http_endpoint("/api/sensor/values", http_handler_sensor_data, NULL);
    mgos_register_http_endpoint("/api/auth/keys/create", http_handler_auth_keys, NULL); // TODO: restrict this path to authenticated user via acl (or using digest authentication)

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