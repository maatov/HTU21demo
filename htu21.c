/*

*/
#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "hardware/i2c.h"
#define _DEBUG_
#include "tracer.h"

// I2C reserves some addresses for special purposes. We exclude these from the scan.
// These are any addresses of the form 000 0xxx or 111 1xxx

bool reserved_addr(uint8_t addr)
{
    return (addr & 0x78) == 0 || (addr & 0x78) == 0x78;
}

void led(bool on)
{
    static bool _initialized = false;
    if (!_initialized)
    {
        gpio_init(PICO_DEFAULT_LED_PIN);
        gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
        _initialized = true;
    }
    gpio_put(PICO_DEFAULT_LED_PIN, (on ? 1 : 0));
}

int scan(i2c_inst_t *i2c)
{
    led(true);
#if !defined(i2c_default) || !defined(PICO_DEFAULT_I2C_SDA_PIN) || !defined(PICO_DEFAULT_I2C_SCL_PIN)
#warning i2c/bus_scan example requires a board with I2C pins
    puts("Default I2C pins were not defined");
#else
    uint rv;

    sleep_ms(50);

    // Make the I2C pins available to picotool
    //bi_decl(bi_2pins_with_func(PICO_DEFAULT_I2C_SDA_PIN, PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C));

    printf("\nI2C Bus Scan\n");
    printf("   0  1  2  3  4  5  6  7  8  9  A  B  C  D  E  F\n");

    for (int addr = 0; addr < (1 << 7); ++addr)
    {
        if (addr % 16 == 0)
        {
            printf("%02x ", addr);
        }

        // Perform a 1-byte dummy read from the probe address. If a slave
        // acknowledges this address, the function returns the number of bytes
        // transferred. If the address byte is ignored, the function returns
        // -1.

        // Skip over any reserved addresses.
        int ret;
        uint8_t rxdata;
        if (reserved_addr(addr))
        {
            ret = PICO_ERROR_GENERIC;
        }
        else
        {
            ret = i2c_read_blocking(i2c0, addr, &rxdata, 1, false);
        }

        printf(ret < 0 ? "." : "@");
        printf(addr % 16 == 15 ? "\n" : "  ");
        sleep_ms(33);
    }
    printf("Done.\n");
    led(false);
    return 0;
#endif
}

int i2c_exchange(i2c_inst_t *i2c, uint8_t address, const uint8_t *out, size_t outLen, uint8_t *in, size_t inMaxLen, size_t *Length)
{
    int rv;
    rv = i2c_write_blocking(i2c_default, address, out, outLen, true); // true to keep master control of bus
    TRACEint("i2c_write_blocking() ", rv);
    if (rv < 0) {
        //error -> end
        return -1;
    }
    rv = i2c_read_blocking(i2c_default, address, in, inMaxLen, false);
    TRACEint("i2c_read_blocking() ", rv);
    if (rv < 0) {
        //error -> end
        return -1;
    } else {
        *Length = rv;
    }
    return *Length;
}

int get_temp(i2c_inst_t *i2c)
{
    TRACE(__FUNCTION__);
    int rv;
    uint8_t addr = 0x40;
    uint8_t val = 0xE3; //read temp hold master
    uint8_t data[8] = {};
    rv = i2c_write_blocking(i2c_default, addr, &val, 1, true); // true to keep master control of bus
    TRACEint("i2c_write_blocking() ", rv);
    if (rv < 0)
    {
        //error -> end
        return -1;
    }
    rv = i2c_read_blocking(i2c_default, addr, data, 3, false);
    TRACEint("i2c_read_blocking() ", rv);
    if (rv < 0)
    {
        //error -> end
        return -1;
    }
    printf("sensor data %02X,%02X,%02X\n", data[0], data[1], data[2]);
    if (rv >= 3)
    {
        //compute temperature
        uint16_t sens_temp = ((data[0] << 8) | ((data[1] >> 2) << 2));
        double TempC = (-46.85 + 175.72 * sens_temp / (1 << 16));
        printf("Temperature: %.2f C\n", TempC);
    }

    return 0;
}

int get_relhum(i2c_inst_t *i2c)
{
    TRACE(__FUNCTION__);
    size_t len = 0;
    uint8_t data[8] = {};
    uint8_t cmd = 0xE5;
    int rv = i2c_exchange(i2c,0x40,&cmd,1,data,3,&len);
    TRACEint("i2c_exchange() ", rv);
    if(rv>0 && len>=3) {
        //humidity computing
        printf("sensor data %02X,%02X,%02X\n", data[0],data[1],data[2]);
        //3rd byte is checksum, last 2bits of 2nd byte is status
        uint32_t sen_val = ((data[0]<<8) | (data[1]>>2)<<2);
        double rh = (-6.0 + 125.0 * sen_val / (1<<16));
        printf("rel.humidity: %.2f %%\n", rh);
    } else if (rv==-1) {
        return -1;
    }
    return 0;
}

int reset_htu(i2c_inst_t *i2c)
{
    TRACE(__FUNCTION__);
    //address 0x40
    //size_t w_aval = i2c_get_write_available (i2c);
    //TRACEint("i2c_get_write_available() ",w_aval);
    absolute_time_t atime;

    uint8_t data[8] = {0xFE};
    int rv = i2c_write_blocking_until(i2c, 0x40, data, 1, true, make_timeout_time_ms(3000));
    TRACEint("i2c_write_blocking()", rv);

    return 0;
}

/*
* decorator
*/
typedef int (*htu_method_t)(i2c_inst_t *i2c);
int runme(htu_method_t fnc, i2c_inst_t *param)
{
    uint64_t s = to_us_since_boot(get_absolute_time());
    led(true);
    int rv = fnc(param);
    led(false);
    uint64_t tottime = to_us_since_boot(get_absolute_time()) - s;
    printf("time consumpt: %llu\n", tottime);
    return rv;
}

/**
 * NEFUNGUJE!!  
 * \brief Callback for a repeating timer
 * \ingroup repeating_timer
 * \param rt repeating time structure containing information about the repeating time. user_data is of primary important to the user
 * \return true to continue repeating, false to stop.
 */
typedef struct {
    i2c_inst_t* i2cpoi;
}  EveryCallArgs_t;

void htu_read_periodically() {
    TRACE(__FUNCTION__);
    i2c_inst_t* i2ch = i2c0;
    printf("i2c addr %p\n",i2ch);
    int rv;
    do {
        rv = runme(get_temp, i2ch);
        if(rv==-1) {
            runme(reset_htu, i2ch);
            sleep_ms(500);
            continue;
        }
        sleep_ms(50);
        rv = runme(get_relhum,i2ch);
        if(rv==-1) {
            runme(reset_htu, i2ch);
            sleep_ms(500);
            continue;
        }
        break;
    } while(true);
}

bool testing_timer(repeating_timer_t* rt) {
    printf("testing timer %ul, addr from user_data : %p \n", rt->delay_us, rt->user_data);
    htu_read_periodically(rt);
    return true;
}

static repeating_timer_t timer = {};
static EveryCallArgs_t args;

int main()
{
    // Enable UART so we can print status output
    stdio_init_all();

    sleep_ms(2 * 1000); //wait for /dev/ttyACM0  terminal to be opened

    //init i2c
    int rv;
    const int sdaPin = 12;
    const int sclPin = 13;

    // This example will use I2C0 on the default SDA and SCL pins (4, 5 on a Pico)
    gpio_set_function(sdaPin, GPIO_FUNC_I2C);
    gpio_set_function(sclPin, GPIO_FUNC_I2C);
    gpio_pull_up(sdaPin);
    gpio_pull_up(sclPin);

    rv = i2c_init(i2c0, 100 * 1000);
    printf("i2c_init() %d\n", rv);

    scan(i2c0);

    {
        runme(reset_htu, i2c0);
        sleep_ms(50);
        runme(get_temp, i2c0);
        sleep_ms(50);
        runme(get_relhum,i2c0);
    }
    {
        printf("ts: %llu\n", to_us_since_boot(get_absolute_time()));
        sleep_ms(333);
        runme(reset_htu, i2c0);
        sleep_ms(50);
        runme(get_temp, i2c0);
        sleep_ms(50);
        runme(get_relhum,i2c0);
        //printf("ts: %"PRIu64"\n", to_us_since_boot(get_absolute_time()));
        printf("ts: %llu\n", to_us_since_boot(get_absolute_time()));
    }

    while(true) {
        sleep_ms(60*1000);
        htu_read_periodically();
    }

    return 0;
}
