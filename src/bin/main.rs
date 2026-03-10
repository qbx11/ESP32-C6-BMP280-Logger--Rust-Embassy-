#![no_std]
#![no_main]

use core::fmt::Write as FmtWrite;

use embassy_executor::Spawner;
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, channel::Channel};
use embassy_time::{Duration, Timer};
use embedded_io_async::Write as AsyncWrite;
use esp_backtrace as _;
use esp_bootloader_esp_idf::esp_app_desc;
use esp_hal::{
    i2c::master::{Config as I2cConfig, I2c},
    timer::timg::TimerGroup,
    uart::{Config as UartConfig, Uart},
};
use heapless::String;

esp_app_desc!();

#[derive(Clone, Copy)]
struct SensorReading {
    temperature_cdeg: i32, 
    pressure_pa: u32,
    counter: u32,
}

// Queue size 4 is plenty for 1Hz sampling rate
static SENSOR_CH: Channel<CriticalSectionRawMutex, SensorReading, 4> = Channel::new();

struct Bmp280Calib {
    t1: u16, t2: i16, t3: i16,
    p1: u16, p2: i16, p3: i16, p4: i16,
    p5: i16, p6: i16, p7: i16, p8: i16, p9: i16,
}

// 0x76 if SDO tied to GND, 0x77 if tied to VCC
const BMP280_ADDR: u8 = 0x76;

fn bmp280_init(i2c: &mut I2c<'static, esp_hal::Blocking>) {
    // 0xF4: Normal mode, temp osrs x2, press osrs x16
    i2c.write(BMP280_ADDR, &[0xF4, 0x57]).unwrap();
    // 0xF5: standby 250ms, filter x4
    i2c.write(BMP280_ADDR, &[0xF5, 0x2C]).unwrap();
}

fn bmp280_read_calib(i2c: &mut I2c<'static, esp_hal::Blocking>) -> Bmp280Calib {
    let mut buf = [0u8; 24];
    // Burst read all 24 bytes of factory calib data
    i2c.write_read(BMP280_ADDR, &[0x88], &mut buf).unwrap();

    // Bosch uses little-endian for these
    Bmp280Calib {
        t1: u16::from_le_bytes([buf[0], buf[1]]),
        t2: i16::from_le_bytes([buf[2], buf[3]]),
        t3: i16::from_le_bytes([buf[4], buf[5]]),
        p1: u16::from_le_bytes([buf[6], buf[7]]),
        p2: i16::from_le_bytes([buf[8], buf[9]]),
        p3: i16::from_le_bytes([buf[10], buf[11]]),
        p4: i16::from_le_bytes([buf[12], buf[13]]),
        p5: i16::from_le_bytes([buf[14], buf[15]]),
        p6: i16::from_le_bytes([buf[16], buf[17]]),
        p7: i16::from_le_bytes([buf[18], buf[19]]),
        p8: i16::from_le_bytes([buf[20], buf[21]]),
        p9: i16::from_le_bytes([buf[22], buf[23]]),
    }
}

fn bmp280_read(i2c: &mut I2c<'static, esp_hal::Blocking>, cal: &Bmp280Calib) -> (i32, u32) {
    let mut buf = [0u8; 6];
    i2c.write_read(BMP280_ADDR, &[0xF7], &mut buf).unwrap();

    // 20-bit raw ADC values
    let adc_p = ((buf[0] as i32) << 12) | ((buf[1] as i32) << 4) | ((buf[2] as i32) >> 4);
    let adc_t = ((buf[3] as i32) << 12) | ((buf[4] as i32) << 4) | ((buf[5] as i32) >> 4);

   
    // Formula straight from datasheet
    let var1 = ((adc_t >> 3) - ((cal.t1 as i32) << 1)) * (cal.t2 as i32) >> 11;
    let tmp  = (adc_t >> 4) - (cal.t1 as i32);
    let var2 = (tmp * tmp >> 12) * (cal.t3 as i32) >> 14;
    let t_fine = var1 + var2;
    let temp_cdeg = (t_fine * 5 + 128) >> 8;

    let mut v1 = (t_fine as i64) - 128000;
    let mut v2 = v1 * v1 * (cal.p6 as i64);
    v2 += (v1 * (cal.p5 as i64)) << 17;
    v2 += (cal.p4 as i64) << 35;
    v1  = ((v1 * v1 * (cal.p3 as i64)) >> 8) + ((v1 * (cal.p2 as i64)) << 12);
    v1  = (((1i64 << 47) + v1) * (cal.p1 as i64)) >> 33;

    let pressure_pa = if v1 == 0 {
        0 // avoid division by zero
    } else {
        let mut p = 1048576i64 - adc_p as i64;
        p = ((p << 31) - v2) * 3125 / v1;
        v1 = ((cal.p9 as i64) * (p >> 13) * (p >> 13)) >> 25;
        v2 = ((cal.p8 as i64) * p) >> 19;
        p  = ((p + v1 + v2) >> 8) + ((cal.p7 as i64) << 4);
        (p >> 8) as u32
    };

    (temp_cdeg, pressure_pa)
}

#[embassy_executor::task]
async fn sensor_task(mut i2c: I2c<'static, esp_hal::Blocking>) {
    // TODO:  handle I2C errors properly
    bmp280_init(&mut i2c);
    let calib = bmp280_read_calib(&mut i2c);
    
    let mut counter = 0;

    loop {
        Timer::after(Duration::from_secs(1)).await;

        let (temp_cdeg, pressure_pa) = bmp280_read(&mut i2c, &calib);

        // Debug print
        log::info!(
            "T={}.{:02}C, P={}Pa, cnt={}",
            temp_cdeg / 100, temp_cdeg.abs() % 100,
            pressure_pa, counter
        );

        SENSOR_CH.send(SensorReading { temperature_cdeg: temp_cdeg, pressure_pa, counter }).await;
        counter = counter.wrapping_add(1);
    }
}

#[embassy_executor::task]
async fn uart_task(mut uart: Uart<'static, esp_hal::Async>) {
    loop {
        let r = SENSOR_CH.receive().await;

        // CSV alike format
        let mut buf: String<64> = String::new();
        let _ = write!(
            buf,
            "T:{}.{:02},P:{},CNT:{}\r\n",
            r.temperature_cdeg / 100,
            r.temperature_cdeg.abs() % 100,
            r.pressure_pa,
            r.counter,
        );

        if let Err(e) = uart.write_all(buf.as_bytes()).await {
            log::warn!("UART write err: {:?}", e); // Happens when the buffer overruns
        }
    }
}

#[esp_rtos::main]
async fn main(spawner: Spawner) {
    esp_println::logger::init_logger_from_env();
    let peripherals = esp_hal::init(esp_hal::Config::default());

    let timg0 = TimerGroup::new(peripherals.TIMG0);
    let sw_ints = esp_hal::interrupt::software::SoftwareInterruptControl::new(
        peripherals.SW_INTERRUPT,
    );
    esp_rtos::start(timg0.timer0, sw_ints.software_interrupt0);

    // I2C0: SDA=19, SCL=20, 400kHz
    let i2c = I2c::new(peripherals.I2C0, I2cConfig::default())
        .expect("I2C init failed")
        .with_sda(peripherals.GPIO19)
        .with_scl(peripherals.GPIO20);

    let uart0 = Uart::new(peripherals.UART0, UartConfig::default())
        .expect("UART init failed")
        .with_tx(peripherals.GPIO16)
        .with_rx(peripherals.GPIO17)
        .into_async();

    spawner.spawn(sensor_task(i2c)).unwrap();
    spawner.spawn(uart_task(uart0)).unwrap();

    log::info!("System started");
}