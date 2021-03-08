#![no_main]
#![no_std]
#![feature(default_alloc_error_handler)]

extern crate alloc;

use alloc::collections::vec_deque::VecDeque;
use alloc::format;
use alloc::string::{String, ToString};
use core::time::Duration;

use usb_device::prelude::*;

use alloc_cortex_m::CortexMHeap;
use arduino_nano33iot::clock::GenericClockController;
use arduino_nano33iot::delay::Delay;
use arduino_nano33iot::gpio::{self, v2::*, Floating, Input, Port};
use arduino_nano33iot::pac;
use arduino_nano33iot::sercom::{self, Pad, Pad0, Pad1, Pad3, PadPin, SPIMaster2};
use arduino_nano33iot::time::{Hertz, MegaHertz};
use arduino_nano33iot::usb::UsbBus;
use atsamd_hal::hal::spi;
use cortex_m::asm;
use cortex_m::peripheral::SCB;
use embedded_hal::digital::v2::OutputPin;
use no_std_net::Ipv4Addr;
use usb_device::bus::UsbBusAllocator;
use usbd_serial::{SerialPort, USB_CLASS_CDC};
use wifi_nina::{
    transport::SpiTransport,
    types::{Config, ConnectionState},
    Wifi,
};

const BOOT_LOADER_MAGIC: u32 = 0x007738135;
const BOOT_LOADER_MAGIC_ADDRESS: usize = 0x20007FFC;

const WIFI_SSID: &[u8] = b"TelstraF5474D";
const WIFI_PASSPHRASE: &[u8] = b"C9AF1CC3A2";

const NTP_SERVER: &str = "au.pool.ntp.org";
const LOCAL_PORT: u16 = 2468;

#[global_allocator]
static ALLOCATOR: CortexMHeap = CortexMHeap::empty();

type NinaTransport = SpiTransport<
    SPIMaster2<
        Pad<pac::SERCOM2, Pad1, gpio::Pin<PA13, Alternate<C>>>,
        Pad<pac::SERCOM2, Pad0, gpio::Pin<PA12, Alternate<C>>>,
        Pad<pac::SERCOM2, Pad3, gpio::Pin<PA15, Alternate<C>>>,
    >,
    gpio::Pin<PA28, Input<Floating>>,
    gpio::Pin<PA08, Output<PushPull>>,
    gpio::Pin<PA14, Output<PushPull>>,
    Delay,
>;
type Nina = Wifi<NinaTransport>;

fn reset_to_bootloader() -> ! {
    let boot_magic = BOOT_LOADER_MAGIC_ADDRESS as *mut u32;
    unsafe {
        *boot_magic = BOOT_LOADER_MAGIC;
    }
    SCB::sys_reset();
}

pub fn nina_spi_master<F: Into<Hertz>>(
    clocks: &mut GenericClockController,
    bus_speed: F,
    sercom1: pac::SERCOM2,
    pm: &mut pac::PM,
    sck: gpio::Pa15<Input<Floating>>,
    mosi: gpio::Pa12<Input<Floating>>,
    miso: gpio::Pa13<Input<Floating>>,
    port: &mut Port,
) -> sercom::SPIMaster2<
    sercom::Sercom2Pad1<gpio::Pa13<gpio::PfC>>,
    sercom::Sercom2Pad0<gpio::Pa12<gpio::PfC>>,
    sercom::Sercom2Pad3<gpio::Pa15<gpio::PfC>>,
> {
    let gclk0 = clocks.gclk0();

    sercom::SPIMaster2::new(
        &clocks.sercom2_core(&gclk0).unwrap(),
        bus_speed.into(),
        spi::Mode {
            phase: spi::Phase::CaptureOnFirstTransition,
            polarity: spi::Polarity::IdleLow,
        },
        sercom1,
        pm,
        (miso.into_pad(port), mosi.into_pad(port), sck.into_pad(port)),
    )
}

macro_rules! clog {
    ($cx:ident, $($arg:tt)*) => {{
        let res = format!($($arg)*);
        $cx.resources.log_buffer.push_back(res);
    }}
}

#[rtic::app(device = arduino_nano33iot::pac, peripherals = true)]
const APP: () = {
    struct Resources {
        clocks: GenericClockController,
        usb_device: UsbDevice<'static, UsbBus>,
        usb_serial: SerialPort<'static, UsbBus>,
        led: arduino_nano33iot::gpio::Pin<
            PA17,
            arduino_nano33iot::gpio::v2::Output<arduino_nano33iot::gpio::v2::PushPull>,
        >,

        nina: Nina,

        log_buffer: VecDeque<String>,
    }

    #[init(spawn=[wifi])]
    fn init(mut cx: init::Context) -> init::LateResources {
        static mut USB_BUS: Option<UsbBusAllocator<UsbBus>> = None;

        let start = cortex_m_rt::heap_start() as usize;
        let size = 4096; // in bytes
        unsafe { ALLOCATOR.init(start, size) }

        let mut clocks = GenericClockController::with_internal_32kosc(
            cx.device.GCLK,
            &mut cx.device.PM,
            &mut cx.device.SYSCTRL,
            &mut cx.device.NVMCTRL,
        );
        let mut pins = arduino_nano33iot::Pins::new(cx.device.PORT);
        let led = pins.led_sck.into_open_drain_output(&mut pins.port);

        *USB_BUS = Some(arduino_nano33iot::usb_allocator(
            cx.device.USB,
            &mut clocks,
            &mut cx.device.PM,
            pins.usb_dm,
            pins.usb_dp,
        ));

        let usb_serial = SerialPort::new(USB_BUS.as_ref().unwrap());
        let usb_device =
            UsbDeviceBuilder::new(USB_BUS.as_ref().unwrap(), UsbVidPid(0x16c0, 0x27dd))
                .manufacturer("Oddly Named Devices")
                .product("Overcomplicated Clock")
                .device_class(USB_CLASS_CDC)
                .build();

        let delay = Delay::new(cx.core.SYST, &mut clocks);
        let spi = nina_spi_master(
            &mut clocks,
            MegaHertz(8),
            cx.device.SERCOM2,
            &mut cx.device.PM,
            pins.nina_sck,
            pins.nina_mosi,
            pins.nina_miso,
            &mut pins.port,
        );
        let busy = pins.nina_ack.into_floating_input(&mut pins.port);
        let reset = pins.nina_resetn.into_open_drain_output(&mut pins.port);
        let cs = pins.nina_cs.into_open_drain_output(&mut pins.port);
        // let mut gpio0 = pins.nina_gpio0.into_open_drain_output(&mut pins.port);
        // gpio0.set_high().unwrap();
        let transport = SpiTransport::start(spi, busy, reset, cs, delay).unwrap();
        let nina = Wifi::new(transport);
        // gpio0.set_low().unwrap();

        let mut log_buffer = VecDeque::new();
        log_buffer.push_back("Hello from thing".to_string());

        cx.spawn.wifi().unwrap();

        init::LateResources {
            clocks,
            usb_device,
            usb_serial,
            led,
            nina,
            log_buffer,
        }
    }

    #[task(binds=USB, resources=[usb_device, usb_serial, led, log_buffer])]
    fn usb(cx: usb::Context) {
        let device = cx.resources.usb_device;
        let serial = cx.resources.usb_serial;

        if !device.poll(&mut [serial]) {
            return;
        }

        let mut buf = [0u8; 16];
        let _ = serial.read(&mut buf);

        if serial.dtr()
            && serial.line_coding().data_rate() == 1200
            && buf.iter().any(|&b| b == 0x2e)
        {
            reset_to_bootloader();
        }

        if let Some(line) = cx.resources.log_buffer.pop_front() {
            serial.write(line.as_bytes()).unwrap();
            serial.write(b"\r\n").unwrap();
        }
    }

    #[task(resources=[led, nina, log_buffer])]
    fn wifi(cx: wifi::Context) {
        let led = cx.resources.led;
        led.set_high().unwrap();

        let nina = cx.resources.nina;
        match nina.get_firmware_version() {
            Ok(v) => clog!(cx, "{}", String::from_utf8(v.to_vec()).unwrap()),
            Err(e) => clog!(cx, "{:?}", e),
        };

        match nina.configure(Config::wpa_psk(WIFI_SSID, WIFI_PASSPHRASE), None) {
            Ok(_) => (),
            Err(e) => {
                clog!(cx, "failed join: {:?}", e);
                return;
            }
        };

        loop {
            if let Ok(_) =
                nina.await_connection_state(ConnectionState::Connected, Duration::from_secs(1))
            {
                clog!(cx, "Connected!");
                break;
            }
            clog!(cx, "Connecting...");
        }

        // let ip = match nina.resolve(NTP_SERVER) {
        //     Ok(a) => a,
        //     Err(e) => {
        //         clog!(cx, "Failed lookup: {:?}", e);
        //         return;
        //     }
        // };
        let addr: [u8; 4] = [10, 0, 0, 45];
        let ip = Ipv4Addr::from(addr);

        let mut client = match nina.new_udp(LOCAL_PORT) {
            Ok(c) => c,
            Err(e) => {
                clog!(cx, "Client failed: {:?}", e);
                return;
            }
        };

        clog!(cx, "client created");

        clog!(cx, "{:?}", client.send(nina, ip, 2222, &[1, 2, 3, 4]));
    }

    #[idle]
    fn idle(_: idle::Context) -> ! {
        loop {
            asm::nop();
        }
    }

    extern "C" {
        fn ADC();
    }
};
