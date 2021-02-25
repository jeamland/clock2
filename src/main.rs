#![no_main]
#![no_std]

use arduino_nano33iot::prelude::*;
use usb_device::prelude::*;

use arduino_nano33iot::clock::GenericClockController;
use arduino_nano33iot::delay::Delay;
use arduino_nano33iot::usb::UsbBus;
use usb_device::bus::UsbBusAllocator;
use usbd_serial::{SerialPort, USB_CLASS_CDC};

const BOOT_DELAY_MS: u16 = 100;

#[rtic::app(device = arduino_nano33iot::pac, peripherals = true)]
const APP: () = {
    struct Resources {
        clocks: GenericClockController,
        usb_device: UsbDevice<'static, UsbBus>,
        usb_serial: SerialPort<'static, UsbBus>,
    }

    #[init]
    fn init(mut cx: init::Context) -> init::LateResources {
        static mut USB_BUS: Option<UsbBusAllocator<UsbBus>> = None;

        let mut clocks = GenericClockController::with_internal_32kosc(
            cx.device.GCLK,
            &mut cx.device.PM,
            &mut cx.device.SYSCTRL,
            &mut cx.device.NVMCTRL,
        );
        let pins = arduino_nano33iot::Pins::new(cx.device.PORT);

        let mut delay = Delay::new(cx.core.SYST, &mut clocks);

        delay.delay_ms(BOOT_DELAY_MS);

        *USB_BUS = Some(arduino_nano33iot::usb_allocator(
            cx.device.USB,
            &mut clocks,
            &mut cx.device.PM,
            pins.usb_dm,
            pins.usb_dp,
        ));
        let usb_bus = USB_BUS.as_ref().unwrap();

        let usb_device = UsbDeviceBuilder::new(usb_bus, UsbVidPid(0x16c0, 0x27dd))
            .manufacturer("Oddly Named Devices")
            .product("Overcomplicated Clock")
            .serial_number("OCC01")
            .device_class(USB_CLASS_CDC)
            .build();
        let usb_serial = SerialPort::new(usb_bus);

        init::LateResources {
            clocks,
            usb_device,
            usb_serial,
        }
    }

    #[task(binds = USB, resources=[usb_device, usb_serial])]
    fn usb(cx: usb::Context) {
        let device = cx.resources.usb_device;
        let serial = cx.resources.usb_serial;

        device.poll(&mut [serial]);

        let mut buf = [0u8; 16];
        let _ = serial.read(&mut buf);
    }
};
