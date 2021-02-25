#![no_main]
#![no_std]

use arduino_nano33iot::prelude::*;
use usb_device::prelude::*;

use arduino_nano33iot::clock::GenericClockController;
use arduino_nano33iot::usb::UsbBus;
use cortex_m::asm;
use usb_device::bus::UsbBusAllocator;
use usbd_serial::{SerialPort, USB_CLASS_CDC};

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
        let mut pins = arduino_nano33iot::Pins::new(cx.device.PORT);
        let mut led = pins.led_sck.into_open_drain_output(&mut pins.port);

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
                .serial_number("OCC01")
                .device_class(USB_CLASS_CDC)
                .build();
        led.set_high().unwrap();

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

        if !device.poll(&mut [serial]) {
            return;
        }

        let mut buf = [0u8; 16];
        let _ = serial.read(&mut buf);
    }

    #[idle]
    fn idle(_: idle::Context) -> ! {
        loop {
            asm::nop();
        }
    }
};
