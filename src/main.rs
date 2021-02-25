#![no_main]
#![no_std]

use usb_device::prelude::*;

use arduino_nano33iot::clock::GenericClockController;
use arduino_nano33iot::usb::UsbBus;
use usb_device::bus::UsbBusAllocator;
use usbd_serial::{SerialPort, USB_CLASS_CDC};

static mut USB_BUS_ALLOCATOR: Option<UsbBusAllocator<UsbBus>> = None;

#[rtic::app(device = arduino_nano33iot::pac, peripherals = true)]
const APP: () = {
    struct Resources {
        clocks: GenericClockController,
        usb_device: UsbDevice<'static, UsbBus>,
        usb_serial: SerialPort<'static, UsbBus>,
    }

    #[init]
    fn init(mut cx: init::Context) -> init::LateResources {
        let mut clocks = GenericClockController::with_internal_32kosc(
            cx.device.GCLK,
            &mut cx.device.PM,
            &mut cx.device.SYSCTRL,
            &mut cx.device.NVMCTRL,
        );
        let pins = arduino_nano33iot::Pins::new(cx.device.PORT);

        let usb_allocator = unsafe {
            USB_BUS_ALLOCATOR = Some(arduino_nano33iot::usb_allocator(
                cx.device.USB,
                &mut clocks,
                &mut cx.device.PM,
                pins.usb_dm,
                pins.usb_dp,
            ));
            USB_BUS_ALLOCATOR.as_ref().unwrap()
        };
        let usb_device = UsbDeviceBuilder::new(usb_allocator, UsbVidPid(0x2317, 0x1723))
            .manufacturer("Oddly Named Devices")
            .product("Overcomplicated Clock")
            .serial_number("OCC01")
            .device_class(USB_CLASS_CDC)
            .build();
        let usb_serial = SerialPort::new(usb_allocator);

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
