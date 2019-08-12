#![no_main]
#![no_std]
#![feature(alloc_error_handler)]
#![feature(clamp)]
#![feature(const_fn)]

//extern crate panic_halt;




#[macro_use(block)]
extern crate nb;
//use no_std_compat::prelude::v1::*;

extern crate alloc;
extern crate alloc_cortex_m;
extern crate cortex_m_rt as rt; // v0.5.x
extern crate keytokey;

use core::alloc::Layout;

#[alloc_error_handler]
fn oom(_info: Layout, //~ ERROR argument should be `Layout`
) -> ! //~ ERROR return type should be `!`
{
    loop {}
}

use alloc_cortex_m::CortexMHeap;

#[global_allocator]
static ALLOCATOR: crate::trallocator::Trallocator<CortexMHeap> = crate::trallocator::Trallocator::new(CortexMHeap::empty());

use core::panic::PanicInfo;
use core::sync::atomic::{self, Ordering};

#[inline(never)]
#[panic_handler]
fn panic(_info: &PanicInfo) -> ! {
    loop {
        atomic::compiler_fence(Ordering::SeqCst);
    }
}

pub mod hid;
pub mod keyboard;
pub mod matrix;
mod usbout;
mod trallocator;
use usbout::USBOut;

use crate::keyboard::Keyboard;
use crate::matrix::Matrix;
use no_std_compat::prelude::v1::*;
use rtfm::app;

//use stm32f1xx_hal::prelude::*; can't use this with v2 digital traits
use stm32_usbd::{UsbBus, UsbBusType};
//use stm32f1xx_hal::gpio::{Alternate, Floating, Input, PushPull};
//use stm32f1xx_hal::prelude::*;
pub use stm32f1xx_hal::afio::AfioExt as _stm32_hal_afio_AfioExt;
pub use stm32f1xx_hal::dma::DmaExt as _stm32_hal_dma_DmaExt;
pub use stm32f1xx_hal::flash::FlashExt as _stm32_hal_flash_FlashExt;
pub use stm32f1xx_hal::gpio::GpioExt as _stm32_hal_gpio_GpioExt;
//pub use stm32f1xx_hal::hal::adc::OneShot as _embedded_hal_adc_OneShot;
//pub use stm32f1xx_hal::hal::digital::StatefulOutputPin as _embedded_hal_digital_StatefulOutputPin;
//pub use stm32f1xx_hal::hal::digital::ToggleableOutputPin as _embedded_hal_digital_ToggleableOutputPin;
//pub use stm32f1xx_hal::hal::prelude::*;
use core::convert::TryInto;
use debouncing::{DebounceResult, Debouncer};
pub use stm32f1xx_hal::dma::CircReadDma as _stm32_hal_dma_CircReadDma;
pub use stm32f1xx_hal::dma::ReadDma as _stm32_hal_dma_ReadDma;
pub use stm32f1xx_hal::dma::WriteDma as _stm32_hal_dma_WriteDma;
pub use stm32f1xx_hal::pwm::PwmExt as _stm32_hal_pwm_PwmExt;
pub use stm32f1xx_hal::rcc::RccExt as _stm32_hal_rcc_RccExt;
pub use stm32f1xx_hal::time::U32Ext as _stm32_hal_time_U32Ext;

#[allow(deprecated)]
use embedded_hal::digital::v1::ToggleableOutputPin;
use embedded_hal::digital::v2::OutputPin;
#[allow(unused_imports)]
use embedded_hal::digital::v2_compat;
use embedded_hal::serial::Write;

use keytokey::Keyboard as K2KKeyboard;
use keytokey::USBKeyOut;
use stm32f1;
use stm32f1xx_hal::stm32;
use stm32f1xx_hal::{gpio, serial, timer};
use usb_device::bus;
use usb_device::class::UsbClass;
use usb_device::prelude::*;

type KeyboardHidClass = hid::HidClass<'static, UsbBusType, Keyboard>;
type Led = gpio::gpioc::PC13<gpio::Output<gpio::PushPull>>;

// Generic keyboard from
// https://github.com/obdev/v-usb/blob/master/usbdrv/USB-IDs-for-free.txt
const VID: u16 = 0x27db;
const PID: u16 = 0x16c0;

pub trait StringSender {
    fn writeln(&mut self, s: &str);
}

impl StringSender for serial::Tx<stm32f1::stm32f103::USART1> {
    fn writeln(&mut self, s: &str) {
        for b in s.bytes() {
            block!(self.write(b)).ok();
        }
        block!(self.write(b'\r')).ok();
        block!(self.write(b'\n')).ok();
    }
}

pub fn get_translation() -> Vec<u32> {
    use keytokey::KeyCode::*;
    vec![
        0, //one of the missing nes?
        //00000001
        V.into(),
        //00000002
        SColon.into(),
        //00000003
        F8.into(),
        //00000004
        K.into(),
        //00000005
        Space.into(),
        //00000006
        BSpace.into(),
        //00000007
        Q.into(),
        //00000008
        F2.into(),
        //9
        9,
        //0000000a
        C.into(),
        //0000000b
        L.into(),
        //c
        0xc,
        //0000000d
        J.into(),
        //0000000e
        LGui.into(),
        //0000000f
        Delete.into(), // which one is this?
        //00000010
        A.into(),
        //00000011
        F1.into(),
        //12
        0x12,
        //13
        D.into(),
        //14
        BSlash.into(), //labled LBracket.into(),
        //15
        Equal.into(),
        //16
        Down.into(),
        //17
        Home.into(),
        //18
        PgUp.into(),
        //19
        0xF0100u32.into(), // te lowe left backslash key
        //1A
        Escape.into(),
        //1B
        0x1b,
        //0000001c
        E.into(),
        //0000001d
        Slash.into(), //label: slash
        //0000001e
        LBracket.into(), //top right, label bslash
        //0000001f
        Comma.into(),
        //0x20
        Enter.into(),
        //00000021
        LAlt.into(),
        //00000022
        Z.into(),
        //00000023
        Minus.into(),
        //24
        0x24,
        //00000025
        Right.into(),
        //00000026
        Dot.into(),
        //00000027
        Quote.into(),
        //00000028
        M.into(),
        //00000029
        End.into(),
        //0000002a
        LCtrl.into(),
        //0000002b
        X.into(),
        //2c
        Tab.into(),
        //2d
        0x2d,
        //0000002e
        Left.into(),
        //0000002f
        RBracket.into(),
        //00000030
        RShift.into(),
        //00000031
        Up.into(),
        //00000032
        RCtrl.into(),
        //00000033
        PgDown.into(),
        //00000034
        Grave.into(),
        //00000035
        LShift.into(),
        //36
        Copy.into(),//palm1
        //00000037
        Kb3.into(),
        //00000038
        Kb0.into(),
        //00000039
        F12.into(),
        //0000003a
        Kb8.into(),
        //0000003b
        Kb6.into(),
        //0000003c
        Kb5.into(),
        //0000003d
        Kb1.into(),
        //0000003e
        F6.into(),
        //3f
        Paste.into(), ///palm 2
        //00000040
        Kb4.into(),
        //00000041
        Kb9.into(),
        //00000042
        F11.into(),
        //00000043
        Kb7.into(),
        //00000044
        Y.into(),
        //00000045
        T.into(),
        //00000046
        Kb2.into(),
        //00000047
        F5.into(),
        //48,
        0, //palm3
        //00000049
        R.into(),
        //0000004a
        P.into(),
        //0000004b
        F10.into(),
        //0000004c
        I.into(),
        //0000004d
        H.into(),
        //0000004e
        G.into(),
        //0000004f
        W.into(),
        //00000050
        F3.into(),
        //51
        0x51,
        //00000052
        F.into(),
        //00000053
        O.into(),
        //00000054
        F9.into(),
        //00000055
        U.into(),
        //00000056
        N.into(),
        //00000057
        B.into(),
        //00000058
        S.into(),
        //00000059
        F4.into(),
    ]
}


pub fn get_keytokey<'a, T: USBKeyOut>(mut  output: T) -> K2KKeyboard<'a, T> {
use keytokey::{
    handlers, HandlerID, debug_handlers,
    Event, EventStatus, KeyCode, Keyboard, ProcessKeys,
    Modifier,
    USBKeyOut, 
    premade
};
    output.debug(&format!("A{}", ALLOCATOR.get()));
    let mut k = Keyboard::new(output);
    k.output.debug(&format!("B{}", ALLOCATOR.get()));
    use handlers::LayerAction::RewriteTo as RT;
    use handlers::LayerAction::SendString;
    use handlers::LayerAction::RewriteToShifted as RTS;
    k.add_handler(premade::space_cadet_handler(KeyCode::F, KeyCode::U, 
        k.future_handler_id(2)));
    let numpad_id = k.add_handler(Box::new(
        handlers::Layer::new(vec![
            (KeyCode::U, RT(KeyCode::Kb7.into())),
            (KeyCode::I, RT(KeyCode::Kb8.into())),
            (KeyCode::O, RT(KeyCode::Kb9.into())),
            (KeyCode::J, RT(KeyCode::Kb4.into())),
            (KeyCode::K, RT(KeyCode::Kb5.into())),
            (KeyCode::L, RT(KeyCode::Kb6.into())),
            (KeyCode::M, RT(KeyCode::Kb1.into())),
            (KeyCode::Comma, RT(KeyCode::Kb2.into())),
            (KeyCode::Dot, RT(KeyCode::Kb3.into())),
            (KeyCode::Up, RT(KeyCode::Kb0.into())),
            (KeyCode::Space, RT(KeyCode::Tab.into())),
            (KeyCode::Down, RT(KeyCode::Dot.into())),
            (KeyCode::BSlash, RT(KeyCode::Comma.into())),
        ])
    )
    );
    k.output.debug(&format!("C{}", ALLOCATOR.get()));

    k.add_handler(premade::space_cadet_handler(KeyCode::J, KeyCode::H, 
        k.future_handler_id(2)));
    let umlaut_id = k.add_handler(Box::new(
        handlers::Layer::new(vec![
            (KeyCode::A, RTS(0xE4, 0xC4)),
            (KeyCode::S, RTS(0xF6, 0xD6)),
            (KeyCode::R, RTS(0xFC, 0xDC)),
            (KeyCode::SColon, SendString("ß")),
        ])
    )
    );
  //  k.output.state().enable_handler(umlaut_id);
    struct EscapeAndOff{ 
        pub ids: Vec<HandlerID>
  };
    impl EscapeAndOff {
        fn new() -> EscapeAndOff {
            EscapeAndOff{ids: Vec::new()}
        }}

    impl handlers::OnOff for EscapeAndOff {
    fn on_activate(&mut self, output: &mut impl USBKeyOut) {
        for id in self.ids.iter(){
            output.state().disable_handler(*id);
        }
        output.state().set_modifier(Modifier::Shift, false);
        output.state().set_modifier(Modifier::Ctrl, false);
        output.state().set_modifier(Modifier::Alt, false);
        output.state().set_modifier(Modifier::Gui, false);
        output.send_keys(&[KeyCode::Escape]);
    }
    fn on_deactivate(&mut self, _output: &mut impl USBKeyOut) {}
}

    let mut ea = EscapeAndOff::new();
    ea.ids.push(numpad_id);
    ea.ids.push(umlaut_id);
    k.output.debug(&format!("D{}", ALLOCATOR.get()));

    k.add_handler(Box::new(handlers::PressReleaseMacro::new(
        KeyCode::Escape, ea)));

    let dvorak_id = k.add_handler(premade::dvorak());
/*
    k.add_handler(Box::new(handlers::LongTap::new(
       KeyCode::F1, 
       KeyCode::F1, 
       premade::ActionToggleHandler{id: dvorak_id},
       5000)));
       */

//$! -> 41, yeah.

    k.output.debug(&format!("E{}", ALLOCATOR.get()));


    k.output.state().enable_handler(dvorak_id);

    k.add_handler(premade::one_shot_shift(400, 1000));
    k.add_handler(premade::one_shot_ctrl(400, 1000));
    k.add_handler(premade::one_shot_alt(400, 1000));
    k.add_handler(premade::one_shot_gui(400, 1000));
    k.output.debug(&format!("F{}", ALLOCATOR.get()));
    k.add_handler(Box::new(premade::CopyPaste{}));
    k.add_handler(Box::new(handlers::UnicodeKeyboard::new()));
    k.add_handler(Box::new(handlers::USBKeyboard::new()));
    k.add_handler(Box::new(debug_handlers::TranslationHelper {}));
    k.output.debug(&format!("G{}", ALLOCATOR.get()));
    return k;
}



#[app(device = stm32f1xx_hal::stm32)]
const APP: () = {
    static mut USB_DEV: UsbDevice<'static, UsbBusType> = ();
    //static mut USB_CLASS: KeyboardHidClass = ();
    static mut TIMER: timer::Timer<stm32::TIM3> = ();
    static mut TIMER_MS: timer::Timer<stm32::TIM4> = ();
    static mut RX: serial::Rx<stm32f1::stm32f103::USART1> = ();
    static mut LED: Led = ();
    static mut MATRIX: Matrix = ();
    static mut DEBOUNCER: Debouncer = ();
    static mut K2K: K2KKeyboard<'static, USBOut> = ();
    static mut TRANSLATION: Vec<u32> = ();
    static mut LAST_TIME_MS: u32 = 0;
    static mut CURRENT_TIME_MS: u32 = 0;
    static mut HEAPSIZE: u32 = 0;

    #[init]
    fn init() -> init::LateResources {
        let start = rt::heap_start() as usize;
        let size = 6 * 1024; // in bytes
        unsafe { ALLOCATOR.0.init(start, size) }

        static mut USB_BUS: Option<bus::UsbBusAllocator<UsbBusType>> = None;

        let mut flash = device.FLASH.constrain();
        let mut rcc = device.RCC.constrain();

        let clocks = rcc
            .cfgr
            .use_hse(8.mhz())
            .sysclk(48.mhz())
            .pclk1(24.mhz())
            .freeze(&mut flash.acr);

        let mut gpioa = device.GPIOA.split(&mut rcc.apb2);
        let mut gpiob = device.GPIOB.split(&mut rcc.apb2);
        let mut gpioc = device.GPIOC.split(&mut rcc.apb2);

        let mut led = gpioc.pc13.into_push_pull_output(&mut gpioc.crh);
        led.set_high().ok();

        // BluePill board has a pull-up resistor on the D+ line.
        // Pull the D+ pin down to send a RESET condition to the USB bus.
        let mut usb_dp = gpioa.pa12.into_push_pull_output(&mut gpioa.crh);
        usb_dp.set_low().ok();
        cortex_m::asm::delay(clocks.sysclk().0 / 100);

        let usb_dm = gpioa.pa11;
        let usb_dp = usb_dp.into_floating_input(&mut gpioa.crh);

        unsafe {
            USB_BUS = Some(UsbBus::new(device.USB, (usb_dm, usb_dp)));
        }
        let usb_bus = unsafe { USB_BUS.as_ref().unwrap() };

        let usb_class = hid::HidClass::new(Keyboard::new(), &usb_bus);
        let usb_dev = UsbDeviceBuilder::new(usb_bus, UsbVidPid(VID, PID))
            .manufacturer("TyberiusPrime")
            .product("K2KAdvantage")
            .serial_number(env!("CARGO_PKG_VERSION"))
            .build();

        let mut timer = timer::Timer::tim3(device.TIM3, 100.hz(), clocks, &mut rcc.apb1); //todo, do this faster ;)
        timer.listen(timer::Event::Update);

        let mut timer_ms = timer::Timer::tim4(device.TIM4, 1000.hz(), clocks, &mut rcc.apb1);
        timer_ms.listen(timer::Event::Update);

        let pin_tx = gpioa.pa9.into_alternate_push_pull(&mut gpioa.crh);
        let pin_rx = gpioa.pa10;
        let mut afio = device.AFIO.constrain(&mut rcc.apb2);

        let ser = serial::Serial::usart1(
            device.USART1,
            (pin_tx, pin_rx),
            &mut afio.mapr,
            9_600.bps(),
            clocks,
            &mut rcc.apb2,
        );
        let (tx, rx) = ser.split();
        let pre_matrix = ALLOCATOR.get();

        let matrix = Matrix::new(
            vec![
                gpioa.pa8.into_pull_up_input(&mut gpioa.crh).downgrade(),
                gpioa.pa15.into_pull_up_input(&mut gpioa.crh).downgrade(),
            ],
            vec![
                gpiob.pb12.into_pull_up_input(&mut gpiob.crh).downgrade(),
                gpiob.pb13.into_pull_up_input(&mut gpiob.crh).downgrade(),
                gpiob.pb14.into_pull_up_input(&mut gpiob.crh).downgrade(),
                gpiob.pb15.into_pull_up_input(&mut gpiob.crh).downgrade(),
                gpiob.pb6.into_pull_up_input(&mut gpiob.crl).downgrade(),
                gpiob.pb4.into_pull_up_input(&mut gpiob.crl).downgrade(),
                gpiob.pb5.into_pull_up_input(&mut gpiob.crl).downgrade(),
            ],
            vec![
                gpioa
                    .pa7
                    .into_open_drain_output_with_state(
                        &mut gpioa.crl,
                        stm32f1xx_hal::gpio::State::High,
                    )
                    .downgrade(), //brown
                gpioa
                    .pa6
                    .into_open_drain_output_with_state(
                        &mut gpioa.crl,
                        stm32f1xx_hal::gpio::State::High,
                    )
                    .downgrade(), //purple
                gpioa
                    .pa5
                    .into_open_drain_output_with_state(
                        &mut gpioa.crl,
                        stm32f1xx_hal::gpio::State::High,
                    )
                    .downgrade(), //red
                gpioa
                    .pa4
                    .into_open_drain_output_with_state(
                        &mut gpioa.crl,
                        stm32f1xx_hal::gpio::State::High,
                    )
                    .downgrade(), //white
                gpioa
                    .pa3
                    .into_open_drain_output_with_state(
                        &mut gpioa.crl,
                        stm32f1xx_hal::gpio::State::High,
                    )
                    .downgrade(), //yellow
                gpioa
                    .pa2
                    .into_open_drain_output_with_state(
                        &mut gpioa.crl,
                        stm32f1xx_hal::gpio::State::High,
                    )
                    .downgrade(), //blue
            ],
            vec![
                gpiob
                    .pb11
                    .into_open_drain_output_with_state(
                        &mut gpiob.crh,
                        stm32f1xx_hal::gpio::State::High,
                    )
                    .downgrade(), // black
                gpiob
                    .pb10
                    .into_open_drain_output_with_state(
                        &mut gpiob.crh,
                        stm32f1xx_hal::gpio::State::High,
                    )
                    .downgrade(), // red
                gpiob
                    .pb1
                    .into_open_drain_output_with_state(
                        &mut gpiob.crl,
                        stm32f1xx_hal::gpio::State::High,
                    )
                    .downgrade(), // blue
                gpiob
                    .pb0
                    .into_open_drain_output_with_state(
                        &mut gpiob.crl,
                        stm32f1xx_hal::gpio::State::High,
                    )
                    .downgrade(), // green
            ],
        );
        let mut  output = USBOut::new(usb_class, tx);
        output.tx.writeln(&format!("pre_matrix {}", ALLOCATOR.get()));
        output.tx.writeln(&format!("matrix {}", ALLOCATOR.get()));

        let debouncer = Debouncer::new(matrix.len());
        output.tx.writeln(&format!("debouncer {}", ALLOCATOR.get()));
        let mut translation = get_translation();
        for ii in translation.len()..matrix.len() {
            translation.push(ii.try_into().unwrap());
        }
        output.tx.writeln(&format!("translation {}", ALLOCATOR.get()));

        let k2k = get_keytokey(output);

        init::LateResources {
            USB_DEV: usb_dev,
            //USB_CLASS: usb_class,
            TIMER: timer,
            TIMER_MS: timer_ms,
            RX: rx,
            LED: led,
            MATRIX: matrix,
            DEBOUNCER: debouncer,
            K2K: k2k,
            TRANSLATION: translation,
        }
    }



    #[interrupt(priority = 3, resources = [USB_DEV, K2K])]
    fn USB_HP_CAN_TX() {
        usb_poll(&mut resources.USB_DEV, &mut resources.K2K.output.usb_class);
        
    }

    #[interrupt(priority = 3, resources = [USB_DEV, K2K])]
    fn USB_LP_CAN_RX0() {
        usb_poll(&mut resources.USB_DEV, &mut resources.K2K.output.usb_class);
        if let Some(report) = resources.K2K.output.buffer.pop_front() {
            match resources.K2K.output.usb_class.write(report.as_bytes()) {
            Ok(0) => { //try again?
                resources.K2K.output.buffer.push_back(report);
            }
            Ok(_i) => {}, //complete report, presumably
            Err(_) => {},
        };
        }
    }

    #[interrupt(priority = 2, resources = [CURRENT_TIME_MS, TIMER_MS])]
    fn TIM4() {
        resources.TIMER_MS.clear_update_interrupt_flag();
        *resources.CURRENT_TIME_MS += 1;
    }

    #[interrupt(priority = 1, resources = [
        CURRENT_TIME_MS,
        DEBOUNCER,
        K2K,
        LAST_TIME_MS,
        LED,
        MATRIX,
        TRANSLATION,
        TIMER,
        HEAPSIZE
    ])]
    fn TIM3() {
        resources.TIMER.clear_update_interrupt_flag();
        #[allow(deprecated)]
        resources.LED.toggle();
        resources.MATRIX.read_matrix();

        let states = &resources.MATRIX.output;
        let mut nothing_changed = true;
        let current_time_ms = resources.CURRENT_TIME_MS.lock(|ct| *ct);
        let delta = current_time_ms
            .overflowing_sub(*resources.LAST_TIME_MS)
            .0
            .clamp(0, 2u32.pow(16) - 1);
        let debouncer = &mut *resources.DEBOUNCER;
        let translation =& *resources.TRANSLATION;
        let mut update_last_time = false;
        let last_hs = *resources.HEAPSIZE;
        let hs = ALLOCATOR.get();
        resources.K2K.lock(|k2k| {
            matrix::Matrix::debug_serial(&states, &mut k2k.output.tx);

            if hs != last_hs {
                k2k.output.tx.writeln(&format!("heap {}", hs));
            }


            for (ii, pressed) in states.iter().enumerate() {
                match debouncer.update(ii, pressed) {
                    DebounceResult::NoChange => {}
                    DebounceResult::Pressed => {
                        nothing_changed = false;
                        k2k.add_keypress(translation[ii], delta as u16);
                        update_last_time = true;
                        k2k.handle_keys().ok();
                        k2k.clear_unhandled();
                    }
                    DebounceResult::Released => {
                        nothing_changed = false;
                        k2k.add_keyrelease(translation[ii], delta as u16);
                        update_last_time = true;
                        k2k.handle_keys().ok();
                        k2k.clear_unhandled();
                    }
                }
            }
            if nothing_changed {




                k2k.add_timeout(delta as u16);
                k2k.handle_keys().ok();
                k2k.clear_unhandled();
            }
        });
        if update_last_time {
            *resources.LAST_TIME_MS = current_time_ms;
        }
        *resources.HEAPSIZE = hs;
    }
};

fn usb_poll(usb_dev: &mut UsbDevice<'static, UsbBusType>, keyboard: &mut KeyboardHidClass) {
    if usb_dev.poll(&mut [keyboard]) {
        keyboard.poll();
    }
}
