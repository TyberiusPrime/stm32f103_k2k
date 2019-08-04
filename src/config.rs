use no_std_compat::prelude::v1::*;

use crate::usbout::USBOut;
use crate::StringSender;
use keytokey::{Keyboard, ProcessKeys, Event, EventStatus, USBKeyOut, debug_handlers, iter_unhandled_mut, KeyCode,
UnicodeSendMode};
// pins for matrix - see main.rs, otherwise the borrow checker has a fit :(

pub fn get_translation() -> Vec<u32> {
    vec![
        0,
        //position -> keycode.
        //if too short, automatically extended with running number.
        // So you can start with an empty one, + key2key::debug_handlers::TranslationHelper
        // in the layout
    ]
}
pub struct AAA {}
impl ProcessKeys<USBOut> for AAA {
    fn process_keys(&mut self, events: &mut Vec<(Event, EventStatus)>, output: &mut USBOut) -> () {
        for (e, status) in iter_unhandled_mut(events) {
            *status = EventStatus::Handled;
            match e {
                Event::KeyRelease(kc) => {
                    if kc.keycode == 32 {
                        output.state().unicode_mode = UnicodeSendMode::WinCompose;
                        output.send_string("Hello0123");
                    }
                    else {
                    output.tx.writeln("Key release!");
                    output.register_key(KeyCode::A);
                    output.tx.writeln("Reg A");
                    output.send_registered();
                    output.tx.writeln("Send B");
                    output.register_key(KeyCode::B);
                    output.send_registered();
                    output.tx.writeln("Send C");
                    output.register_key(KeyCode::C);
                    output.send_registered();
                    
                    output.tx.writeln("Send empty");
                    output.send_empty();
                    output.tx.writeln("Done sending");
                    }
                }
                _ => {
                    *status = EventStatus::Handled;
                }
            };
        }
    }
}


pub fn get_keytokey<'a>(output: USBOut) -> Keyboard<'a, USBOut> {
    let mut k = Keyboard::new(output);
    k.add_handler(Box::new(AAA{}));
    return k;
}
