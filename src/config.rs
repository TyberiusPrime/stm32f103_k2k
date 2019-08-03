use no_std_compat::prelude::v1::*;

use crate::usbout::USBOut;
use keytokey;
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

pub fn get_keytokey<'a>(output: USBOut) -> keytokey::Keyboard<'a, USBOut> {
    let mut k = keytokey::Keyboard::new(output);
    k.add_handler(Box::new(keytokey::debug_handlers::TranslationHelper {}));
    return k;
}
