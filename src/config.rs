use no_std_compat::prelude::v1::*;

use crate::usbout::USBOut;
use crate::StringSender;
use keytokey::{
    handlers, debug_handlers, iter_unhandled_mut, 
    Event, EventStatus, KeyCode, Keyboard, ProcessKeys,
    USBKeyOut, UnicodeSendMode,
};
// pins for matrix - see main.rs, otherwise the borrow checker has a fit :(

pub fn get_translation() -> Vec<u32> {
    vec![
        0, //one of the missing nes?
        //00000001
        KeyCode::V.into(),
        //00000002
        KeyCode::SColon.into(),
        //00000003
        KeyCode::F8.into(),
        //00000004
        KeyCode::K.into(),
        //00000005
        KeyCode::Space.into(),
        //00000006
        KeyCode::BSpace.into(),
        //00000007
        KeyCode::Q.into(),
        //00000008
        KeyCode::F2.into(),
        //9
        9,
        //0000000a
        KeyCode::C.into(),
        //0000000b
        KeyCode::L.into(),
        //c
        0xc,
        //0000000d
        KeyCode::J.into(),
        //0000000e
        KeyCode::LGui.into(),
        //0000000f
        KeyCode::Delete.into(), // which one is this?
        //00000010
        KeyCode::A.into(),
        //00000011
        KeyCode::F1.into(),
        //12
        0x12,
        //13
        0x13,
        //14
        0x14,
        //15
        0x15,
        //16
        0x16,
        //17
        0x17,
        //18
        0x18,
        //19
        0x19,
        //1A
        0x1a,
        //1B
        0x1b,
        //0000001c
        KeyCode::E.into(),
        //0000001d
        KeyCode::Slash.into(),
        //0000001e
        KeyCode::Delete.into(),
        //0000001f
        KeyCode::Comma.into(),
        //0x20
        KeyCode::Enter.into(),
        //00000021
        KeyCode::LAlt.into(),
        //00000022
        KeyCode::Z.into(),
        //00000023
        KeyCode::Equal.into(),
        //24
        0x24,
        //00000025
        KeyCode::Right.into(),
        //00000026
        KeyCode::Dot.into(),
        //00000027
        KeyCode::Quote.into(),
        //00000028
        KeyCode::M.into(),
        //00000029
        KeyCode::End.into(),
        //0000002a
        KeyCode::LCtrl.into(),
        //0000002b
        KeyCode::X.into(),
        //2c
        0x2c,
        //2d
        0x2d,
        //0000002e
        KeyCode::Left.into(),
        //0000002f
        KeyCode::LBracket.into(),
        //00000030
        KeyCode::RShift.into(),
        //00000031
        KeyCode::Up.into(),
        //00000032
        KeyCode::RCtrl.into(),
        //00000033
        KeyCode::PgDown.into(),
        //00000034
        KeyCode::Grave.into(),
        //00000035
        KeyCode::LShift.into(),
        //36
        0,//palm1
        //00000037
        KeyCode::Kb3.into(),
        //00000038
        KeyCode::Kb0.into(),
        //00000039
        KeyCode::F12.into(),
        //0000003a
        KeyCode::Kb8.into(),
        //0000003b
        KeyCode::Kb6.into(),
        //0000003c
        KeyCode::Kb5.into(),
        //0000003d
        KeyCode::Kb1.into(),
        //0000003e
        KeyCode::F6.into(),
        //3f
        0, ///palm 2
        .into()
        //00000040
        KeyCode::Kb4.into(),
        //00000041
        KeyCode::Kb9.into(),
        //00000042
        KeyCode::F11.into(),
        //00000043
        KeyCode::Kb7.into(),
        //00000044
        KeyCode::Y.into(),
        //00000045
        KeyCode::T.into(),
        //00000046
        KeyCode::Kb2.into(),
        //00000047
        KeyCode::F5.into(),
        //48,
        0, //palm3
        //00000049
        KeyCode::R.into(),
        //0000004a
        KeyCode::P.into(),
        //0000004b
        KeyCode::F10.into(),
        //0000004c
        KeyCode::I.into(),
        //0000004d
        KeyCode::H.into(),
        //0000004e
        KeyCode::G.into(),
        //0000004f
        KeyCode::W.into(),
        //00000050
        KeyCode::F3.into(),
        //51
        0x51,
        //00000052
        KeyCode::F.into(),
        //00000053
        KeyCode::O.into(),
        //00000054
        KeyCode::F9.into(),
        //00000055
        KeyCode::U.into(),
        //00000056
        KeyCode::N.into(),
        //00000057
        KeyCode::B.into(),
        //00000058
        KeyCode::S.into(),
        //00000059
        KeyCode::F4.into(),
    ]
    /*

    .into()
    01	KeyCode::.into()
    01	KeyCode::
            not working: caps, D, left backslash, down, [, pgup, Home, F7
            */
    //position -> keycode.
    //if too short, automatically extended with running number.
    // So you can start with an empty one, + key2key::debug_handlers::TranslationHelper
    // in the layout
}
pub struct AAA {}
impl ProcessKeys<USBOut> for AAA {
    fn process_keys(&mut self, events: &mut Vec<(Event, EventStatus)>, output: &mut USBOut) -> () {
        for (e, status) in iter_unhandled_mut(events) {
            // *status = EventStatus::Handled;
            match e {
                Event::KeyRelease(kc) => {
                    if kc.keycode == 32 {
                        output.state().unicode_mode = UnicodeSendMode::Linux;
                        output.send_string("Hello0123");
                        *status = EventStatus::Handled;
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
 //   k.add_handler(Box::new(AAA {}));
    k.add_handler(Box::new(handlers::USBKeyboard::new()));
    k.add_handler(Box::new(debug_handlers::TranslationHelper {}));
    return k;
}
