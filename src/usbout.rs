use keytokey::{KeyCode, KeyboardState, USBKeyOut};

pub struct USBOut {
    state: KeyboardState,
}

impl USBOut {
    pub fn new() -> USBOut {
        USBOut {
            state: KeyboardState::new(),
        }
    }
}

impl USBKeyOut for USBOut {
    /// send these USB Keycodes concurrently rigth away.
    fn send_keys(&mut self, keys: &[KeyCode]) {}
    /// register these USB keycodes to be send on .send_registered
    fn register_key(&mut self, key: KeyCode) {}
    /// send registered keycodes (or an empty nothing-pressed status)
    fn send_registered(&mut self) {}

    /// helper that sends an empty status
    fn send_empty(&mut self) {}

    /// retrieve a mutable KeyboardState
    fn state(&mut self) -> &mut KeyboardState {
        return &mut self.state;
    }
}
