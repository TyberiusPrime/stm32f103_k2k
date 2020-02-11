# Building instructions

Take one handwired keyboard matrix,
attach to a blue pill
adjust for the pins you used (avoid PA11/PA12 - used for USB,
and possibly PA9/PA10 used for serial & bootloader),
adjust the pins and layout in main::App::init (line 546 or thereabout),
throw out my configuration from main::get_key_to_key, 
and uncomment the translation helper to get started.
