del k2k_advantage.bin
cargo build --release && arm-none-eabi-objcopy -O binary target/thumbv7m-none-eabi/release/k2k_advantage k2k_advantage.bin && stm32loader -p COM10 -f F1 -g 0x08000000 -e -v -w k2k_advantage.bin

