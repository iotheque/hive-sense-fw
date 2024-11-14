![GitHub](https://img.shields.io/github/license/pcurt/hive-sense-fw)
![GitHub tag (with filter)](https://img.shields.io/github/v/tag/iotheque/hive-sense-fw)
![GitHub Actions Workflow Status](https://img.shields.io/github/actions/workflow/status/iotheque/hive-sense-fw/rust.yml?labelColor=1C2C2E&label=CI&logo=github&style=flat-square)


# hive-sense-fw
This is the hive-sense firmware.
Hive sense is an IoT product developed by [IOTheque](https://home.iotheque.com) company. It is based on a ESP32S3 chip.
It monitors hives and communicates using LoRaWAN network.
This firmware is written in **Rust**, it is a **no_std** application 


## Build the project

First install rust using the official procedure https://rustup.rs

Then install RISC-V and Xtensa Targets : https://docs.esp-rs.org/book/installation/riscv-and-xtensa.html#risc-v-and-xtensa-targets


Get the project
```sh
git clone https://github.com/pcurt/hive-sense-fw.git
```

Build the project
```sh
cargo build --release
```

Flash and start the application 
Build the project
```sh
cargo run --release
```
