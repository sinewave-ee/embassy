#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

#[path = "../example_common.rs"]
mod example_common;

use embassy_stm32::dma::NoDma;
use embassy_stm32::usart::{Config, Uart};
use example_common::*;
use embassy_stm32::lpuart::LpUart;

#[cortex_m_rt::entry]
fn main() -> ! {
    info!("Hello World!");

    let p = embassy_stm32::init(Default::default());

    let mut lpuart = LpUart::new(p.LPUART1, p.PB10, p.PB11, NoDma, NoDma, Default::default());

    let config = Config::default();
    // let mut usart = Uart::new(p.UART4, p.PA1, p.PA0, NoDma, NoDma, config);

    // unwrap!(usart.blocking_write(b"Hello Embassy World!\r\n"));
    info!("hi");
    unwrap!(lpuart.blocking_write(b"Hello Embassy World!\r\n"));
    info!("wrote Hello, starting echo");

    let mut buf = [0u8; 1];
    loop {
        unwrap!(lpuart.blocking_read(&mut buf));
        unwrap!(lpuart.blocking_write(&buf));
    }
}
