#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

#[path = "../example_common.rs"]
mod example_common;

use embassy::{executor::Spawner, io::AsyncWriteExt};
use embassy::interrupt_take;
use embassy_stm32::{interrupt, lpuart};
use embassy::traits::{
    adapter::BlockingAsync,
    uart::{Read, Write},
};
use embassy_stm32::dma::NoDma;
use embassy_stm32::lpuart::{LpUart, BufferedLpUart, State};
use embassy_stm32::usart::{Config, Uart};
use embassy_stm32::Peripherals;
use example_common::*;
use heapless::String;
use core::fmt::Write as coreWrite;
use core::mem::MaybeUninit;
use embassy_stm32::peripherals::LPUART1;
use embassy::io::AsyncBufReadExt;

#[embassy::main]
async fn main(_spawner: Spawner, p: Peripherals) {
    let config = Config::default();

    static mut TX_BUFFER: [u8; 512] = [0; 512];
    static mut RX_BUFFER: [u8; 512] = [0; 512];
    static mut STATE: MaybeUninit<State<LPUART1>> = MaybeUninit::uninit();

    let mut lpuart = unsafe {
        let lpuart1 = LpUart::new(p.LPUART1, p.PB10, p.PB11, NoDma, NoDma, Default::default());
        BufferedLpUart::new(STATE.write(State::new()), lpuart1, interrupt_take!(LPUART1), &mut TX_BUFFER, &mut RX_BUFFER)
    };

    for n in 0u32.. {
        // let mut s: String<128> = String::new();
        // core::write!(&mut s, "Hello DMA World {}!\r\n", n).unwrap();
        let mut buf_in: [u8; 5] = [0u8; 5];
        let mut buf_out: [u8; 5] = [1,2,3,4,5];

        info!("Writing...");
        lpuart.write_all(&buf_out).await.ok();
        let n = lpuart.read(&mut buf_in).await.unwrap();
        lpuart.write_all(&buf_in[..n]).await.ok();

        info!("wrote DMA {:?}", &buf_in[..n]);
    }
    // unwrap!(lpuart.write(b"Hello Embassy World!\r\n").await);
    // info!("wrote Hello, starting echo");

    // let mut buf = [0u8; 1];
    // loop {
    //     unwrap!(lpuart.read(&mut buf).await);
    //     unwrap!(lpuart.write(&buf).await);
    // }
}
