#![no_std]
#![no_main]

use panic_halt as _;
use cortex_m_rt::entry;
use cortex_m_rt::exception;

#[entry]
fn main() -> ! {
    loop {}
}

#[exception]
fn SysTick() {
	todo!();
}
