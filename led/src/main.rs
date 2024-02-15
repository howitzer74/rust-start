#![no_main]
#![no_std]

use panic_halt as _;

use cortex_m_rt::entry;
use cortex_m::peripheral::{syst, Peripherals};

#[entry]
fn main() -> ! {
    let peripherals = Peripherals::take().unwrap();
    let mut systick = peripherals.SYST;

    systick.set_clock_source(syst::SystClkSource::Core);
    systick.set_reload(1_000);
    systick.clear_current();
    systick.enable_counter();

    while !systick.has_wrapped() {
        // Loop
    }

    unsafe {
//        let mut val: u32;
        let val: u32;
        let reg = 0x4800_0018 as *mut u32;

//        val = reg.read_volatile();

        val = 0x5555_5555;
        reg.write_volatile(val);

//        val = reg.read_volatile();
    }

    loop {}
}