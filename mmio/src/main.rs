/*
//! Prints "Hello, world!" on the host console using semihosting

#![no_main]
#![no_std]

use panic_halt as _;

use cortex_m_rt::entry;
use cortex_m_semihosting::{debug, hprintln};

#[entry]
fn main() -> ! {
    hprintln!("Hello, world!").unwrap();

    // exit QEMU
    // NOTE do not run this on hardware; it can corrupt OpenOCD state
    debug::exit(debug::EXIT_SUCCESS);

    loop {}
}
*/



//#![deny(unsafe_code)]
#![no_main]
#![no_std]
#![allow(warnings)]

//extern crate panic_halt as _;

mod interrupts;
//mod gd32e23x;
pub mod gd32e23x_rcu;
pub mod gd32e23x_gpio;

use core::mem::size_of;
//use cortex_m::peripheral::{syst, Peripherals};
use cortex_m_rt::entry;
//use panic_halt as _;

use crate::gd32e23x_gpio::GPIO_A;
use crate::gd32e23x_gpio::gpio_mode_set;
use crate::gd32e23x_gpio::gpio_output_options_set;
use crate::gd32e23x_gpio::gpio_bit_reset;
use crate::gd32e23x_gpio::GPIO_PIN_8;
use crate::gd32e23x_gpio::GPIO_PIN_11;
use crate::gd32e23x_gpio::GPIO_PIN_12;
use crate::gd32e23x_gpio::GPIO_PIN_15;
use crate::gd32e23x_gpio::GPIO_MODE_OUTPUT;
use crate::gd32e23x_gpio::GPIO_PUPD_NONE;
use crate::gd32e23x_gpio::GPIO_OTYPE_PP;
use crate::gd32e23x_gpio::GPIO_OSPEED_50MHZ;
use crate::gd32e23x_rcu::rcu_periph_clock_enable;



//#define RCU_REGIDX_BIT(regidx, bitpos)      (((uint32_t)(regidx)<<6) | (uint32_t)(bitpos))
macro_rules! RCU_REGIDX_BIT    { ($regidx:expr,$bitpos:expr) => ((($regidx)<<6) | $bitpos) }
const RCU_GPIOA: u32 = RCU_REGIDX_BIT!(0x14, 17);

const GPIO_PORT: [u32; 4] = [GPIO_A, GPIO_A, GPIO_A, GPIO_A];
const GPIO_PIN: [u32; 4] = [GPIO_PIN_8, GPIO_PIN_11, GPIO_PIN_12, GPIO_PIN_15];
const GPIO_CLK: [u32; 4] = [RCU_GPIOA, RCU_GPIOA, RCU_GPIOA, RCU_GPIOA];
/*
const rcu_periph_enum COM_CLK[COMn]  = {EVAL_COM_CLK};
const uint32_t COM_TX_PIN[COMn]      = {EVAL_COM_TX_PIN};
const uint32_t COM_RX_PIN[COMn]      = {EVAL_COM_RX_PIN};

static const uint32_t KEY_PORT[KEYn]        = {WAKEUP_KEY_GPIO_PORT, TAMPER_KEY_GPIO_PORT};
static const uint32_t KEY_PIN[KEYn]         = {WAKEUP_KEY_PIN, TAMPER_KEY_PIN};
static const rcu_periph_enum KEY_CLK[KEYn]  = {WAKEUP_KEY_GPIO_CLK, TAMPER_KEY_GPIO_CLK};
static const exti_line_enum KEY_EXTI_LINE[KEYn] = {WAKEUP_KEY_EXTI_LINE, TAMPER_KEY_EXTI_LINE};
static const uint8_t KEY_PORT_SOURCE[KEYn]      = {WAKEUP_KEY_EXTI_PORT_SOURCE, TAMPER_KEY_EXTI_PORT_SOURCE};
static const uint8_t KEY_PIN_SOURCE[KEYn]       = {WAKEUP_KEY_EXTI_PIN_SOURCE, TAMPER_KEY_EXTI_PIN_SOURCE};
static const uint8_t KEY_IRQn[KEYn]             = {WAKEUP_KEY_EXTI_IRQn, TAMPER_KEY_EXTI_IRQn};
*/



//#[panic_handler]
//fn panic(_info: &core::panic::PanicInfo) -> ! {
//    loop {}
//}



/**!
    \brief      configure led GPIO
    \param[in]  lednum: specify the led to be configured
      \arg        LED1
      \arg        LED2
      \arg        LED3
      \arg        LED4
    \param[out] none
    \retval     none
**/
fn gd_eval_led_init(lednum: usize) {
    /* enable the led clock */
    rcu_periph_clock_enable(GPIO_CLK[lednum]);
    /* configure led GPIO port */ 
    gd32e23x_gpio::gpio_mode_set(GPIO_PORT[lednum], GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO_PIN[lednum]);
    gd32e23x_gpio::gpio_output_options_set(GPIO_PORT[lednum], GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO_PIN[lednum]);
    gd32e23x_gpio::gpio_bit_reset(GPIO_PORT[lednum], GPIO_PIN[lednum]);
}



#[entry]
fn main() -> ! {
//    let peripherals = Peripherals::take().unwrap();
//    let mut systick = peripherals.SYST;

    gd32e23x_gpio::gpio_mode_set(GPIO_A, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO_PIN_8);
    gd32e23x_gpio::gpio_mode_set(GPIO_A, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO_PIN_11);
    gd32e23x_gpio::gpio_mode_set(GPIO_A, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO_PIN_12);
    gd32e23x_gpio::gpio_mode_set(GPIO_A, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO_PIN_15);



    let gpioa_odr = 0x4800_0014 as *mut u32;
    unsafe {
        gpioa_odr.write_volatile(1 << 5);
    }

//    let gpioa = dp.GPIOA.split();
//    let mut led = gpioa.pa15.into_push_pull_output();
//    led.set_high().unwrap();



    loop {
        gd32e23x_gpio::gpio_bit_toggle(gd32e23x_gpio::GPIO_A, gd32e23x_gpio::GPIO_PIN_8);
    }
}