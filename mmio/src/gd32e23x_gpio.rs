//mod gd32e23x;
//use crate::gd32e23x;
//mod gd32e23x_rcu;
use volatile_register::RW;

const GPIO_BASE: usize = 0x4800_0000;

/// GPIO port peripheral addresses
//const GPIO_A: usize = GPIO_BASE + 0x0000_0000;
//const GPIO_B: usize = GPIO_BASE + 0x0000_0400;
//const GPIO_C: usize = GPIO_BASE + 0x0000_0800;
//const GPIO_F: usize = GPIO_BASE + 0x0000_1400;
pub const GPIO_A: u32 = 0x4800_0000;
pub const GPIO_B: u32 = 0x4800_0400;
pub const GPIO_C: u32 = 0x4800_0800;
pub const GPIO_F: u32 = 0x4800_1400;

macro_rules! GPIO_CTL    { ($reg:expr) => ( $reg + 0x00; ) }
macro_rules! GPIO_OMODE  { ($reg:expr) => ( $reg + 0x04; ) }
macro_rules! GPIO_OSPD   { ($reg:expr) => ( $reg + 0x08; ) }
macro_rules! GPIO_PUD    { ($reg:expr) => ( $reg + 0x0C; ) }
macro_rules! GPIO_ISTAT  { ($reg:expr) => ( $reg + 0x10; ) }
macro_rules! GPIO_OCTL   { ($reg:expr) => ( $reg + 0x14; ) }
macro_rules! GPIO_BOP    { ($reg:expr) => ( $reg + 0x18; ) }
macro_rules! GPIO_LOCK   { ($reg:expr) => ( $reg + 0x1C; ) }
macro_rules! GPIO_AFSEL0 { ($reg:expr) => ( $reg + 0x20; ) }
macro_rules! GPIO_AFSEL1 { ($reg:expr) => ( $reg + 0x24; ) }
macro_rules! GPIO_BC     { ($reg:expr) => ( $reg + 0x28; ) }
macro_rules! GPIO_TG     { ($reg:expr) => ( $reg + 0x2C; ) }



macro_rules! BITS { ($start:expr,$end:expr) => ((0xFFFF_FFFF << $start) & (0xFFFF_FFFF >> (31 - $end))) }
//#define GET_BITS(regval, start, end) (((regval) & BITS((start),(end))) >> (start))
//#define GET_BITS(regval, start, end) (((regval) & BITS((start),(end))) >> (start))



/* GPIO mode configuration values */
macro_rules! GPIO_MODE_SET    { ($n:expr,$mode:expr) => ( $mode << (2 * $n) ) }
pub(crate) use GPIO_MODE_SET;
macro_rules! GPIO_MODE_MASK   { ($reg:expr) => (3 << (2 * $reg)) }
pub(crate) use GPIO_MODE_MASK;


/* GPIO pull-up/pull-down values */
macro_rules! GPIO_PUPD_SET    { ($n:expr,$mode:expr) => ( $mode << (2 * $n) ) }
pub(crate) use GPIO_PUPD_SET;
macro_rules! GPIO_PUPD_MASK    { ($n:expr) => ( 3 << (2 * $n) ) }
pub(crate) use GPIO_PUPD_MASK;

/* GPIO output speed values */
macro_rules! GPIO_OSPEED_SET  { ($n:expr,$mode:expr) => ( $mode << (2 * $n) ) }
pub(crate) use GPIO_OSPEED_SET;
macro_rules! GPIO_OSPEED_MASK    { ($n:expr) => ( 3 << (2 * $n) ) }
pub(crate) use GPIO_OSPEED_MASK;

/* GPIO output type */
pub const GPIO_OTYPE_PP: u8 = 0x00;    // push pull mode
pub const GPIO_OTYPE_OD: u8 = 0x01;    // open drain mode

/* GPIO output max speed value */
macro_rules! OSPD_OSPD0    { ($regval:expr) => ( $regval & BITS!(0,1) ) }
pub(crate) use OSPD_OSPD0;
pub const GPIO_OSPEED_2MHZ:  u32 = OSPD_OSPD0!(0);
pub const GPIO_OSPEED_10MHZ: u32 = OSPD_OSPD0!(1);
pub const GPIO_OSPEED_50MHZ: u32 = OSPD_OSPD0!(3);

/* GPIO alternate function values */
//#define GPIO_AFR_SET(n, af)        ((uint32_t)((uint32_t)(af) << (4U * (n))))
//#define GPIO_AFR_MASK(n)           (0xFU << (4U * (n)))

/* output mode definitions */
macro_rules! CTL_CLTR    { ($regval:expr) => ( BITS!(0,1) & ($regval << 0) ) }
pub const GPIO_MODE_INPUT:  u32 = CTL_CLTR!(0);
pub const GPIO_MODE_OUTPUT: u32 = CTL_CLTR!(1);
pub const GPIO_MODE_AF:     u32 = CTL_CLTR!(2);
pub const GPIO_MODE_ANALOG: u32 = CTL_CLTR!(3);

/* pull-up/pull-down definitions */
macro_rules! PUD_PUPD    { ($regval:expr) => ( BITS!(0,1) & ($regval << 0) ) }
pub const GPIO_PUPD_NONE:     u32 = PUD_PUPD!(0);
pub const GPIO_PUPD_PULLUP:   u32 = PUD_PUPD!(1);
pub const GPIO_PUPD_PULLDOWN: u32 = PUD_PUPD!(2);



pub const GPIO_PIN_0:   u32 = 0x0000_0001; // GPIO pin 0
pub const GPIO_PIN_1:   u32 = 0x0000_0002; // GPIO pin 1
pub const GPIO_PIN_2:   u32 = 0x0000_0004; // GPIO pin 2
pub const GPIO_PIN_3:   u32 = 0x0000_0008; // GPIO pin 3
pub const GPIO_PIN_4:   u32 = 0x0000_0010; // GPIO pin 4
pub const GPIO_PIN_5:   u32 = 0x0000_0020; // GPIO pin 5
pub const GPIO_PIN_6:   u32 = 0x0000_0040; // GPIO pin 6
pub const GPIO_PIN_7:   u32 = 0x0000_0080; // GPIO pin 7
pub const GPIO_PIN_8:   u32 = 0x0000_0100; // GPIO pin 8
pub const GPIO_PIN_9:   u32 = 0x0000_0200; // GPIO pin 9
pub const GPIO_PIN_10:  u32 = 0x0000_0400; // GPIO pin 10
pub const GPIO_PIN_11:  u32 = 0x0000_0800; // GPIO pin 11
pub const GPIO_PIN_12:  u32 = 0x0000_1000; // GPIO pin 12
pub const GPIO_PIN_13:  u32 = 0x0000_2000; // GPIO pin 13
pub const GPIO_PIN_14:  u32 = 0x0000_4000; // GPIO pin 14
pub const GPIO_PIN_15:  u32 = 0x0000_8000; // GPIO pin 15
pub const GPIO_PIN_ALL: u32 = 0x0000_FFFF; // All GPIO pins



#[repr(C)]
pub struct Gpio {
//    pub port: [RW<u32>; 8],
//    reserved0: [u32; 24],
    pub gpio_ctl:     RW<u32>, //GPIO port control register
    pub gpio_omode:   RW<u32>, //GPIO port output mode register
    pub gpio_ospd:    RW<u32>, //GPIO port output speed register
    pub gpio_pud:     RW<u32>, //GPIO port pull-up/pull-down register
    pub gpio_istat:   RW<u32>, //GPIO port input status register
    pub gpio_octl:    RW<u32>, //GPIO port output control register
    pub gpio_bop:     RW<u32>, //GPIO port bit operation register
    pub gpio_lock:    RW<u32>, //GPIO port configuration lock register
    pub gpio_afsel0:  RW<u32>, //GPIO alternate function selected register 0
    pub gpio_afsel1:  RW<u32>, //GPIO alternate function selected register 1
    pub gpio_bc:      RW<u32>, //GPIO bit clear register
    pub gpio_tg:      RW<u32>, //GPIO port bit toggle register
}




pub fn gpio_mode_set(gpio_periph: u32, mode: u32, pull_up_down: u32, pin: u32) {
    let gpio_a = gpio_periph as *const Gpio;
    let mut ctl: u32;
    let mut pupd: u32;

    unsafe { 
        ctl = (*gpio_a).gpio_ctl.read();
        pupd = (*gpio_a).gpio_pud.read();
    }
        /* clear the specified pin mode bits */
        ctl &= !GPIO_MODE_MASK!(pin);
        ctl |= GPIO_MODE_SET!(pin, mode);

        pupd &= !GPIO_PUPD_MASK!(pin);
        pupd |= GPIO_PUPD_SET!(pin, pull_up_down);

    unsafe { 
        (*gpio_a).gpio_ctl.write(ctl);
        (*gpio_a).gpio_pud.write(pupd);
    }
}

/**!
    \brief      set GPIO pin bit
    \param[in]  gpio_periph: GPIOx(x = A,B,C,F) 
                only one parameter can be selected which is shown as below:
      \arg        GPIOx(x = A,B,C,F) 
    \param[in]  pin: GPIO pin
                one or more parameters can be selected which are shown as below:
      \arg        GPIO_PIN_x(x=0..15), GPIO_PIN_ALL
    \param[out] none
    \retval     none
**/
pub fn gpio_bit_set(gpio_periph: u32, pin: u32) {
    //    GPIO_BOP(gpio_periph) = (uint32_t)pin;
/*
    let gpio = gpio_periph as *const Gpio;

    unsafe { 
        (*gpio).gpio_bop[0].write(pin);
    }
*/

    let reg = GPIO_BOP!(gpio_periph) as *mut RW<u32>;
    unsafe { 
        (*reg).write(pin);
    }
}

/**!
    \brief      reset GPIO pin bit
    \param[in]  gpio_periph: GPIOx(x = A,B,C,F) 
                only one parameter can be selected which is shown as below:
      \arg        GPIOx(x = A,B,C,F) 
    \param[in]  pin: GPIO pin
                one or more parameters can be selected which are shown as below:
      \arg        GPIO_PIN_x(x=0..15), GPIO_PIN_ALL
    \param[out] none
    \retval     none
**/
pub fn gpio_bit_reset(gpio_periph: u32, pin: u32) {
//    GPIO_BC(gpio_periph) = (uint32_t)pin;
    let gpio = gpio_periph as *const Gpio;

    unsafe { 
        (*gpio).gpio_bc.write(pin);
    }
}


/**!
    \brief      toggle GPIO pin status
    \param[in]  gpio_periph: GPIOx(x = A,B,C,F) 
                only one parameter can be selected which is shown as below:
      \arg        GPIOx(x = A,B,C,F) 
    \param[in]  pin: GPIO pin
                one or more parameters can be selected which are shown as below:
      \arg        GPIO_PIN_x(x=0..15), GPIO_PIN_ALL
    \param[out] none
    \retval     none
**/
pub fn gpio_bit_toggle(gpio_periph: u32, pin: u32) {
//    GPIO_TG(gpio_periph) = (uint32_t)pin;
    let gpio = gpio_periph as *const Gpio;

    unsafe { 
        (*gpio).gpio_tg.write(pin);
    }
}


/**!
    \brief      set GPIO output type and speed
    \param[in]  gpio_periph: GPIOx(x = A,B,C,F) 
                only one parameter can be selected which is shown as below:
      \arg        GPIOx(x = A,B,C,F) 
    \param[in]  otype: gpio pin output mode
      \arg        GPIO_OTYPE_PP: push pull mode
      \arg        GPIO_OTYPE_OD: open drain mode
    \param[in]  speed: gpio pin output max speed
      \arg        GPIO_OSPEED_2MHZ: output max speed 2MHz 
      \arg        GPIO_OSPEED_10MHZ: output max speed 10MHz 
      \arg        GPIO_OSPEED_50MHZ: output max speed 50MHz
    \param[in]  pin: GPIO pin
                one or more parameters can be selected which are shown as below:
      \arg        GPIO_PIN_x(x=0..15), GPIO_PIN_ALL
    \param[out] none
    \retval     none
**/
pub fn gpio_output_options_set(gpio_periph: u32, otype: u8, speed: u32, pin: u32) {
    let i = 0 as u16;
    let mut ospeed = 0 as u32;
    let mut pin_val: u32;
    let gpio = gpio_periph as *const Gpio;

    unsafe { 
        pin_val = (*gpio).gpio_omode.read();
    }

    if(GPIO_OTYPE_OD == otype){
//        GPIO_OMODE(gpio_periph) |= pin;
        pin_val |= pin;
    }else{
    //        GPIO_OMODE(gpio_periph) &= (!pin);
        pin_val &= !pin;
    }

    unsafe { 
        (*gpio).gpio_omode.write(pin_val);
    }

    /* get the specified pin output speed bits value */
    ospeed = GPIO_OSPD!(gpio_periph);

    /* clear the specified pin output speed bits */
    ospeed &= !GPIO_OSPEED_MASK!(i);

    /* set the specified pin output speed bits */
    ospeed |= GPIO_OSPEED_SET!(i,speed);

    unsafe { 
        (*gpio).gpio_ospd.write(ospeed);
    }
}
