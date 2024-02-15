
const APB1_BUS_BASE: u32 = 0x40000000;
const AHB2_BUS_BASE: usize = 0x4800_0000;

const RCU_BASE: usize = AHB1_BUS_BASE + 0x0000_1000;
const GPIO_BASE: usize = AHB2_BUS_BASE + 0x0000_0000;


/* bit operations */
//#define REG32(addr)                  (*(volatile uint32_t *)(uint32_t)(addr))
//#define REG16(addr)                  (*(volatile uint16_t *)(uint32_t)(addr))
//#define REG8(addr)                   (*(volatile uint8_t *)(uint32_t)(addr))
//#define BIT(x)                       ((uint32_t)((uint32_t)0x01U<<(x)))
macro_rules! BIT     { ($x:expr) => (((0x0000_0001 as u32) << x) as u32) }
//#define BITS(start, end)             (((0xFFFF_FFFF as u32) << (start)) & (0xFFFFFFFFUL >> (31U - (uint32_t)(end))))
macro_rules! BITS     { ($start:expr,$end:expr) => (((0xFFFF_FFFF as u32) << ($start as u32)) & ((0xFFFF_FFFF as u32) >> (31U - ($end as u32)))) }
//#define GET_BITS(regval, start, end) (((regval) & BITS((start),(end))) >> (start))



macro_rules! RCU_BIT_POS     { ($val:expr) => ((($val as u32) & 0x1F) as u32) }
/* define the voltage key unlock value */
const RCU_VKEY_UNLOCK: u32 = 0x1A2B_3C4D;
