#![no_main]
#![no_std]

extern crate alloc;
use alloc::vec;

use stm32h7xx_hal::{ethernet, pac, prelude::*, spi, stm32};
use stm32h755zi as _;

use smoltcp::{
    iface::{Config, Interface, SocketSet},
    socket::tcp,
    time::Instant,
    wire::{EthernetAddress, IpAddress, IpCidr},
};

use embedded_alloc::LlffHeap as EmbeddedAllocator;

// Locally administered MAC address
const MAC_ADDRESS: [u8; 6] = [0x02, 0x00, 0x11, 0x22, 0x33, 0x44];

const ARDUCHIP_TEST1: u8 = 0x00;
const ARDUCHIP_FIFO: u8 = 0x04;
const ARDUCHIP_TRIG: u8 = 0x41;
const ARDUCHIP_FIFO_SIZE1: u8 = 0x42;
const ARDUCHIP_FIFO_SIZE2: u8 = 0x43;
const ARDUCHIP_FIFO_SIZE3: u8 = 0x44;
const ARDUCHIP_BURST_FIFO_READ: u8 = 0x3C;

const SENSOR_ADDRESS: u8 = 0x30;

const OV2640_CHIPID_HIGH: u8 = 0x0A;
const OV2640_CHIPID_LOW: u8 = 0x0B;

const OV2640_QVGA: [[u8; 2]; 193] = [
    [0xff, 0x0],
    [0x2c, 0xff],
    [0x2e, 0xdf],
    [0xff, 0x1],
    [0x3c, 0x32],
    [0x11, 0x0],
    [0x9, 0x2],
    [0x4, 0xa8],
    [0x13, 0xe5],
    [0x14, 0x48],
    [0x2c, 0xc],
    [0x33, 0x78],
    [0x3a, 0x33],
    [0x3b, 0xfb],
    [0x3e, 0x0],
    [0x43, 0x11],
    [0x16, 0x10],
    [0x39, 0x2],
    [0x35, 0x88],
    [0x22, 0xa],
    [0x37, 0x40],
    [0x23, 0x0],
    [0x34, 0xa0],
    [0x6, 0x2],
    [0x6, 0x88],
    [0x7, 0xc0],
    [0xd, 0xb7],
    [0xe, 0x1],
    [0x4c, 0x0],
    [0x4a, 0x81],
    [0x21, 0x99],
    [0x24, 0x40],
    [0x25, 0x38],
    [0x26, 0x82],
    [0x5c, 0x0],
    [0x63, 0x0],
    [0x46, 0x22],
    [0xc, 0x3a],
    [0x5d, 0x55],
    [0x5e, 0x7d],
    [0x5f, 0x7d],
    [0x60, 0x55],
    [0x61, 0x70],
    [0x62, 0x80],
    [0x7c, 0x5],
    [0x20, 0x80],
    [0x28, 0x30],
    [0x6c, 0x0],
    [0x6d, 0x80],
    [0x6e, 0x0],
    [0x70, 0x2],
    [0x71, 0x94],
    [0x73, 0xc1],
    [0x3d, 0x34],
    [0x12, 0x4],
    [0x5a, 0x57],
    [0x4f, 0xbb],
    [0x50, 0x9c],
    [0xff, 0x0],
    [0xe5, 0x7f],
    [0xf9, 0xc0],
    [0x41, 0x24],
    [0xe0, 0x14],
    [0x76, 0xff],
    [0x33, 0xa0],
    [0x42, 0x20],
    [0x43, 0x18],
    [0x4c, 0x0],
    [0x87, 0xd0],
    [0x88, 0x3f],
    [0xd7, 0x3],
    [0xd9, 0x10],
    [0xd3, 0x82],
    [0xc8, 0x8],
    [0xc9, 0x80],
    [0x7c, 0x0],
    [0x7d, 0x0],
    [0x7c, 0x3],
    [0x7d, 0x48],
    [0x7d, 0x48],
    [0x7c, 0x8],
    [0x7d, 0x20],
    [0x7d, 0x10],
    [0x7d, 0xe],
    [0x90, 0x0],
    [0x91, 0xe],
    [0x91, 0x1a],
    [0x91, 0x31],
    [0x91, 0x5a],
    [0x91, 0x69],
    [0x91, 0x75],
    [0x91, 0x7e],
    [0x91, 0x88],
    [0x91, 0x8f],
    [0x91, 0x96],
    [0x91, 0xa3],
    [0x91, 0xaf],
    [0x91, 0xc4],
    [0x91, 0xd7],
    [0x91, 0xe8],
    [0x91, 0x20],
    [0x92, 0x0],
    [0x93, 0x6],
    [0x93, 0xe3],
    [0x93, 0x3],
    [0x93, 0x3],
    [0x93, 0x0],
    [0x93, 0x2],
    [0x93, 0x0],
    [0x93, 0x0],
    [0x93, 0x0],
    [0x93, 0x0],
    [0x93, 0x0],
    [0x93, 0x0],
    [0x93, 0x0],
    [0x96, 0x0],
    [0x97, 0x8],
    [0x97, 0x19],
    [0x97, 0x2],
    [0x97, 0xc],
    [0x97, 0x24],
    [0x97, 0x30],
    [0x97, 0x28],
    [0x97, 0x26],
    [0x97, 0x2],
    [0x97, 0x98],
    [0x97, 0x80],
    [0x97, 0x0],
    [0x97, 0x0],
    [0xa4, 0x0],
    [0xa8, 0x0],
    [0xc5, 0x11],
    [0xc6, 0x51],
    [0xbf, 0x80],
    [0xc7, 0x10],
    [0xb6, 0x66],
    [0xb8, 0xa5],
    [0xb7, 0x64],
    [0xb9, 0x7c],
    [0xb3, 0xaf],
    [0xb4, 0x97],
    [0xb5, 0xff],
    [0xb0, 0xc5],
    [0xb1, 0x94],
    [0xb2, 0xf],
    [0xc4, 0x5c],
    [0xa6, 0x0],
    [0xa7, 0x20],
    [0xa7, 0xd8],
    [0xa7, 0x1b],
    [0xa7, 0x31],
    [0xa7, 0x0],
    [0xa7, 0x18],
    [0xa7, 0x20],
    [0xa7, 0xd8],
    [0xa7, 0x19],
    [0xa7, 0x31],
    [0xa7, 0x0],
    [0xa7, 0x18],
    [0xa7, 0x20],
    [0xa7, 0xd8],
    [0xa7, 0x19],
    [0xa7, 0x31],
    [0xa7, 0x0],
    [0xa7, 0x18],
    [0x7f, 0x0],
    [0xe5, 0x1f],
    [0xe1, 0x77],
    [0xdd, 0x7f],
    [0xc2, 0xe],
    [0xff, 0x0],
    [0xe0, 0x4],
    [0xc0, 0xc8],
    [0xc1, 0x96],
    [0x86, 0x3d],
    [0x51, 0x90],
    [0x52, 0x2c],
    [0x53, 0x0],
    [0x54, 0x0],
    [0x55, 0x88],
    [0x57, 0x0],
    [0x50, 0x92],
    [0x5a, 0x50],
    [0x5b, 0x3c],
    [0x5c, 0x0],
    [0xd3, 0x4],
    [0xe0, 0x0],
    [0xff, 0x0],
    [0x5, 0x0],
    [0xda, 0x8],
    [0xd7, 0x3],
    [0xe0, 0x0],
    [0x5, 0x0],
];

const BMP_HEADER: [u8; 66] = [
    // BMP header : 14 bytes
    0x42, 0x4D, // BM
    0x42, 0x58, 0x02, 0x00, // Size : 153666 bytes
    0x00, 0x00, // Application
    0x00, 0x00, // Application
    0x42, 0x00, 0x00, 0x00, // Offset : 66 bytes
    // DIB Header : 52 bytes
    0x28, 0x00, 0x00, 0x00, // DIB Header size : 40 bytes
    0x40, 0x01, 0x00, 0x00, // Image width : 320 px
    0xF0, 0x00, 0x00, 0x00, // Image height : 240 px
    0x01, 0x00, // Color plane : 1
    0x10, 0x00, // Bits per pixel : 16 bits
    0x03, 0x00, 0x00, 0x00, // Compression method : 3 (BITFIELDS)
    0x00, 0x58, 0x02, 0x00, // Raw Bitmap size : 153600 bytes
    0xC4, 0x0E, 0x00, 0x00, // Horizontal resolution : 3780 px/m
    0xC4, 0x0E, 0x00, 0x00, // Vertical resolution : 3780 px/m
    0x00, 0x00, 0x00, 0x00, // Colors in palette : 0 (256)
    0x00, 0x00, 0x00, 0x00, // Importants colors : 0
    // Bit masks
    0x00, 0xF8, 0x00, 0x00, // Red mask :   1111 1000  0000 0000
    0xE0, 0x07, 0x00, 0x00, // Green mask : 0000 0111  1110 0000
    0x1F, 0x00, 0x00, 0x00, // Blue mask :  0000 0000  0001 1111
];

macro_rules! i2c_read {
    ($i2c:ident, $reg:expr) => {{
        let mut value = [0u8];
        $i2c.write_read(SENSOR_ADDRESS, &[$reg], &mut value)
            .expect("I2C read");
        value[0]
    }};
}

macro_rules! i2c_write {
    ($i2c:ident, $reg:expr, $val:expr) => {
        $i2c.write(SENSOR_ADDRESS, &[$reg, $val])
            .expect("I2C write");
    };
}

macro_rules! spi_read {
    ($spi:ident, $cs:ident, $reg:expr) => {{
        $cs.set_low();
        $spi.write(&[$reg]).expect("SPI write");
        let value = $spi.transfer(&mut [0u8]).expect("SPI read")[0];
        $cs.set_high();
        value
    }};
}

macro_rules! spi_write {
    ($spi:ident, $cs:ident, $reg:expr, $val:expr) => {
        $cs.set_low();
        $spi.write(&[$reg | 0x80, $val]).expect("SPI write");
        $cs.set_high();
    };
}

macro_rules! clear_fifo_flag {
    ($spi:ident, $cs:ident) => {
        spi_write!($spi, $cs, ARDUCHIP_FIFO, 0x01)
    };
}

macro_rules! start_capture {
    ($spi:ident, $cs:ident) => {
        spi_write!($spi, $cs, ARDUCHIP_FIFO, 0x02)
    };
}

macro_rules! capture_done {
    ($spi:ident, $cs:ident) => {{ spi_read!($spi, $cs, ARDUCHIP_TRIG) & 0x08 != 0 }};
}

macro_rules! read_fifo_flag {
    ($spi:ident, $cs:ident) => {{
        ((spi_read!($spi, $cs, ARDUCHIP_FIFO_SIZE3) as u32) << 16
            | (spi_read!($spi, $cs, ARDUCHIP_FIFO_SIZE2) as u32) << 8
            | (spi_read!($spi, $cs, ARDUCHIP_FIFO_SIZE1) as u32))
            & 0x7fffff
    }};
}

macro_rules! burst_fifo_read {
    ($spi:ident, $cs:ident, $length:expr) => {{
        let mut v = [0u8; 153666];
        for i in 0..BMP_HEADER.len() {
            v[i] = BMP_HEADER[i];
        }
        $cs.set_low();
        $spi.write(&[ARDUCHIP_BURST_FIFO_READ]).expect("SPI write");
        let mut i = BMP_HEADER.len();
        for _ in 0..153600 / 2 {
            let vh = $spi.transfer(&mut [0u8]).expect("SPI read")[0];
            v[i] = $spi.transfer(&mut [0u8]).expect("SPI read")[0];
            v[i + 1] = vh;
            i += 2;
        }
        $cs.set_high();
        v
    }};
}


fn rgb565_to_gray(pixel: u16) -> u8 {
    let r = (pixel >> 11) & 0x1F;
    let g = (pixel >> 5) & 0x3F;
    let b = pixel & 0x1F;

    // Mise à l’échelle intégrée
    let y =
        77 * r * 255 / 31 +
            150 * g * 255 / 63 +
            29 * b * 255 / 31;

    (y >> 8) as u8
}

pub fn rgb565_buffer_to_gray(
    src: &[u8],
    dst: &mut [u8],
) {
    let mut j = 0;

    for i in (0..src.len()).step_by(2) {
        let pixel = u16::from_be_bytes([src[i], src[i + 1]]);
        dst[j] = rgb565_to_gray(pixel);
        j += 1;
    }
}


#[global_allocator]
static ALLOCATOR: EmbeddedAllocator = EmbeddedAllocator::empty();

// the program entry point
#[cortex_m_rt::entry]
fn main() -> ! {
    {
        use core::mem::MaybeUninit;
        const ALLOCATION_BUFFER_SIZE: usize = 256 * 1024;
        static mut ALLOCATION_BUFFER: [MaybeUninit<u8>; ALLOCATION_BUFFER_SIZE] =
            [MaybeUninit::uninit(); ALLOCATION_BUFFER_SIZE];
        unsafe {
            ALLOCATOR.init(&raw mut ALLOCATION_BUFFER as usize, ALLOCATION_BUFFER_SIZE);
        }
    }

    let dp = stm32::Peripherals::take().unwrap();
    let cp = stm32::CorePeripherals::take().unwrap();

    // Power
    let pwrcfg = dp.PWR.constrain().freeze();

    // Clocks...
    let rcc = dp.RCC.constrain();
    let ccdr = rcc
        .sys_ck(200.MHz())
        .hclk(200.MHz())
        .pll1_r_ck(100.MHz()) // for TRACECK
        .pll1_q_ck(48.MHz())
        .freeze(pwrcfg, &dp.SYSCFG);

    // Get the delay provider.
    let mut delay = cp.SYST.delay(ccdr.clocks);

    // Initialise IO...
    let gpioa = dp.GPIOA.split(ccdr.peripheral.GPIOA);
    let gpiob = dp.GPIOB.split(ccdr.peripheral.GPIOB);
    let gpioc = dp.GPIOC.split(ccdr.peripheral.GPIOC);
    let gpiod = dp.GPIOD.split(ccdr.peripheral.GPIOD);
    let gpiog = dp.GPIOG.split(ccdr.peripheral.GPIOG);

    let rmii_ref_clk = gpioa.pa1.into_alternate();
    let rmii_mdio = gpioa.pa2.into_alternate();
    let rmii_mdc = gpioc.pc1.into_alternate();
    let rmii_crs_dv = gpioa.pa7.into_alternate();
    let rmii_rxd0 = gpioc.pc4.into_alternate();
    let rmii_rxd1 = gpioc.pc5.into_alternate();
    let rmii_tx_en = gpiog.pg11.into_alternate();
    let rmii_txd0 = gpiog.pg13.into_alternate();
    let rmii_txd1 = gpiob.pb13.into_alternate();

    let sck = gpioa.pa5.into_alternate();
    let miso = gpioa.pa6.into_alternate();
    let mosi = gpiob.pb5.into_alternate();
    let mut cs = gpiod.pd15.into_push_pull_output();
    cs.set_high();

    let scl = gpiob.pb6.into_alternate_open_drain();
    let sda = gpiob.pb7.into_alternate_open_drain();

    // Initialise ethernet...
    assert_eq!(ccdr.clocks.hclk().raw(), 200_000_000); // HCLK 200MHz
    assert_eq!(ccdr.clocks.pclk1().raw(), 100_000_000); // PCLK 100MHz
    assert_eq!(ccdr.clocks.pclk2().raw(), 100_000_000); // PCLK 100MHz
    assert_eq!(ccdr.clocks.pclk4().raw(), 100_000_000); // PCLK 100MHz

    let mac_addr = smoltcp::wire::EthernetAddress::from_bytes(&MAC_ADDRESS);
    let (mut eth_dma, _eth_mac) = unsafe {
        static mut DES_RING: ethernet::DesRing<4, 4> = ethernet::DesRing::new();

        ethernet::new(
            dp.ETHERNET_MAC,
            dp.ETHERNET_MTL,
            dp.ETHERNET_DMA,
            (
                rmii_ref_clk,
                rmii_mdio,
                rmii_mdc,
                rmii_crs_dv,
                rmii_rxd0,
                rmii_rxd1,
                rmii_tx_en,
                rmii_txd0,
                rmii_txd1,
            ),
            #[allow(static_mut_refs)]
            &mut DES_RING,
            mac_addr,
            ccdr.peripheral.ETH1MAC,
            &ccdr.clocks,
        )
    };

    let mut iface = Interface::new(
        Config::new(EthernetAddress::from_bytes(&MAC_ADDRESS).into()),
        &mut eth_dma,
        Instant::from_micros(cortex_m::peripheral::DWT::cycle_count() / 200),
    );
    iface.update_ip_addrs(|ip_addrs| {
        ip_addrs
            .push(IpCidr::new(IpAddress::v4(192, 168, 122, 100), 24))
            .unwrap();
    });

    let tcp_rx_buffer = tcp::SocketBuffer::new(vec![0; 65535 / 10]);
    let tcp_tx_buffer = tcp::SocketBuffer::new(vec![0; 3 * 65535]);
    let tcp_socket = tcp::Socket::new(tcp_rx_buffer, tcp_tx_buffer);

    let mut sockets = SocketSet::new(vec![]);
    let socket_handle = sockets.add(tcp_socket);

    let mut spi: spi::Spi<pac::SPI1, _, u8> = dp.SPI1.spi(
        (sck, miso, mosi),
        spi::MODE_0,
        3.MHz(),
        ccdr.peripheral.SPI1,
        &ccdr.clocks,
    );
    let mut i2c = dp
        .I2C1
        .i2c((scl, sda), 100.kHz(), ccdr.peripheral.I2C1, &ccdr.clocks);

    // ArduCAM reset
    spi_write!(spi, cs, 0x07, 0x80);
    delay.delay_ms(100_u16);
    spi_write!(spi, cs, 0x07, 0x00);
    delay.delay_ms(100_u16);

    // SPI Test
    spi_write!(spi, cs, ARDUCHIP_TEST1, 0x55);
    //defmt::println!("SPI_TEST = 0x{=u8:X}", spi_read!(spi, cs, ARDUCHIP_TEST1));

    // I2C Test
    i2c_write!(i2c, 0xFF, 0x01);
    //defmt::println!("VID = 0x{=u8:X}", i2c_read!(i2c, OV2640_CHIPID_HIGH));
    //defmt::println!("PID = 0x{=u8:X}", i2c_read!(i2c, OV2640_CHIPID_LOW));

    // Camera Init
    i2c_write!(i2c, 0xFF, 0x01);
    i2c_write!(i2c, 0x12, 0x80);
    delay.delay_ms(100_u16);

    // Camera config
    for [reg, val] in OV2640_QVGA {
        i2c_write!(i2c, reg, val);
    }
    delay.delay_ms(1000_u16);

    defmt::println!("BEGIN LOOP");
    loop {
        let timestamp = Instant::from_micros(cortex_m::peripheral::DWT::cycle_count() / 200);
        iface.poll(timestamp, &mut eth_dma, &mut sockets);

        let socket = sockets.get_mut::<tcp::Socket>(socket_handle);

        if !socket.is_open() {
            defmt::println!("Socket OPEN");
            socket.listen(80).unwrap();
        }

        if socket.may_recv() {
            let data_received = socket
                .recv(|buffer| {
                    if !buffer.is_empty() {
                        defmt::println!("{=str}", str::from_utf8(buffer).unwrap());
                        (buffer.len(), true)
                    } else {
                        (0, false)
                    }
                })
                .unwrap();
            if socket.can_send() && data_received {
                // Take photo
                clear_fifo_flag!(spi, cs);
                start_capture!(spi, cs);

                while !capture_done!(spi, cs) {
                    delay.delay_ms(10_u16);
                }
                delay.delay_ms(50_u16);

                let length = read_fifo_flag!(spi, cs);

                if length >= 153600 {
                    defmt::println!("RESPONSE");
                    socket
                        .send_slice("HTTP/1.1 200\nContent-Type: image/png\n\n".as_bytes())
                        .unwrap();
                    socket.send_slice(&BMP_HEADER).unwrap();
                    let mut finished = false;
                    cs.set_low();
                    spi.write(&[ARDUCHIP_BURST_FIFO_READ]).expect("SPI write");
                    let mut send_length = 0;
                    while send_length < 76800 {
                        send_length += socket
                            .send(|buffer| {
                                let mut written_length = 0;
                                for i in 0..buffer.len() {
                                    if send_length + written_length == 76800 {
                                        break;
                                    }
                                    let color: u16 = ((spi.transfer(&mut [0u8]).expect("SPI read")[0] as u16) << 8_u16) | spi.transfer(&mut [0u8]).expect("SPI read")[0] as u16;
                                    buffer[i] = rgb565_to_gray(color);
                                    written_length += 2;
                                    defmt::println!("{=u8:02X}", buffer[i + 1]);
                                    defmt::println!("{=u8:02X}", buffer[i]);
                                }
                                (written_length, written_length)
                            })
                            .unwrap();
                    }
                    defmt::println!("Socket CLOSE");
                    socket.close();
                }
            }
        } else if socket.may_send() {
            defmt::println!("Socket CLOSE");
            socket.close();
        }
    }
}
