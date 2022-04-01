//! Tock kernel for the SMA Q3 smartwatch.
//!
//! It is based on nRF52840 SoC (Cortex M4 core with a BLE transceiver) with
//! SWD as I/O and many peripherals.

#![no_std]
// Disable this attribute when documenting, as a workaround for
// https://github.com/rust-lang/rust/issues/62184.
#![cfg_attr(not(doc), no_main)]
#![deny(missing_docs)]

use capsules::bmp280::Bmp280;
use capsules::mx25r6435f;
use capsules::virtual_aes_ccm::MuxAES128CCM;
use capsules::virtual_alarm::VirtualMuxAlarm;
use components::bmp280_component_helper;
use components::bmp280::Bmp280Component;
use kernel::component::Component;
use kernel::dynamic_deferred_call::{DynamicDeferredCall, DynamicDeferredCallClientState};
use kernel::hil::sensors::{TemperatureClient, TemperatureDriver};
use kernel::hil::i2c::I2CMaster;
use kernel::hil::led::LedHigh;
use kernel::hil::symmetric_encryption::AES128;
use kernel::hil::time::Counter;
use kernel::platform::{KernelResources, SyscallDriverLookup};
use kernel::scheduler::round_robin::RoundRobinSched;
#[allow(unused_imports)]
use kernel::{capabilities, create_capability, debug, debug_gpio, debug_verbose, static_init};
use nrf52840::gpio::Pin;
use nrf52840::interrupt_service::Nrf52840DefaultPeripherals;
use nrf52_components::{self, UartChannel};

mod gnss;
mod periodic;
mod util;

// The backlight LED
const LED1_PIN: Pin = Pin::P0_08;

// Vibration motor
const VIBRA1_PIN: Pin = Pin::P0_19;

// The side button
const BUTTON_PIN: Pin = Pin::P0_17;

/// I2C pins for the temp/pressure sensor
const I2C_TEMP_SDA_PIN: Pin = Pin::P1_15;
const I2C_TEMP_SCL_PIN: Pin = Pin::P0_02;

// Constants related to the configuration of the 15.4 network stack
const SRC_MAC: u16 = 0xf00f;
const PAN_ID: u16 = 0xABCD;

/// UART Writer
pub mod io;

// State for loading and holding applications.
// How should the kernel respond when a process faults.
const FAULT_RESPONSE: kernel::process::PanicFaultPolicy = kernel::process::PanicFaultPolicy {};

// Number of concurrent processes this platform supports.
const NUM_PROCS: usize = 8;

static mut PROCESSES: [Option<&'static dyn kernel::process::Process>; NUM_PROCS] =
    [None; NUM_PROCS];

// Static reference to chip for panic dumps
static mut CHIP: Option<&'static nrf52840::chip::NRF52<Nrf52840DefaultPeripherals>> = None;
// Static reference to process printer for panic dumps
static mut PROCESS_PRINTER: Option<&'static kernel::process::ProcessPrinterText> = None;

/// Dummy buffer that causes the linker to reserve enough space for the stack.
#[no_mangle]
#[link_section = ".stack_buffer"]
pub static mut STACK_MEMORY: [u8; 0x1000] = [0; 0x1000];

/// Supported drivers by the platform
pub struct Platform {
    temperature: &'static capsules::temperature::TemperatureSensor<'static>,
    ble_radio: &'static capsules::ble_advertising_driver::BLE<
        'static,
        nrf52840::ble_radio::Radio<'static>,
        VirtualMuxAlarm<'static, nrf52840::rtc::Rtc<'static>>,
    >,
    block_storage: &'static capsules::block_storage_driver::BlockStorage<
        'static,
        mx25r6435f::MX25R6435F<
            'static,
            capsules::virtual_spi::VirtualSpiMasterDevice<'static, nrf52840::spi::SPIM>,
            nrf52840::gpio::GPIOPin<'static>,
            VirtualMuxAlarm<'static, nrf52840::rtc::Rtc<'static>>
        >,
        { mx25r6435f::PAGE_SIZE },
        4096,
    >,
    ieee802154_radio: &'static capsules::ieee802154::RadioDriver<'static>,
    button: &'static capsules::button::Button<'static, nrf52840::gpio::GPIOPin<'static>>,
    gnss: &'static capsules::console::Console<'static>,
    pconsole: &'static capsules::process_console::ProcessConsole<
        'static,
        VirtualMuxAlarm<'static, nrf52840::rtc::Rtc<'static>>,
        components::process_console::Capability,
    >,
    console: &'static capsules::console::Console<'static>,
    gpio: &'static capsules::gpio::GPIO<'static, nrf52840::gpio::GPIOPin<'static>>,
    led: &'static capsules::led::LedDriver<
        'static,
        LedHigh<'static, nrf52840::gpio::GPIOPin<'static>>,
        2,
    >,
    rng: &'static capsules::rng::RngDriver<'static>,
    ipc: kernel::ipc::IPC<NUM_PROCS>,
    analog_comparator: &'static capsules::analog_comparator::AnalogComparator<
        'static,
        nrf52840::acomp::Comparator<'static>,
    >,
    alarm: &'static capsules::alarm::AlarmDriver<
        'static,
        capsules::virtual_alarm::VirtualMuxAlarm<'static, nrf52840::rtc::Rtc<'static>>,
    >,
    scheduler: &'static RoundRobinSched<'static>,
    systick: cortexm4::systick::SysTick,
}

impl SyscallDriverLookup for Platform {
    fn with_driver<F, R>(&self, driver_num: usize, f: F) -> R
    where
        F: FnOnce(Option<&dyn kernel::syscall::SyscallDriver>) -> R,
    {
        match driver_num {
            capsules::console::DRIVER_NUM => f(Some(self.console)),
            capsules::gpio::DRIVER_NUM => f(Some(self.gpio)),
            capsules::alarm::DRIVER_NUM => f(Some(self.alarm)),
            capsules::led::DRIVER_NUM => f(Some(self.led)),
            capsules::button::DRIVER_NUM => f(Some(self.button)),
            capsules::rng::DRIVER_NUM => f(Some(self.rng)),
            capsules::ble_advertising_driver::DRIVER_NUM => f(Some(self.ble_radio)),
            capsules::ieee802154::DRIVER_NUM => f(Some(self.ieee802154_radio)),
            capsules::temperature::DRIVER_NUM => f(Some(self.temperature)),
            capsules::analog_comparator::DRIVER_NUM => f(Some(self.analog_comparator)),
            capsules::block_storage_driver::DRIVER_NUM => f(Some(self.block_storage)),
            gnss::DRIVER_NUM => f(Some(self.gnss)),
            kernel::ipc::DRIVER_NUM => f(Some(&self.ipc)),
            _ => f(None),
        }
    }
}

impl KernelResources<nrf52840::chip::NRF52<'static, Nrf52840DefaultPeripherals<'static>>>
    for Platform
{
    type SyscallDriverLookup = Self;
    type SyscallFilter = ();
    type ProcessFault = ();
    type Scheduler = RoundRobinSched<'static>;
    type SchedulerTimer = cortexm4::systick::SysTick;
    type WatchDog = ();
    type ContextSwitchCallback = ();

    fn syscall_driver_lookup(&self) -> &Self::SyscallDriverLookup {
        &self
    }
    fn syscall_filter(&self) -> &Self::SyscallFilter {
        &()
    }
    fn process_fault(&self) -> &Self::ProcessFault {
        &()
    }
    fn scheduler(&self) -> &Self::Scheduler {
        self.scheduler
    }
    fn scheduler_timer(&self) -> &Self::SchedulerTimer {
        &self.systick
    }
    fn watchdog(&self) -> &Self::WatchDog {
        &()
    }
    fn context_switch_callback(&self) -> &Self::ContextSwitchCallback {
        &()
    }
}

/// This is in a separate, inline(never) function so that its stack frame is
/// removed when this function returns. Otherwise, the stack space used for
/// these static_inits is wasted.
#[inline(never)]
unsafe fn get_peripherals() -> &'static mut Nrf52840DefaultPeripherals<'static> {
    // Initialize chip peripheral drivers
    let nrf52840_peripherals = static_init!(
        Nrf52840DefaultPeripherals,
        Nrf52840DefaultPeripherals::new()
    );

    nrf52840_peripherals
}

/// Main function called after RAM initialized.
#[no_mangle]
pub unsafe fn main() {
    nrf52840::init();

    let nrf52840_peripherals = get_peripherals();

    // set up circular peripheral dependencies
    nrf52840_peripherals.init();
    let base_peripherals = &nrf52840_peripherals.nrf52;

    let board_kernel = static_init!(kernel::Kernel, kernel::Kernel::new(&PROCESSES));

    // GPIOs
    let gpio = components::gpio::GpioComponent::new(
        board_kernel,
        capsules::gpio::DRIVER_NUM,
        components::gpio_component_helper!(
            nrf52840::gpio::GPIOPin,
            0 => &nrf52840_peripherals.gpio_port[Pin::P0_29],
        ),
    )
    .finalize(components::gpio_component_buf!(nrf52840::gpio::GPIOPin));
    
    let button = components::button::ButtonComponent::new(
        board_kernel,
        capsules::button::DRIVER_NUM,
        components::button_component_helper!(
            nrf52840::gpio::GPIOPin,
            (
                &nrf52840_peripherals.gpio_port[BUTTON_PIN],
                kernel::hil::gpio::ActivationMode::ActiveLow,
                kernel::hil::gpio::FloatingState::PullUp
            )
        ),
    )
    .finalize(components::button_component_buf!(nrf52840::gpio::GPIOPin));

    let led = components::led::LedsComponent::new().finalize(components::led_component_helper!(
        LedHigh<'static, nrf52840::gpio::GPIOPin>,
        LedHigh::new(&nrf52840_peripherals.gpio_port[LED1_PIN]),
        LedHigh::new(&nrf52840_peripherals.gpio_port[VIBRA1_PIN]),
    ));

    let chip = static_init!(
        nrf52840::chip::NRF52<Nrf52840DefaultPeripherals>,
        nrf52840::chip::NRF52::new(nrf52840_peripherals)
    );
    CHIP = Some(chip);

    nrf52_components::startup::NrfStartupComponent::new(
        false,
        BUTTON_PIN,
        nrf52840::uicr::Regulator0Output::V3_0,
        &base_peripherals.nvmc,
    )
    .finalize(());

    // Create capabilities that the board needs to call certain protected kernel
    // functions.
    let process_management_capability =
        create_capability!(capabilities::ProcessManagementCapability);
    let main_loop_capability = create_capability!(capabilities::MainLoopCapability);
    let memory_allocation_capability = create_capability!(capabilities::MemoryAllocationCapability);

    let gpio_port = &nrf52840_peripherals.gpio_port;

    // Configure kernel debug gpios as early as possible
    kernel::debug::assign_gpios(
        Some(&gpio_port[LED1_PIN]),
        None,
        None,
    );

    let rtc = &base_peripherals.rtc;
    let _ = rtc.start();
    let mux_alarm = components::alarm::AlarmMuxComponent::new(rtc)
        .finalize(components::alarm_mux_component_helper!(nrf52840::rtc::Rtc));
    let alarm = components::alarm::AlarmDriverComponent::new(
        board_kernel,
        capsules::alarm::DRIVER_NUM,
        mux_alarm,
    )
    .finalize(components::alarm_component_helper!(nrf52840::rtc::Rtc));
    // Initialize early so any panic beyond this point can use the RTT memory object.
    let uart_channel = {
        let mut rtt_memory_refs =
            components::segger_rtt::SeggerRttMemoryComponent::new().finalize(());

        // XXX: This is inherently unsafe as it aliases the mutable reference to rtt_memory. This
        // aliases reference is only used inside a panic handler, which should be OK, but maybe we
        // should use a const reference to rtt_memory and leverage interior mutability instead.
        self::io::set_rtt_memory(&mut *rtt_memory_refs.get_rtt_memory_ptr());

        UartChannel::Rtt(rtt_memory_refs)
    };
    let channel = nrf52_components::UartChannelComponent::new(
        uart_channel,
        mux_alarm,
        &base_peripherals.uarte0,
    )
    .finalize(());

    let dynamic_deferred_call_clients =
        static_init!([DynamicDeferredCallClientState; 3], Default::default());
    let dynamic_deferred_caller = static_init!(
        DynamicDeferredCall,
        DynamicDeferredCall::new(dynamic_deferred_call_clients)
    );
    DynamicDeferredCall::set_global_instance(dynamic_deferred_caller);

    let process_printer =
        components::process_printer::ProcessPrinterTextComponent::new().finalize(());
    PROCESS_PRINTER = Some(process_printer);

    // Create a shared UART channel for the console and for kernel debug.
    let uart_mux =
        components::console::UartMuxComponent::new(channel, 115200, dynamic_deferred_caller)
            .finalize(());

    let pconsole = components::process_console::ProcessConsoleComponent::new(
        board_kernel,
        uart_mux,
        mux_alarm,
        process_printer,
    )
    .finalize(components::process_console_component_helper!(
        nrf52840::rtc::Rtc<'static>
    ));

    use capsules::virtual_uart::UartDevice;

    // Setup the console.
    let console_uart = static_init!(UartDevice, UartDevice::new(uart_mux, true));
    console_uart.setup();
    let console = components::console::ConsoleComponent::new(
        board_kernel,
        capsules::console::DRIVER_NUM,
        console_uart,
    )
    .finalize(components::console_component_helper!());
    // Create the debugger object that handles calls to `debug!()`.
    components::debug_writer::DebugWriterComponent::new(uart_mux).finalize(());

    let ble_radio = nrf52_components::BLEComponent::new(
        board_kernel,
        capsules::ble_advertising_driver::DRIVER_NUM,
        &base_peripherals.ble_radio,
        mux_alarm,
    )
    .finalize(());

    let aes_mux = static_init!(
        MuxAES128CCM<'static, nrf52840::aes::AesECB>,
        MuxAES128CCM::new(&base_peripherals.ecb, dynamic_deferred_caller)
    );
    base_peripherals.ecb.set_client(aes_mux);
    aes_mux.initialize_callback_handle(
        dynamic_deferred_caller.register(aes_mux).unwrap(), // Unwrap fail = no deferred call slot available for ccm mux
    );

    let (ieee802154_radio, _mux_mac) = components::ieee802154::Ieee802154Component::new(
        board_kernel,
        capsules::ieee802154::DRIVER_NUM,
        &base_peripherals.ieee802154_radio,
        aes_mux,
        PAN_ID,
        SRC_MAC,
        dynamic_deferred_caller,
    )
    .finalize(components::ieee802154_component_helper!(
        nrf52840::ieee802154_radio::Radio,
        nrf52840::aes::AesECB<'static>
    ));

    let temp = components::temperature::TemperatureComponent::new(
        board_kernel,
        capsules::temperature::DRIVER_NUM,
        &base_peripherals.temp,
    )
    .finalize(());

    let dynamic_deferred_call_clients =
        static_init!([DynamicDeferredCallClientState; 5], Default::default());
    let dynamic_deferred_caller = static_init!(
        DynamicDeferredCall,
        DynamicDeferredCall::new(dynamic_deferred_call_clients)
    );
    DynamicDeferredCall::set_global_instance(dynamic_deferred_caller);
    
    let sensors_i2c_bus = static_init!(
        capsules::virtual_i2c::MuxI2C<'static>,
        capsules::virtual_i2c::MuxI2C::new(&base_peripherals.twi1, None, dynamic_deferred_caller)
    );
    
    base_peripherals.twi1.configure(
        nrf52840::pinmux::Pinmux::new(I2C_TEMP_SCL_PIN as u32),
        nrf52840::pinmux::Pinmux::new(I2C_TEMP_SDA_PIN as u32),
    );
    base_peripherals.twi1.set_master_client(sensors_i2c_bus);
        
    let bmp280 = Bmp280Component::new(sensors_i2c_bus, mux_alarm).finalize(
        bmp280_component_helper!(nrf52840::rtc::Rtc<'static>),
    );

    let temperature = components::temperature::TemperatureComponent::new(
        board_kernel,
        capsules::temperature::DRIVER_NUM,
        bmp280,
    )
    .finalize(());
    
    use kernel::hil::block_storage::BlockStorage;

    let flash = {
        let mux_spi =
        components::spi::SpiMuxComponent::new(&base_peripherals.spim0, dynamic_deferred_caller)
            .finalize(components::spi_mux_component_helper!(nrf52840::spi::SPIM));
        
        base_peripherals.spim0.configure(
            nrf52840::pinmux::Pinmux::new(Pin::P0_15 as u32),
            nrf52840::pinmux::Pinmux::new(Pin::P0_13 as u32),
            nrf52840::pinmux::Pinmux::new(Pin::P0_16 as u32),
        );
        
        components::mx25r6435f::Mx25r6435fComponent::new(
            None,
            None,
            &nrf52840_peripherals.gpio_port[Pin::P0_14] as &dyn kernel::hil::gpio::Pin,
            mux_alarm,
            mux_spi,
        )
        .finalize(components::mx25r6435f_component_helper!(
            nrf52840::spi::SPIM,
            nrf52840::gpio::GPIOPin,
            nrf52840::rtc::Rtc,
        ))
    };
    
    use kernel::hil::block_storage::HasClient;

    let block_storage_driver
        = components::block_storage::BlockStorageComponent{
            board_kernel,
            driver_num: capsules::block_storage_driver::DRIVER_NUM,
            device: flash,
        }
        .finalize(components::block_storage_component_helper!(
            mx25r6435f::MX25R6435F<
                'static,
                capsules::virtual_spi::VirtualSpiMasterDevice<'static, nrf52840::spi::SPIM>,
                nrf52840::gpio::GPIOPin<'static>,
                VirtualMuxAlarm<'static, nrf52840::rtc::Rtc<'static>>
            >,
            { mx25r6435f::PAGE_SIZE },
            4096,
        ));

    let display = {
        use capsules::virtual_spi::VirtualSpiMasterDevice;

        let mux_spi
            = components::spi::SpiMuxComponent::new(
                &base_peripherals.spim2,
                dynamic_deferred_caller,
            )
            .finalize(components::spi_mux_component_helper!(
                nrf52840::spi::SPIM
            ));
            
        use kernel::hil::spi::SpiMaster;
        base_peripherals.spim2.set_rate(1_000_000);
        
        base_peripherals.spim2.configure(
            nrf52840::pinmux::Pinmux::new(Pin::P0_27 as u32),
            //nrf52840::pinmux::Pinmux::new_disabled(),
            // not used but let's check
            nrf52840::pinmux::Pinmux::new(Pin::P0_28 as u32),
            nrf52840::pinmux::Pinmux::new(Pin::P0_26 as u32),
        );

        use kernel::hil;
        struct Inverted<'a, P: hil::gpio::Pin>(&'a P);
        impl<'a, P: hil::gpio::Pin> hil::gpio::Configure for Inverted<'a, P> {
            fn configuration(&self) -> kernel::hil::gpio::Configuration {
                self.0.configuration()
            }
            fn make_output(&self) -> kernel::hil::gpio::Configuration {
                self.0.make_output()
            }
            fn disable_output(&self) -> kernel::hil::gpio::Configuration {
                self.0.disable_output()
            }
            fn make_input(&self) -> kernel::hil::gpio::Configuration {
                self.0.make_input()
            }
            fn disable_input(&self) -> kernel::hil::gpio::Configuration {
                self.0.disable_input()
            }
            fn deactivate_to_low_power(&self) {
                self.0.deactivate_to_low_power()
            }
            fn set_floating_state(&self, _: kernel::hil::gpio::FloatingState) {
                unimplemented!() // not sure what it looks like with inversion
            }
            fn floating_state(&self) -> kernel::hil::gpio::FloatingState {
                unimplemented!() // not sure what it looks like with inversion
            }
        }
        impl<'a, P: hil::gpio::Pin> hil::gpio::Output for Inverted<'a, P> {
            fn set(&self) {
                self.0.clear()
            }
            fn clear(&self) {
                self.0.set()
            }
            fn toggle(&self) -> bool {
                self.0.toggle()
            }
        }
        impl<'a, P: hil::gpio::Pin> hil::gpio::Input for Inverted<'a, P> {
            fn read(&self) -> bool {
                !self.0.read()
            }
        }
        
        let chip_select = static_init!(
            Inverted<'static, nrf52840::gpio::GPIOPin>,
            Inverted(&nrf52840_peripherals.gpio_port[Pin::P0_05]),
        );

        let spi_device = static_init!(
            VirtualSpiMasterDevice<'static, nrf52840::spi::SPIM>,
            VirtualSpiMasterDevice::new(
                mux_spi,
                chip_select,
            ),
        );
        spi_device.setup();

        let display
            = components::lpm013m126::Lpm013m126Component {
                spi: spi_device,
                disp: &nrf52840_peripherals.gpio_port[Pin::P0_07],
                extcomin: &nrf52840_peripherals.gpio_port[Pin::P0_06],
                alarm_mux: mux_alarm,
            }
            .finalize(
                components::lpm013m126_component_helper!(
                    nrf52840::rtc::Rtc<'static>,
                    nrf52840::gpio::GPIOPin,
                    VirtualSpiMasterDevice<'static, nrf52840::spi::SPIM>,
                ),
            );

        dbg!(display.initialize());
        display
    };
    debug!("Display started");
    use kernel::ErrorCode;
    
    struct D<S: 'static + kernel::hil::time::Alarm<'static>, P: 'static + kernel::hil::gpio::Pin, B: 'static + kernel::hil::spi::SpiMasterDevice>(
        &'static capsules::lpm013m126::Lpm013m126<'static, S, P, B>
    );
    
    impl<S: 'static + kernel::hil::time::Alarm<'static>, P: 'static + kernel::hil::gpio::Pin, B: 'static + kernel::hil::spi::SpiMasterDevice> kernel::hil::screen::ScreenClient for D<S, P, B> {
        fn screen_is_ready(&self) {
            debug!("Display ready");
            let mut b = unsafe { static_init!([u8; 100], [0x83; 100]) };
            dbg!(self.0.write(&mut b[..], 100));
        }
        fn command_complete(&self, res: Result<(), ErrorCode>) {
            debug!("Command complete");
        }
        fn write_complete(&self, _: &'static mut [u8], _: Result<(), ErrorCode>) {
            debug!("Write complete");
        }
    }
    use kernel::hil::screen::Screen;
    use capsules::virtual_spi::VirtualSpiMasterDevice;
    
    let cd = static_init!(
        D<
            VirtualMuxAlarm<'static, nrf52840::rtc::Rtc<'static>>,
                    nrf52840::gpio::GPIOPin,
                    VirtualSpiMasterDevice<'static, nrf52840::spi::SPIM>,
                    >,
        D(display)
    );
    
    display.set_client(Some(cd));
    
    let gnss = {
        use kernel::hil::uart;
        use kernel::hil::uart::Configure;
        use kernel::hil::uart::Receive;
        
        base_peripherals.uarte0.initialize(
            nrf52840::pinmux::Pinmux::new(Pin::P0_31 as u32),
            nrf52840::pinmux::Pinmux::new(Pin::P0_30 as u32),
            None,
            None,
        );
        base_peripherals.uarte0.configure(uart::Parameters {
            baud_rate: 9600,
            width: uart::Width::Eight,
            parity: uart::Parity::None,
            stop_bits: uart::StopBits::One,
            hw_flow_control: false,
        }).unwrap();
        static mut BUFFER: [u8; gnss::BUFFER_SIZE] = [0; gnss::BUFFER_SIZE];

        /*
        let gnss: &'static _ = kernel::static_init!(
            gnss::Gnss::<
                nrf52840::uart::Uarte,
            >,
            gnss::Gnss::new(&base_peripherals.uarte0, &mut BUFFER),
        );
        base_peripherals.uarte0.set_receive_client(gnss);
        //gnss.start_receive();
        */
        
        let gnss = components::console::ConsoleComponent::new(
            board_kernel,
            capsules::driver::NUM::GNSS as usize,
            &base_peripherals.uarte0,
        ).finalize(components::console_component_helper!());
        
        use kernel::hil::gpio::Configure as _;
        use kernel::hil::gpio::Output;
        let pin = &nrf52840_peripherals.gpio_port[Pin::P0_29];
        pin.make_output();
        pin.set();
        gnss
    };
    
    let rng = components::rng::RngComponent::new(
        board_kernel,
        capsules::rng::DRIVER_NUM,
        &base_peripherals.trng,
    )
    .finalize(());

    // Initialize AC using AIN5 (P0.29) as VIN+ and VIN- as AIN0 (P0.02)
    // These are hardcoded pin assignments specified in the driver
    let analog_comparator = components::analog_comparator::AcComponent::new(
        &base_peripherals.acomp,
        components::acomp_component_helper!(
            nrf52840::acomp::Channel,
            &nrf52840::acomp::CHANNEL_AC0
        ),
        board_kernel,
        capsules::analog_comparator::DRIVER_NUM,
    )
    .finalize(components::acomp_component_buf!(
        nrf52840::acomp::Comparator
    ));

    nrf52_components::NrfClockComponent::new(&base_peripherals.clock).finalize(());

    let scheduler = components::sched::round_robin::RoundRobinComponent::new(&PROCESSES)
        .finalize(components::rr_component_helper!(NUM_PROCS));

        
    let periodic_virtual_alarm = static_init!(
        capsules::virtual_alarm::VirtualMuxAlarm<'static, nrf52840::rtc::Rtc>,
        capsules::virtual_alarm::VirtualMuxAlarm::new(mux_alarm)
    );
    periodic_virtual_alarm.setup();
    use kernel::hil::time::Alarm;
    
    struct Print(&'static Bmp280<'static, VirtualMuxAlarm<'static, nrf52840::rtc::Rtc<'static>>>);
    /*impl periodic::Callable for Print {
        fn next(&mut self) {
            //debug!("read request: {:?}", self.0.read_temperature());
        }
    }*/
    
    struct TempCelsius;
    impl TemperatureClient for TempCelsius {
        fn callback(&self, temp: usize) {
            //debug!("Temp: {} °C", temp);
        }
    }

    bmp280.set_client(&TempCelsius);
    let periodic = static_init!(
        periodic::Periodic<'static, VirtualMuxAlarm<'static, nrf52840::rtc::Rtc<'static>>, Print>,
        periodic::Periodic::new(periodic_virtual_alarm, Print(bmp280)),
    );
    
    
    //periodic_virtual_alarm.set_alarm_client(periodic);
    periodic.arm();
    bmp280.begin_reset().unwrap();
    let platform = Platform {
        temperature,
        button,
        ble_radio,
        block_storage: block_storage_driver,
        ieee802154_radio,
        gnss,
        pconsole,
        console,
        led,
        gpio,
        rng,
        //temp,
        alarm,
        analog_comparator,
        ipc: kernel::ipc::IPC::new(
            board_kernel,
            kernel::ipc::DRIVER_NUM,
            &memory_allocation_capability,
        ),
        scheduler,
        systick: cortexm4::systick::SysTick::new_with_calibration(64000000),
    };

    let _ = platform.pconsole.start();
    debug!("Initialization complete. Entering main loop\r");
    debug!("{}", &nrf52840::ficr::FICR_INSTANCE);

    // When a process crashes during loading, it usually isn't caught by RTT.
    // This delays process loading so that it can be caught.
    struct Apps {
        board_kernel: &'static kernel::Kernel,
        chip: &'static nrf52840::chip::NRF52<'static, Nrf52840DefaultPeripherals<'static>>,
    }
    
    impl periodic::Callable for Apps{
        fn next(&mut self) -> bool {
            debug!("process load");
            let process_management_capability =
        create_capability!(capabilities::ProcessManagementCapability);
            unsafe {
            kernel::process::load_processes(
                self.board_kernel,
                self.chip,
                core::slice::from_raw_parts(
                    &_sapps as *const u8,
                    &_eapps as *const u8 as usize - &_sapps as *const u8 as usize,
                ),
                core::slice::from_raw_parts_mut(
                    &mut _sappmem as *mut u8,
                    &_eappmem as *const u8 as usize - &_sappmem as *const u8 as usize,
                ),
                &mut PROCESSES,
                &FAULT_RESPONSE,
                &process_management_capability,
            )
            .unwrap_or_else(|err| {
                debug!("Error loading processes!");
                debug!("{:?}", err);
            });
            }
            false
        }
    };
    
    let periodic = static_init!(
        periodic::Periodic<'static, VirtualMuxAlarm<'static, nrf52840::rtc::Rtc<'static>>, Apps>,
        periodic::Periodic::new(
            periodic_virtual_alarm, 
            Apps{
                board_kernel,
                chip,
            },
        ),
    );
    
    periodic_virtual_alarm.set_alarm_client(periodic);

    periodic.arm();
    /// These symbols are defined in the linker script.
    extern "C" {
        /// Beginning of the ROM region containing app images.
        static _sapps: u8;
        /// End of the ROM region containing app images.
        static _eapps: u8;
        /// Beginning of the RAM region for app memory.
        static mut _sappmem: u8;
        /// End of the RAM region for app memory.
        static _eappmem: u8;
    }

    board_kernel.kernel_loop(&platform, chip, Some(&platform.ipc), &main_loop_capability);
}
