//! Serial-in parallel-out shift register

use core::cell::RefCell;
use core::mem::{self, MaybeUninit};

use crate::hal::digital::v2::OutputPin;

trait ShiftRegisterInternal {
    fn update(&self, index: usize, command: bool) -> Result<(), ()>;
}

/// Output pin of the shift register
pub struct ShiftRegisterPin<'a> {
    shift_register: &'a dyn ShiftRegisterInternal,
    index: usize,
}

impl<'a> ShiftRegisterPin<'a> {
    fn new(shift_register: &'a dyn ShiftRegisterInternal, index: usize) -> Self {
        ShiftRegisterPin {
            shift_register,
            index,
        }
    }
}

impl OutputPin for ShiftRegisterPin<'_> {
    type Error = ();

    fn set_low(&mut self) -> Result<(), Self::Error> {
        self.shift_register.update(self.index, false)?;
        Ok(())
    }

    fn set_high(&mut self) -> Result<(), Self::Error> {
        self.shift_register.update(self.index, true)?;
        Ok(())
    }
}

macro_rules! shift_register_builder {
    ($name: ident, $size: expr) => {
        /// Serial-in parallel-out shift register
        pub struct $name<Pin1, Pin2, Pin3>
        where
            Pin1: OutputPin,
            Pin2: OutputPin,
            Pin3: OutputPin,
        {
            clock: RefCell<Pin1>,
            latch: RefCell<Pin2>,
            data: RefCell<Pin3>,
            output_state: RefCell<[bool; $size]>,
            inverted: bool,
        }

        impl<Pin1, Pin2, Pin3> ShiftRegisterInternal for $name<Pin1, Pin2, Pin3>
        where
            Pin1: OutputPin,
            Pin2: OutputPin,
            Pin3: OutputPin,
        {
            /// Sets the value of the shift register output at `index` to value `command`
            fn update(&self, index: usize, command: bool) -> Result<(), ()> {
                self.output_state.borrow_mut()[index] = command;
                let output_state = self.output_state.borrow();
                if self.inverted {
                    self.latch.borrow_mut().set_high().map_err(|_e| ())?;
                } else {
                    self.latch.borrow_mut().set_low().map_err(|_e| ())?;
                }

                for i in 1..=output_state.len() {
                    if output_state[output_state.len() - i] {
                        if self.inverted {
                            self.data.borrow_mut().set_low().map_err(|_e| ())?;
                        } else {
                            self.data.borrow_mut().set_high().map_err(|_e| ())?;
                        }
                    } else {
                        if self.inverted {
                            self.data.borrow_mut().set_high().map_err(|_e| ())?;
                        } else {
                            self.data.borrow_mut().set_low().map_err(|_e| ())?;
                        }
                    }
                    self.clock.borrow_mut().set_high().map_err(|_e| ())?;
                    self.clock.borrow_mut().set_low().map_err(|_e| ())?;
                }

                if self.inverted {
                    self.latch.borrow_mut().set_low().map_err(|_e| ())?;
                } else {
                    self.latch.borrow_mut().set_high().map_err(|_e| ())?;
                }
                Ok(())
            }
        }

        impl<Pin1, Pin2, Pin3> $name<Pin1, Pin2, Pin3>
        where
            Pin1: OutputPin,
            Pin2: OutputPin,
            Pin3: OutputPin,
        {
            /// Creates a new SIPO shift register from clock, latch, and data output pins
            pub fn new(clock: Pin1, latch: Pin2, data: Pin3) -> Self {
                $name {
                    clock: RefCell::new(clock),
                    latch: RefCell::new(latch),
                    data: RefCell::new(data),
                    output_state: RefCell::new([false; $size]),
                    inverted: false,
                }
            }

            /// Inverts the latch output pin. This depends on which shift register is used.
            pub fn inverted(mut self, state: bool) -> Self {
                self.inverted = state;
                self
            }

            /// Get embedded-hal output pins to control the shift register outputs
            pub fn decompose(&self) -> [ShiftRegisterPin; $size] {
                // Create an uninitialized array of `MaybeUninit`. The `assume_init` is
                // safe because the type we are claiming to have initialized here is a
                // bunch of `MaybeUninit`s, which do not require initialization.
                let mut pins: [MaybeUninit<ShiftRegisterPin>; $size] =
                    unsafe { MaybeUninit::uninit().assume_init() };

                // Dropping a `MaybeUninit` does nothing, so if there is a panic during this loop,
                // we have a memory leak, but there is no memory safety issue.
                for (index, elem) in pins.iter_mut().enumerate() {
                    elem.write(ShiftRegisterPin::new(self, index));
                }

                // Everything is initialized. Transmute the array to the
                // initialized type.
                unsafe { mem::transmute::<_, [ShiftRegisterPin; $size]>(pins) }
            }

            /// Consume the shift register and return the original clock, latch, and data output pins
            pub fn release(self) -> (Pin1, Pin2, Pin3) {
                let Self {
                    clock,
                    latch,
                    data,
                    output_state: _,
                    inverted: _,
                } = self;
                (clock.into_inner(), latch.into_inner(), data.into_inner())
            }
        }
    };
}

shift_register_builder!(ShiftRegister8, 8);
shift_register_builder!(ShiftRegister16, 16);
shift_register_builder!(ShiftRegister24, 24);
shift_register_builder!(ShiftRegister32, 32);
shift_register_builder!(ShiftRegister40, 40);
shift_register_builder!(ShiftRegister48, 48);
shift_register_builder!(ShiftRegister56, 56);
shift_register_builder!(ShiftRegister64, 64);
shift_register_builder!(ShiftRegister72, 72);
shift_register_builder!(ShiftRegister80, 80);
shift_register_builder!(ShiftRegister88, 88);
shift_register_builder!(ShiftRegister96, 96);
shift_register_builder!(ShiftRegister104, 104);
shift_register_builder!(ShiftRegister112, 112);
shift_register_builder!(ShiftRegister120, 120);
shift_register_builder!(ShiftRegister128, 128);

/// 8 output serial-in parallel-out shift register
pub type ShiftRegister<Pin1, Pin2, Pin3> = ShiftRegister8<Pin1, Pin2, Pin3>;
