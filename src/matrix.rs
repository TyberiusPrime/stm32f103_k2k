use stm32f1xx_hal::gpio::{gpioa::*, gpiob::*, Input, Output, PullUp, PushPull};
//use stm32f1xx_hal::prelude::*;
use crate::StringSender;
use embedded_hal::digital::v2::{InputPin, OutputPin};
#[allow(unused_imports)]
use embedded_hal::digital::v2_compat;
use no_std_compat::prelude::v1::*;
use smallbitvec::SmallBitVec;

type SinksA = Vec<PAx<Input<PullUp>>>;
type SinksB = Vec<PBx<Input<PullUp>>>;

pub struct Matrix {
    sinks_pa: SinksA,
    sinks_pb: SinksB,
    sources_pa: Vec<PAx<Output<PushPull>>>,
    sources_pb: Vec<PBx<Output<PushPull>>>,
    output: SmallBitVec,
}

impl Matrix {
    pub fn new(
        sinks_pa: SinksA,
        sinks_pb: SinksB,
        sources_pa: Vec<PAx<Output<PushPull>>>,
        sources_pb: Vec<PBx<Output<PushPull>>>,
    ) -> Matrix {
        let sink_count = sinks_pa.len() + sinks_pb.len();
        let source_count = sources_pa.len() + sources_pb.len();
        let output = SmallBitVec::with_capacity(sink_count * source_count);
        Matrix {
            sinks_pa,
            sinks_pb,
            sources_pa,
            sources_pb,
            output,
        }
    }

    pub fn read_matrix(&mut self) -> &SmallBitVec {
        self.output.clear();
        for source in self.sources_pa.iter_mut() {
            source.set_high().ok();
        }
        for source in self.sources_pb.iter_mut() {
            source.set_high().ok();
        }

        for source in self.sources_pa.iter_mut() {
            source.set_low().ok();
            Self::read_row(&mut self.output, &self.sinks_pa, &self.sinks_pb);
            source.set_high().ok();
        }
        for source in self.sources_pb.iter_mut() {
            source.set_low().ok();
            Self::read_row(&mut self.output, &self.sinks_pa, &self.sinks_pb);
            source.set_high().ok();
        }
        &self.output
    }

    fn read_row(output: &mut SmallBitVec, sinks_pa: &SinksA, sinks_pb: &SinksB) {
        for sink in sinks_pa.iter() {
            output.push(sink.is_low().unwrap_or(false));
        }
        for sink in sinks_pb.iter() {
            output.push(sink.is_low().unwrap_or(false));
        }
    }

    pub fn debug_serial(&self, tx: &mut impl StringSender) {
        for (ii, value) in self.output.iter().enumerate() {
            let o = format!("{}: {}", ii, value);
            tx.writeln(&o);
        }
    }
}
