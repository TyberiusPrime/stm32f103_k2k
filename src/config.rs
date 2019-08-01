use crate::matrix::Matrix;
use stm32f1xx_hal;
use no_std_compat::prelude::v1::*;

pub fn get_matrix(
    mut gpioa: stm32f1xx_hal::gpio::gpioa::Parts,
    mut gpiob: stm32f1xx_hal::gpio::gpiob::Parts
    ) -> (
    Matrix,
    stm32f1xx_hal::gpio::gpioa::Parts,
    stm32f1xx_hal::gpio::gpiob::Parts
    )
     {
    (Matrix::new(
            vec![
                gpioa.pa7.into_pull_up_input(&mut gpioa.crl).downgrade(),
                gpioa.pa6.into_pull_up_input(&mut gpioa.crl).downgrade(),
                gpioa.pa5.into_pull_up_input(&mut gpioa.crl).downgrade(),
                gpioa.pa4.into_pull_up_input(&mut gpioa.crl).downgrade(),
                gpioa.pa3.into_pull_up_input(&mut gpioa.crl).downgrade(),
                gpioa.pa2.into_pull_up_input(&mut gpioa.crl).downgrade(),
            ],
            vec![
                gpiob.pb11.into_pull_up_input(&mut gpiob.crh).downgrade(),
                gpiob.pb10.into_pull_up_input(&mut gpiob.crh).downgrade(),
                gpiob.pb1.into_pull_up_input(&mut gpiob.crl).downgrade(),
                gpiob.pb0.into_pull_up_input(&mut gpiob.crl).downgrade(),
            ],
            vec![
                gpioa.pa8.into_push_pull_output(&mut gpioa.crh).downgrade(),
                gpioa.pa15.into_push_pull_output(&mut gpioa.crh).downgrade(),
            ],
            vec![
                gpiob.pb12.into_push_pull_output(&mut gpiob.crh).downgrade(),
                gpiob.pb13.into_push_pull_output(&mut gpiob.crh).downgrade(),
                gpiob.pb14.into_push_pull_output(&mut gpiob.crh).downgrade(),
                gpiob.pb15.into_push_pull_output(&mut gpiob.crh).downgrade(),
                gpiob.pb3.into_push_pull_output(&mut gpiob.crl).downgrade(),
                gpiob.pb4.into_push_pull_output(&mut gpiob.crl).downgrade(),
                gpiob.pb5.into_push_pull_output(&mut gpiob.crl).downgrade(),
            ],
        ), gpioa, gpiob)
}