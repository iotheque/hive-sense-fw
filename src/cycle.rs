use crate::{consts, BOOT_CNT, IS_JOIN};
use embassy_time::{Duration, Timer};
use embedded_storage::ReadStorage;
use esp_hal::{
    reset::{get_reset_reason, get_wakeup_cause},
    rtc_cntl::{sleep::TimerWakeupSource, Rtc, SocResetReason},
};
use esp_storage::FlashStorage;

pub fn cycle_start() {
    log::info!("SW version {:?}", env!("CARGO_PKG_VERSION"));
    let reason = get_reset_reason().unwrap_or(SocResetReason::ChipPowerOn);
    log::info!("Reset reason: {:?}", reason);
    let wake_reason = get_wakeup_cause();
    log::info!("Wake reason: {:?}", wake_reason);

    unsafe {
        log::info!("BOOT_CNT {:x?}", BOOT_CNT);
        BOOT_CNT += 1;
        log::debug!("IS_JOIN: {:?}", IS_JOIN);
    }
}

pub async fn cycle_end(mut rtc: Rtc<'_>) {
    log::info!("End of cycle, go to sleep");
    let mut wake_period_raw = [0u8; 2];
    let mut flash = FlashStorage::new();
    flash
        .read(consts::NVS_WAKEUP_PERIOD_ADDRESS, &mut wake_period_raw)
        .unwrap();
    let wake_period_s: u16 = u16::from_be_bytes(wake_period_raw);

    log::info!("Next wakeup in {:?} s", wake_period_s);
    let timer = TimerWakeupSource::new(core::time::Duration::from_secs(wake_period_s.into()));
    Timer::after(Duration::from_millis(100)).await;
    rtc.sleep_deep(&[&timer]);
}
