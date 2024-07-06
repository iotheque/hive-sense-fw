use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, signal::Signal};
use esp_hal::peripherals::WIFI;
use esp_wifi::wifi::{AccessPointInfo, WifiError, WifiStaDevice};

use crate::consts::{BSSIDS_TOTAL_SIZE, BSSID_ITEM_NUMBER};

/// Wifi scan
#[embassy_executor::task]
pub async fn scan_wifi(
    init: esp_wifi::EspWifiInitialization,
    wifi: WIFI,
    signal: &'static Signal<CriticalSectionRawMutex, [u8; BSSIDS_TOTAL_SIZE]>,
) {
    let mut out: [u8; BSSIDS_TOTAL_SIZE] = [0; BSSIDS_TOTAL_SIZE];
    let (_, mut controller) = esp_wifi::wifi::new_with_mode(&init, wifi, WifiStaDevice).unwrap();
    controller.start().unwrap();
    let res: Result<(heapless::Vec<AccessPointInfo, BSSID_ITEM_NUMBER>, usize), WifiError> =
        controller.scan_n();
    match res {
        Ok((access_points, count)) => {
            log::info!("Number of access points found: {}", count);
            for (i, ap) in access_points.iter().enumerate().take(BSSID_ITEM_NUMBER) {
                log::info!("SSID: {}", ap.ssid);
                log::info!(
                    "BSSID: {:02X}:{:02X}:{:02X}:{:02X}:{:02X}:{:02X}",
                    ap.bssid[0],
                    ap.bssid[1],
                    ap.bssid[2],
                    ap.bssid[3],
                    ap.bssid[4],
                    ap.bssid[5]
                );
                out[(6 * i)..(6 * i + 6)].copy_from_slice(&ap.bssid);
            }
        }
        Err(e) => {
            log::error!("Failed to scan WiFi: {:?}", e);
        }
    }
    signal.signal(out);
}
