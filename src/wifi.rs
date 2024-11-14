use esp_hal::peripherals::WIFI;
use esp_wifi::wifi::{AccessPointInfo, WifiError, WifiStaDevice};

/// Wifi scan
pub fn scan_wifi(init: esp_wifi::EspWifiInitialization, wifi: WIFI, out: &mut [u8]) {
    let (_, mut controller) = esp_wifi::wifi::new_with_mode(&init, wifi, WifiStaDevice).unwrap();
    controller.start().unwrap();
    let res: Result<(heapless::Vec<AccessPointInfo, 10>, usize), WifiError> = controller.scan_n();
    match res {
        Ok((access_points, count)) => {
            log::info!("Number of access points found: {}", count);
            for (i, ap) in access_points.iter().enumerate().take(2) {
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
                out[i..(6 + i)].copy_from_slice(&ap.bssid);
            }
        }
        Err(e) => {
            log::error!("Failed to scan WiFi: {:?}", e);
        }
    }
}
