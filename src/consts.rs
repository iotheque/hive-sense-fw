/// Define flash NVS addresses
pub const NVS_BASE_ADDRESS: u32 = 0x9000;
pub const NVS_DEV_EUI_ADDRESS: u32 = NVS_BASE_ADDRESS; // SIZE OF 8
pub const NVS_APP_EUI_ADDRESS: u32 = NVS_DEV_EUI_ADDRESS + 8; // SIZE OF 8
pub const NVS_APP_KEY_ADDRESS: u32 = NVS_APP_EUI_ADDRESS + 8; // SIZE OF 16
pub const NVS_WAKEUP_PERIOD_ADDRESS: u32 = NVS_APP_KEY_ADDRESS + 16; // SIZE OF 32
pub const NVS_HX711_1_TARE_VALUE: u32 = NVS_WAKEUP_PERIOD_ADDRESS + 32; // SIZE OF 32
pub const NVS_HX711_2_TARE_VALUE: u32 = NVS_HX711_1_TARE_VALUE + 32; // SIZE OF 32

/// Lora Max TX power
pub const MAX_TX_POWER: u8 = 14;

/// Size of a BSSID in a Lora frame in bytes
pub const BSSID_ITEM_SIZE: usize = 6;
/// Number of BSSID send to Lora
pub const BSSID_ITEM_NUMBER: usize = 3;
/// Total BSSIDs field size on Lora frame in bytes
pub const BSSIDS_TOTAL_SIZE: usize = BSSID_ITEM_NUMBER * BSSID_ITEM_SIZE;

/// Lora data fame size in bytes
/// Fields (bytes)
/// - [0] device id
/// - [1] Vbat (value in mv - 3000 / 5)
/// - [2] HX711 value bit 0 to 7
/// - [3] HX711 value bit 8 to 15
/// - [4] HX711 value bit 16 to 23
/// - [5] BSSID number 1
/// - [6] BSSID number 1
/// - [7] BSSID number 1
/// - [8] BSSID number 1
/// - [9] BSSID number 1
/// - [10] BSSID number 1
/// - [11] BSSID number 2
/// - [12] BSSID number 2
/// - [13] BSSID number 2
/// - [14] BSSID number 2
/// - [15] BSSID number 2
/// - [16] BSSID number 2
/// - [17] BSSID number 3
/// - [18] BSSID number 3
/// - [19] BSSID number 3
/// - [20] BSSID number 3
/// - [21] BSSID number 3
/// - [22] BSSID number 3
pub const LORA_FRAME_SIZE_BYTES: usize = 23;
