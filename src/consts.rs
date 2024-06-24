/// Define flaash NVS addresses
pub const NVS_BASE_ADDRESS: u32 = 0x9000;
pub const NVS_DEV_EUI_ADDRESS: u32 = NVS_BASE_ADDRESS; // SIZE OF 8
pub const NVS_APP_EUI_ADDRESS: u32 = NVS_DEV_EUI_ADDRESS + 8; // SIZE OF 8
pub const NVS_APP_KEY_ADDRESS: u32 = NVS_APP_EUI_ADDRESS + 8; // SIZE OF 16
pub const NVS_WAKEUP_PERIOD_ADDRESS: u32 = NVS_APP_KEY_ADDRESS + 16; // SIZE OF 32

/// Lora Max TX power
pub const MAX_TX_POWER: u8 = 14;

/// Lora data fame size in bytes
pub const LORA_FRAME_SIZE_BYTES: usize = 24;
