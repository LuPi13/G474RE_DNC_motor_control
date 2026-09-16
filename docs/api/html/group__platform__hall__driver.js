var group__platform__hall__driver =
[
    [ "hall_driver.c", "_core_2_platform_2hall__driver_8c.html", null ],
    [ "hall_driver.h", "_core_2_platform_2hall__driver_8h.html", null ],
    [ "hall_driver_input_config_t", "structhall__driver__input__config__t.html", [
      [ "pin", "structhall__driver__input__config__t.html#a25fb7b4388088f46e52c6be45da25934", null ],
      [ "port", "structhall__driver__input__config__t.html#af1c6334b6c01b3b2d0fc467bfc19a34a", null ]
    ] ],
    [ "hall_driver_config_t", "structhall__driver__config__t.html", [
      [ "hall_a", "structhall__driver__config__t.html#a4260fb18f705b73a94a99b54aa4b945a", null ],
      [ "hall_b", "structhall__driver__config__t.html#a98f43ab03475eb8fd884cd4a70070795", null ],
      [ "hall_c", "structhall__driver__config__t.html#a29f4a581407b922162e08b351736492c", null ],
      [ "timer", "structhall__driver__config__t.html#ae9b4fe9ca20c6dc9db33b44719f14368", null ],
      [ "timer_clock_hz", "structhall__driver__config__t.html#a7c651203ea114a00dca72211be3ee147", null ]
    ] ],
    [ "hall_driver_feedback_t", "structhall__driver__feedback__t.html", [
      [ "capture_count", "structhall__driver__feedback__t.html#a4f053d8c765c0655e1252c0759a7b560", null ],
      [ "capture_ticks", "structhall__driver__feedback__t.html#a8d819690042b6f18ab505bf39ef1d91f", null ],
      [ "edge_interval_s", "structhall__driver__feedback__t.html#ae73fa9b6e4d0747c795dcb0e1534d8cf", null ],
      [ "hall_state", "structhall__driver__feedback__t.html#af81adcff24c36b4a255b11a762d1e645", null ],
      [ "has_state_sample", "structhall__driver__feedback__t.html#a12046055d891f749f090b9ef09726607", null ],
      [ "has_valid_interval", "structhall__driver__feedback__t.html#ae2dc2e8f420bd5e781e7adff0c6e3415", null ],
      [ "invalid_capture_count", "structhall__driver__feedback__t.html#ab52fbb302d55e59e7ff37376ed879364", null ],
      [ "is_timed_out", "structhall__driver__feedback__t.html#a1b6b21ec45157a838d47494ded49ccbb", null ],
      [ "timeout_count", "structhall__driver__feedback__t.html#a5523c1e5611819e8594bd30294b5ba15", null ]
    ] ],
    [ "hall_driver_t", "structhall__driver__t.html", [
      [ "active_feedback_index", "structhall__driver__t.html#a3adeb7f43a0779d3f159ee09a1c09e97", null ],
      [ "capture_with_pending_timeout", "structhall__driver__t.html#a609ce7448009573f14d5dfdfb5d446d0", null ],
      [ "config", "structhall__driver__t.html#ade5d73ceb912dbdb952e13802a2bca65", null ],
      [ "counter_frequency_hz", "structhall__driver__t.html#a81256f349ea973ced32f62c98d65268c", null ],
      [ "feedback_buffer", "structhall__driver__t.html#a69e75144427f8107dc5c579dec812317", null ],
      [ "has_valid_interval_reference", "structhall__driver__t.html#adaeefe03ad95b21b464b54527d2fe9b5", null ],
      [ "is_initialized", "structhall__driver__t.html#aa14dfbef660d96c72dc14b77996e658b", null ],
      [ "is_running", "structhall__driver__t.html#a88651ebf2bf97e503d19c339021f8bd0", null ],
      [ "timeout_s", "structhall__driver__t.html#af6d581162aaab8fcf62430f5caeb20db", null ]
    ] ],
    [ "hall_driver_signal_feedback_t", "group__platform__hall__driver.html#gaa895412d47986ed45831c9bfd7ac0ecf", null ],
    [ "hall_driver_status_t", "group__platform__hall__driver.html#ga0fab1479be005df9c115534d44ab9eb3", [
      [ "HALL_DRIVER_STATUS_OK", "group__platform__hall__driver.html#gga0fab1479be005df9c115534d44ab9eb3af8a9fee52a5a74bf8826ecdb3f226e13", null ],
      [ "HALL_DRIVER_STATUS_INVALID_ARGUMENT", "group__platform__hall__driver.html#gga0fab1479be005df9c115534d44ab9eb3a4ae6ff54ab0b65af43566df308c47692", null ],
      [ "HALL_DRIVER_STATUS_INVALID_CONFIG", "group__platform__hall__driver.html#gga0fab1479be005df9c115534d44ab9eb3a49239577dddd1d56255ff6903f987aee", null ],
      [ "HALL_DRIVER_STATUS_INVALID_STATE", "group__platform__hall__driver.html#gga0fab1479be005df9c115534d44ab9eb3a1743905cff71d0244d5b57d37f3a9ee4", null ],
      [ "HALL_DRIVER_STATUS_INVALID_CAPTURE", "group__platform__hall__driver.html#gga0fab1479be005df9c115534d44ab9eb3a8480f380a1cc8f834d35214df8464b43", null ],
      [ "HALL_DRIVER_STATUS_HAL_ERROR", "group__platform__hall__driver.html#gga0fab1479be005df9c115534d44ab9eb3a32d1e04c9697c120d37d1884ed100e83", null ]
    ] ],
    [ "hall_driver_get_feedback", "group__platform__hall__driver.html#ga4f4d3833350266300f461d558d84d45f", null ],
    [ "hall_driver_get_signal_feedback", "group__platform__hall__driver.html#gad8515c4a38e4338c9da7fa412e18e3e0", null ],
    [ "hall_driver_handle_capture", "group__platform__hall__driver.html#ga20272742cde3e2b6c6a893cc6f4b31a7", null ],
    [ "hall_driver_handle_timeout", "group__platform__hall__driver.html#ga7a7279e1aa117b59e1ba7ce4c488ea2a", null ],
    [ "hall_driver_init", "group__platform__hall__driver.html#ga23e13488b3a456efca526dd010a08686", null ],
    [ "hall_driver_start", "group__platform__hall__driver.html#ga2339efb06f87e9b6cb5a4b9b4eca0b4e", null ],
    [ "hall_driver_stop", "group__platform__hall__driver.html#ga208a72b8955aaf3b703570b3ee80e665", null ]
];