var group__control__motor__control =
[
    [ "motor_control.c", "motor__control_8c.html", null ],
    [ "motor_control.h", "motor__control_8h.html", null ],
    [ "motor_control_config_t", "structmotor__control__config__t.html", [
      [ "current_reference_fall_rate_per_s", "structmotor__control__config__t.html#a6489049f4e6d6a59c20a4a5cab098560", null ],
      [ "current_reference_magnitude_limit", "structmotor__control__config__t.html#a6d1d4b4e3b42bc315f1ab46cf382de35", null ],
      [ "current_reference_max", "structmotor__control__config__t.html#ac718378f1052de7b183dd6416c99cacb", null ],
      [ "current_reference_min", "structmotor__control__config__t.html#a97e69fedc67d6998aad6889c7ea9746f", null ],
      [ "current_reference_rise_rate_per_s", "structmotor__control__config__t.html#a23bed19b17cbf68620bc83a38e0c9763", null ],
      [ "foc", "structmotor__control__config__t.html#af606c94bd858eca31658aac7ae4870cf", null ],
      [ "pole_pairs", "structmotor__control__config__t.html#a30c569c62d37150ffb44e85e68936a3b", null ],
      [ "sampling_period_s", "structmotor__control__config__t.html#a97ef4378c836e6962c875a6d902b336e", null ],
      [ "speed_controller", "structmotor__control__config__t.html#a2720d571d44a6669a63eb831f0ba0760", null ],
      [ "speed_reference_fall_rate_rad_s2", "structmotor__control__config__t.html#a51db473cd2b7dee748b2774b48e7ade4", null ],
      [ "speed_reference_max_rad_s", "structmotor__control__config__t.html#abcbaeb34e7df859839d6baec334933ba", null ],
      [ "speed_reference_min_rad_s", "structmotor__control__config__t.html#a855db1039eac499493450fc4141140e0", null ],
      [ "speed_reference_rise_rate_rad_s2", "structmotor__control__config__t.html#ab334f07589d396766ddf0055821a4a06", null ]
    ] ],
    [ "motor_control_current_reference_target_t", "structmotor__control__current__reference__target__t.html", [
      [ "i_dq_ref", "structmotor__control__current__reference__target__t.html#a60835cc915f8831a17c56d4b1a0c7934", null ],
      [ "was_saturated", "structmotor__control__current__reference__target__t.html#a3e0d34681523040193a84325ecaa45b7", null ]
    ] ],
    [ "motor_control_input_t", "structmotor__control__input__t.html", [
      [ "cos_theta", "structmotor__control__input__t.html#aa389b2acad20c6c7478660b171ac60c0", null ],
      [ "i_abc", "structmotor__control__input__t.html#a263278843fae7e854230fb7ed6599e59", null ],
      [ "omega_e_rad_s", "structmotor__control__input__t.html#adbe86f90cb427b4b4aa968ba085b992d", null ],
      [ "requested_i_dq_ref", "structmotor__control__input__t.html#a7e294fccbff756023f29d78b087f6608", null ],
      [ "sin_theta", "structmotor__control__input__t.html#a7cd51ebea1ad550cb408ff8b2d69f287", null ],
      [ "v_dc", "structmotor__control__input__t.html#ad8f52b42a2a22c1bcad4f945480f1346", null ]
    ] ],
    [ "motor_control_fast_input_t", "structmotor__control__fast__input__t.html", [
      [ "cos_theta", "structmotor__control__fast__input__t.html#a33bb92675cef33b7254bf166bb07c571", null ],
      [ "current_reference_target", "structmotor__control__fast__input__t.html#ac0b08feff8bce6ee96d0f325f75ccb19", null ],
      [ "i_abc", "structmotor__control__fast__input__t.html#a5580a6da0c400a8546341cfb63df623a", null ],
      [ "omega_e_rad_s", "structmotor__control__fast__input__t.html#ab4518059d81f8378dc2710bcdc30b77f", null ],
      [ "sin_theta", "structmotor__control__fast__input__t.html#a874fa8d8113485b5ee6ba2da999239ba", null ],
      [ "v_dc", "structmotor__control__fast__input__t.html#ad372a2befce181655eb3e8535762bda3", null ]
    ] ],
    [ "motor_control_speed_input_t", "structmotor__control__speed__input__t.html", [
      [ "omega_e_feedback_rad_s", "structmotor__control__speed__input__t.html#a4ed0313fce43cbb7038909a05595e1c7", null ],
      [ "omega_m_requested_rad_s", "structmotor__control__speed__input__t.html#a7eefb8c63e2634984f1654bf861ea817", null ]
    ] ],
    [ "motor_control_speed_output_t", "structmotor__control__speed__output__t.html", [
      [ "current_reference_target", "structmotor__control__speed__output__t.html#abb0623484bb038f10b5ff00ef73ae9ba", null ],
      [ "omega_m_feedback_rad_s", "structmotor__control__speed__output__t.html#a732ee88ffc407e8dbe5b4da52b8885f9", null ],
      [ "omega_m_ref_limited_rad_s", "structmotor__control__speed__output__t.html#add5c6d42e5bada66a9589ac263629397", null ],
      [ "speed_controller", "structmotor__control__speed__output__t.html#a2bbe8d695a6b234b026f86d4a263c1c7", null ]
    ] ],
    [ "motor_control_output_t", "structmotor__control__output__t.html", [
      [ "foc", "structmotor__control__output__t.html#a466ba0d349f126878c71075cc9832179", null ],
      [ "i_dq_ref", "structmotor__control__output__t.html#a81bb7a03189f7a9894113ada56c44c82", null ],
      [ "is_current_reference_rate_limited", "structmotor__control__output__t.html#ac1850a31c52b0b7eabb40f804532e6a9", null ],
      [ "is_current_reference_saturated", "structmotor__control__output__t.html#a99e30a5d545c6537ea6374d3b3564e48", null ]
    ] ],
    [ "motor_control_profile_segment_t", "structmotor__control__profile__segment__t.html", [
      [ "last_cycles", "structmotor__control__profile__segment__t.html#ad0472cc762b28b88dc0c8cf5febd43ad", null ],
      [ "max_cycles", "structmotor__control__profile__segment__t.html#a9d5e16173991527aaa7699f2accf2931", null ]
    ] ],
    [ "motor_control_profile_t", "structmotor__control__profile__t.html", [
      [ "complete_sample_count", "structmotor__control__profile__t.html#acd79daf2828c5b0990bbd1e9125d6c27", null ],
      [ "foc", "structmotor__control__profile__t.html#adc8db202c2c08e388c60e943ab7cf84a", null ],
      [ "foc_detail", "structmotor__control__profile__t.html#a1a8f9feb3baae3aa4152522a6e6da46e", null ],
      [ "is_last_sample_complete", "structmotor__control__profile__t.html#a37898951eb21324ef33deacb34cf174c", null ],
      [ "output", "structmotor__control__profile__t.html#a1fad2f07b83f6b1acb75a09c769312f4", null ],
      [ "reference", "structmotor__control__profile__t.html#a40409eca24cdba538df5955e1562f9f5", null ]
    ] ],
    [ "motor_control_t", "structmotor__control__t.html", [
      [ "config", "structmotor__control__t.html#a5af03972ab40f73b8e6cc4568b99384f", null ],
      [ "foc", "structmotor__control__t.html#a52aad3a8bc7dadbeec0d0fb57820385a", null ],
      [ "i_d_rate_limiter", "structmotor__control__t.html#a8511c7463b4e8d5a5642b6bd231f3fdc", null ],
      [ "i_dq_ref", "structmotor__control__t.html#a7f8db50ad2e0a38e3a6a133ed303fb18", null ],
      [ "i_q_rate_limiter", "structmotor__control__t.html#a859689eb99c50be98bc587e580e80a35", null ],
      [ "is_current_reference_rate_limited", "structmotor__control__t.html#aa0c8b173981edef9b6a7e60da515739f", null ],
      [ "is_current_reference_saturated", "structmotor__control__t.html#a006b5a7eeb95737a9dd1ca9da9baef89", null ],
      [ "is_initialized", "structmotor__control__t.html#a02670240d6f36a260fea84cf5ed5ec75", null ],
      [ "speed_controller", "structmotor__control__t.html#a714efca7c72faa419bf63f54187b0847", null ],
      [ "speed_reference_rate_limiter", "structmotor__control__t.html#a1cb60133536a7395a946a5a3e0039188", null ]
    ] ],
    [ "motor_control_cycle_counter_reader_t", "group__control__motor__control.html#ga584521de505ba3af6acbeecaee367bb2", null ],
    [ "motor_control_status_t", "group__control__motor__control.html#gafa2f3b5d702156315547d16195a60f8d", [
      [ "MOTOR_CONTROL_STATUS_OK", "group__control__motor__control.html#ggafa2f3b5d702156315547d16195a60f8da8d6e172f376a438ef8577104c73d3515", null ],
      [ "MOTOR_CONTROL_STATUS_INVALID_ARGUMENT", "group__control__motor__control.html#ggafa2f3b5d702156315547d16195a60f8daed2ab91bd6fde893dc5f925b36cb12f5", null ],
      [ "MOTOR_CONTROL_STATUS_INVALID_CONFIG", "group__control__motor__control.html#ggafa2f3b5d702156315547d16195a60f8da3aff18e66cc457a5b730210af73bb9c8", null ],
      [ "MOTOR_CONTROL_STATUS_INVALID_STATE", "group__control__motor__control.html#ggafa2f3b5d702156315547d16195a60f8da34431531e9f437cb921e13ed57731ad5", null ],
      [ "MOTOR_CONTROL_STATUS_RATE_LIMITER_ERROR", "group__control__motor__control.html#ggafa2f3b5d702156315547d16195a60f8dabbd31cddb2be1bd83100ac1b462bac28", null ],
      [ "MOTOR_CONTROL_STATUS_FOC_ERROR", "group__control__motor__control.html#ggafa2f3b5d702156315547d16195a60f8da6551eec3a8beb4a144648c80748ebc32", null ],
      [ "MOTOR_CONTROL_STATUS_SPEED_CONTROLLER_ERROR", "group__control__motor__control.html#ggafa2f3b5d702156315547d16195a60f8da0faca9a772d974abfe2a20bebdf033c0", null ]
    ] ],
    [ "motor_control_init", "group__control__motor__control.html#ga9f3c12f6e01a2e19f79664b358739326", null ],
    [ "motor_control_prepare_current_reference_target", "group__control__motor__control.html#ga766bb5754f75406d04ec350969234de1", null ],
    [ "motor_control_reset", "group__control__motor__control.html#ga6d2149a281af0e4dcfdc21eac724dff4", null ],
    [ "motor_control_reset_current_reference", "group__control__motor__control.html#gaf47de94c41079e182d4ff12f9c44b439", null ],
    [ "motor_control_update", "group__control__motor__control.html#ga4934e12a0dea11517c17731ef2f49192", null ],
    [ "motor_control_update_current_reference", "group__control__motor__control.html#gac768a732db7cc459e85cf587585b07bc", null ],
    [ "motor_control_update_fast", "group__control__motor__control.html#gac705b160cd53c8e3ddfdf9d6c6c70c7e", null ],
    [ "motor_control_update_fast_profiled", "group__control__motor__control.html#ga4a40835bdd82326966a2a8831f5880f4", null ],
    [ "motor_control_update_fast_voltage", "group__control__motor__control.html#gae427220aa14acbed510287ec07441295", null ],
    [ "motor_control_update_profiled", "group__control__motor__control.html#gaaa6bb72f3de50ea5f9847658b53af5a9", null ],
    [ "motor_control_update_speed", "group__control__motor__control.html#ga2ddfbd671791d08b598f3d5fc3bda2d1", null ]
];