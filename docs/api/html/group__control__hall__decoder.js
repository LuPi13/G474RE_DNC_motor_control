var group__control__hall__decoder =
[
    [ "hall_decoder.c", "hall__decoder_8c.html", null ],
    [ "hall_decoder.h", "hall__decoder_8h.html", null ],
    [ "hall_decoder_profile_t", "structhall__decoder__profile__t.html", [
      [ "forward_edge_angle_rad", "structhall__decoder__profile__t.html#ad15b0b820fbc1dd6b4e621000ad55dda", null ],
      [ "sector_by_state", "structhall__decoder__profile__t.html#a5dedd9226ac89f1822c3509957769000", null ]
    ] ],
    [ "hall_decoder_output_t", "structhall__decoder__output__t.html", [
      [ "direction", "structhall__decoder__output__t.html#a96d773256454e5fd6afd97f6e9e92b5d", null ],
      [ "hall_state", "structhall__decoder__output__t.html#a07b1797ea89876a4526e394156956e11", null ],
      [ "has_valid_angle", "structhall__decoder__output__t.html#a03f0d20ae437157a4bf7b96bf6e04c3a", null ],
      [ "has_valid_direction", "structhall__decoder__output__t.html#a438e53e48afe4fc017e2455d4677b053", null ],
      [ "has_valid_speed", "structhall__decoder__output__t.html#a9192c89823ba7998ae76f2b2c7387ed4", null ],
      [ "has_valid_state", "structhall__decoder__output__t.html#ae03d903d4ee5f81d2588e80cd5b096e7", null ],
      [ "is_angle_from_edge", "structhall__decoder__output__t.html#a961ac17a5220340620bc31ef3bb5680e", null ],
      [ "is_timed_out", "structhall__decoder__output__t.html#abb173abd51edf01a459b514ebc33c3e8", null ],
      [ "omega_e_rad_s", "structhall__decoder__output__t.html#ac7a0177789a0c36deeb05609dcfafa11", null ],
      [ "sector", "structhall__decoder__output__t.html#acacb37ae35b86bf56fcb68431525bd32", null ],
      [ "sector_span_rad", "structhall__decoder__output__t.html#a075e01daecd28482a5f470c08377ebca", null ],
      [ "theta_e_rad", "structhall__decoder__output__t.html#a72a564422f50c3a1b39ba69edf049b04", null ],
      [ "transition_count", "structhall__decoder__output__t.html#a35f9a7f698c5052b7e18109b1d577434", null ]
    ] ],
    [ "hall_decoder_t", "structhall__decoder__t.html", [
      [ "has_observation", "structhall__decoder__t.html#ab9059b3148fb244564f5793d39db2806", null ],
      [ "invalid_state_count", "structhall__decoder__t.html#ab4f9fa7c855472785b9aa2fd56499064", null ],
      [ "invalid_transition_count", "structhall__decoder__t.html#a1c3f1de7eb286ea43a52155fbc2d34c7", null ],
      [ "is_initialized", "structhall__decoder__t.html#aafe829c505ac33d3f1e32264ef25d244", null ],
      [ "last_capture_count", "structhall__decoder__t.html#ae0667d71983b058ed91b3fbd10c37f86", null ],
      [ "missed_capture_count", "structhall__decoder__t.html#ab86e269138e2266ad8c67df2cd2387b3", null ],
      [ "output", "structhall__decoder__t.html#a23fca085c21fe6028937c1a9cc58bb6d", null ],
      [ "profile", "structhall__decoder__t.html#ac5e4e12528a1745f5663020adc4adb2a", null ],
      [ "sector_center_angle_rad", "structhall__decoder__t.html#a7268cdedd825c2c151821ebe51a34cb1", null ],
      [ "sector_span_rad", "structhall__decoder__t.html#af11258c893815d618dc166d0722a0509", null ]
    ] ],
    [ "hall_decoder_observation_t", "group__control__hall__decoder.html#ga22bdbb9ea258e73bf28e8a0f14f9141c", null ],
    [ "hall_decoder_direction_t", "group__control__hall__decoder.html#ga48dab52f0c83f355ca0583b7ee358183", null ],
    [ "hall_decoder_status_t", "group__control__hall__decoder.html#ga9249cbf229a48546e2a6275d729ac83b", [
      [ "HALL_DECODER_STATUS_OK", "group__control__hall__decoder.html#gga9249cbf229a48546e2a6275d729ac83baa319c532b7095109a2dc1a3b5b899647", null ],
      [ "HALL_DECODER_STATUS_INVALID_ARGUMENT", "group__control__hall__decoder.html#gga9249cbf229a48546e2a6275d729ac83bafb2fe3f40fd1c0df0bcbe6039aca5ea1", null ],
      [ "HALL_DECODER_STATUS_INVALID_CONFIG", "group__control__hall__decoder.html#gga9249cbf229a48546e2a6275d729ac83ba571c7ffc3862520b0ff46d4f6a0cd4fc", null ],
      [ "HALL_DECODER_STATUS_INVALID_STATE", "group__control__hall__decoder.html#gga9249cbf229a48546e2a6275d729ac83ba44d50dc63e08c0e157da1296664e269c", null ],
      [ "HALL_DECODER_STATUS_INVALID_OBSERVATION", "group__control__hall__decoder.html#gga9249cbf229a48546e2a6275d729ac83ba57357229baeb15a1fd3a163974018ecc", null ],
      [ "HALL_DECODER_STATUS_INVALID_HALL_STATE", "group__control__hall__decoder.html#gga9249cbf229a48546e2a6275d729ac83babbf212c90041708373ea0d51005e3b79", null ],
      [ "HALL_DECODER_STATUS_INVALID_TRANSITION", "group__control__hall__decoder.html#gga9249cbf229a48546e2a6275d729ac83ba8001d0e81cdf99cbe16224ce77f67687", null ],
      [ "HALL_DECODER_STATUS_MISSED_CAPTURE", "group__control__hall__decoder.html#gga9249cbf229a48546e2a6275d729ac83bac19812c45e5cf1442bd3be31ff2c175e", null ]
    ] ],
    [ "hall_decoder_get_latest_output_fast", "group__control__hall__decoder.html#gaa61380709263b0631a0fc272076f847b", null ],
    [ "hall_decoder_init", "group__control__hall__decoder.html#gaca3501674d1d4cb70bdde84a6aba2d12", null ],
    [ "hall_decoder_reset", "group__control__hall__decoder.html#gac37b451d7b006a9a80f1cfc72c2b3a97", null ],
    [ "hall_decoder_update", "group__control__hall__decoder.html#ga2c9a4d228339320d6175ce47d347eef2", null ],
    [ "hall_decoder_update_fast", "group__control__hall__decoder.html#ga76fedba818e4ef65956a41f372dd038e", null ]
];