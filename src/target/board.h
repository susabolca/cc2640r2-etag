// select board based on MCU type.
#if defined(CC2640R2_LAUNCHXL)
    #include "./cc2640r2lp/cc2640r2lp_board.h"
#else // unknown board
    #error "***ERROR*** Invalid Board Specified! Please see board.h for options."
#endif
