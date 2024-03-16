#if defined(CC2640R2_LAUNCHXL)
    #warning "Target overrided."
    #include "./cc2640r2lp/cc2640r2lp_board.c"
#else // unknown board
    #error "***ERROR*** Invalid Board Specified! Please see board.h for options."
#endif
