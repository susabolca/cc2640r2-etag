#ifndef SDK_BOARD_H
#define SDK_BOARD_H

#if defined(CC2640R2_LAUNCHXL)
    #include "../../boards/CC2640R2_LAUNCHXL/Board.h"
#else
    #error "Must define 'CC2640R2_LAUNCHXL'. Please set include path to point \
to appropriate device."
#endif

#endif /* SDK_BOARD_H */
