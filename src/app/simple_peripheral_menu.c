#include <bcomdef.h>

#if !defined(Display_DISABLE_ALL)
#include <ti/display/Display.h>
#include <menu/two_btn_menu.h>
#include "simple_peripheral_menu.h"
#include "simple_peripheral.h"

/*
 * Menu Lists Initializations
 */

  // Note: BLE_V50_FEATURES is defined by default and PHY_LR_CFG is defined in
  //       build_config.opt
  #if (BLE_V50_FEATURES & PHY_LR_CFG)
/* Menu: Main
     5 actions, no upper */
MENU_OBJ(sbpMenuMain, NULL, 5, NULL)
  MENU_ITEM_ACTION("1 Mbps",                 SimpleBLEPeripheral_doSetPhy)
  MENU_ITEM_ACTION("2 Mbps",                 SimpleBLEPeripheral_doSetPhy)
  MENU_ITEM_ACTION("1 & 2 Mbps",             SimpleBLEPeripheral_doSetPhy)
  MENU_ITEM_ACTION("Coded:S2",               SimpleBLEPeripheral_doSetPhy)
  MENU_ITEM_ACTION("1 & 2 Mbps, & Coded:S2", SimpleBLEPeripheral_doSetPhy)
MENU_OBJ_END
  #else // !PHY_LR_CFG
/* Menu: Main
     3 actions, no upper */
MENU_OBJ(sbpMenuMain, NULL, 3, NULL)
  MENU_ITEM_ACTION("1 Mbps",              SimpleBLEPeripheral_doSetPhy)
  MENU_ITEM_ACTION("2 Mbps",              SimpleBLEPeripheral_doSetPhy)
  MENU_ITEM_ACTION("1 & 2 Mbps",          SimpleBLEPeripheral_doSetPhy)
MENU_OBJ_END
  #endif // PHY_LR_CFG
#endif  // !Display_DISABLE_ALL
