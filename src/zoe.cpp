#include "zoe.h"

// ZOE CAN computer related functions ****************************************
uint16_t getRequestId(uint16_t responseId) {
                      // from ECU id   to ECU id
  if      (responseId == 0x7ec) return 0x7e4; // EVC
  else if (responseId == 0x7da) return 0x7ca; // TCU
  else if (responseId == 0x7bb) return 0x79b; // LBC
  else if (responseId == 0x77e) return 0x75a; // PEB
  else if (responseId == 0x772) return 0x752; // Airbag
  else if (responseId == 0x76d) return 0x74d; // UDP
  else if (responseId == 0x763) return 0x743; // instrument panel
  else if (responseId == 0x762) return 0x742; // PAS
  else if (responseId == 0x760) return 0x740; // ABS
  else if (responseId == 0x7bc) return 0x79c; // UBP
  else if (responseId == 0x765) return 0x745; // BCM
  else if (responseId == 0x764) return 0x744; // CLIM
  else if (responseId == 0x76e) return 0x74e; // UPA
  else if (responseId == 0x793) return 0x792; // BCB
  else if (responseId == 0x7b6) return 0x796; // LBC2
  else if (responseId == 0x722) return 0x702; // LINSCH
  else if (responseId == 0x767) return 0x747; // AUTOS (R-LINK)
  else return 0;
}
