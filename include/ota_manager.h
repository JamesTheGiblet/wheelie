#ifndef OTA_MANAGER_H
#define OTA_MANAGER_H

#include <ArduinoOTA.h>

// ═══════════════════════════════════════════════════════════════════════════
// OTA UPDATE SYSTEM - Over-The-Air firmware and filesystem updates
// ═══════════════════════════════════════════════════════════════════════════

// OTA configuration structure
typedef struct {
  bool enabled;
  bool update_in_progress;
  String hostname;
  String password;
  unsigned long last_check;
  int update_progress;
  bool require_auth;
  unsigned long update_timeout;
} OTAManager_t;

extern OTAManager_t otaManager;

// Core Functions
void initializeOTA();
void handleOTA();
void checkForOTAUpdate();

// Control Functions
void enableOTA(bool enable);
void setOTAPassword(String newPassword);
void setOTAHostname(String newHostname);
void forceOTAMode();
bool isOTAInProgress();

// Status Reporting
void printOTAStatus();

#endif // OTA_MANAGER_H