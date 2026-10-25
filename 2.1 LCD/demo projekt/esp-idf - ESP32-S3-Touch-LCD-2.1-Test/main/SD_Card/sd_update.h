#pragma once

// Prueft beim Boot, ob im Root der SD-Karte eine "update.bin" liegt, spielt
// sie als OTA-Firmware-Image ein (identische Validierung wie der WLAN-Upload
// in ota_web.c) und benennt die Datei danach in "update.backup" um - so wird
// sie nicht bei jedem weiteren Neustart erneut eingespielt. Muss NACH
// SD_Init() aufgerufen werden (Karte muss bereits gemountet sein); nutzt
// SD_Log() fuer die Diagnose, SD_Log_Init() sollte also vorher gelaufen sein.
// Bei erfolgreichem Einspielen startet das Geraet selbst per esp_restart()
// neu und kehrt nicht zurueck.
void SD_Update_CheckAndApply(void);
