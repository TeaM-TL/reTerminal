#ifndef STACJA_H
#define STACJA_H

// ==== delays ====
const unsigned long buzzerPeriod = 900000UL;  // 15 min
const unsigned long fullRefreshPeriod = 3600000UL; // 60 min
const unsigned long weatherRefreshPeriod = 1800000UL; // 30 min
const unsigned long sensorRefreshPeriod = 60000UL; // 1 min
const unsigned long batteryRefreshPeriod = 600000UL; // 10 min

// ==== wind unit ====
int windUnit = 1;   // 0 = km/h, 1 = knot

// ===== NTP servers =====
const char* ntpServer = "pl.pool.ntp.org";

#endif