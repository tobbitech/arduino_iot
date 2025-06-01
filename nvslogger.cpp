// #include "nvslogger.h"
// #include <Arduino.h>
// #include <ArduinoNvs.h>

// ArduinoNvs nvs_log;

// NvsLogger::NvsLogger(String log_namespace)
// {
//     nvs_log.begin(log_namespace);
// }

// void NvsLogger::log(String entry, String timestamp)
// {
//     String time_entry = "Uptime " + String(round(millis() / 1000.0), 0) + "s";
    
//     if (timestamp != "none") {
//         // custom timestamp added
//         time_entry = time_entry + " " + timestamp;
//     }
       
//     time_entry = time_entry + ": " + entry;
    
//     uint16_t log_index = nvs_log.getInt("log_index");
//     log_index = (log_index + 1) % NVS_LOG_LENGTH;
//     String key = NvsLogger::_make_nvs_key(log_index);
//     nvs_log.setString(key, time_entry);
//     nvs_log.setInt("log_index", log_index);
// }

// String NvsLogger::_make_nvs_key(uint16_t index) {
//     return( NVS_LOG_PREFIX + String(index) );
// }

// String NvsLogger::get_log_entry_by_index(uint16_t index) {
//     String key = NvsLogger::_make_nvs_key(index);
//     return( nvs_log.getString(key) );
// }

// String NvsLogger::get_newest_log_entry(uint16_t n_newest) {
//     uint16_t log_index = nvs_log.getInt("log_index");
//     uint16_t index = (NVS_LOG_LENGTH + (log_index - n_newest)) % NVS_LOG_LENGTH;
//     String entry = NvsLogger::get_log_entry_by_index(index);
//     return(entry);
// }

// void NvsLogger::print_log_to_serial(uint16_t n_entries) {
//     if (n_entries == 0 || n_entries > NVS_LOG_LENGTH) {
//         // print all entries
//         n_entries = NVS_LOG_LENGTH;
//     }

//     if(!Serial) {
//         NvsLogger::log("Cannot print log to serial");
//         return;
//     }
//     Serial.println("Dumping log:");

//     // Newest log entry first
//     for (int i = n_entries; i >= 0; i--) {
//         Serial.println( NvsLogger::get_newest_log_entry(i) );
//     }
// }


