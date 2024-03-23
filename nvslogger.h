#ifndef NVSLOGGER_H
#define NVSLOGGER_H

#define NVS_LOG_PREFIX "log_"
#define NVS_LOG_LENGTH 100


#include <Arduino.h>

class NvsLogger
{
    public:
        NvsLogger(String log_namespace = "log");
        void log(String entry, String timestamp = "none");
        void print_log_to_serial(uint16_t n_entries = 0);
        void get_log_entry(uint16_t index);
        String get_log_entry_by_index(uint16_t index);
        String get_newest_log_entry(uint16_t n_newest = 0);

    private:
        // uint16_t _current_nvs_log_index;
        String _make_nvs_key(uint16_t index);
};


#endif