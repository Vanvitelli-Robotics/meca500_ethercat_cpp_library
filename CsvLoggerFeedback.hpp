#ifndef CSV_LOGGER_H
#define CSV_LOGGER_H

#include <iostream>
#include <fstream>

class CsvLoggerFeedback {
    private:
        const char* FILENAME;
        std::ofstream file;
    public:
        CsvLoggerFeedback(const std::string filename);
        ~CsvLoggerFeedback();
        void flush();
        CsvLoggerFeedback& operator << (const double new_val);
        void end_row();
        void close();
};

#endif