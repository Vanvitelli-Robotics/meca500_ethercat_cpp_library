#include "CsvLoggerFeedback.hpp"

CsvLoggerFeedback::CsvLoggerFeedback(const std::string filename) :FILENAME(filename.c_str()){
    file.open(filename);
    if (!file.is_open()) {
        printf("impossibile aprire il file di log. esco...");
        exit(1);
    }
    file.precision(16);
    file << "x,y,z,alpha,beta,gamma,vel_x_des,vel_y_des,vel_z_des,vel_alpha_des,vel_beta_des,vel_gamma_des,vj_1,vj_2,vj_3,vj_4,vj_5,vj_6,th1,th2,th3,th4,th5,th6,\n";
}

CsvLoggerFeedback::~CsvLoggerFeedback() {
    flush();
    file.close();
}

void CsvLoggerFeedback::flush() {
    file.flush();
}

CsvLoggerFeedback& CsvLoggerFeedback::operator << (const double new_val) {
    file << std::scientific << new_val << ',';
    return *this;
}

void CsvLoggerFeedback::end_row() {
    file << '\n';
}

void CsvLoggerFeedback::close() {
    flush();
    file.close();
}