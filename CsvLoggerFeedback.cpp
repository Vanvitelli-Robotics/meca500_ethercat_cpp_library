#include "CsvLoggerFeedback.hpp"

//#define LOGGING_DISABLED TRUE

CsvLoggerFeedback::CsvLoggerFeedback(const std::string filename) : FILENAME(filename.c_str())
{
    file.open(filename);
    if (!file.is_open())
    {
        printf("impossibile aprire il file di log. esco...");
        exit(1);
    }
    file.precision(16);
    file << "x,y,z,alpha,beta,gamma,vel_x_des,vel_y_des,vel_z_des,vel_alpha_des,vel_beta_des,vel_gamma_des,vj_1,vj_2,vj_3,vj_4,vj_5,vj_6,th1,th2,th3,th4,th5,th6,out_of_range,\n";
}

CsvLoggerFeedback::~CsvLoggerFeedback()
{
    flush();
    file.close();
}

void CsvLoggerFeedback::flush()
{
    file.flush();
}

CsvLoggerFeedback &CsvLoggerFeedback::operator<<(const double new_val)
{
#ifndef LOGGING_DISABLED
    file << std::scientific << new_val << ',';
#endif
    return *this;
}

void CsvLoggerFeedback::end_row()
{
#ifndef LOGGING_DISABLED
    file << '\n';
#endif
}

void CsvLoggerFeedback::close()
{
    flush();
    file.close();
}
