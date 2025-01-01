#include <Eigen/Dense>
#include <fstream>

#define MAXBUFSIZE  ((int)1e6)

inline Eigen::MatrixXd readMatrix(const char * filename) {
    int cols = 0, rows = 0;
    double buff[MAXBUFSIZE] = {0};

    // Read numbers from file into buffer.
    std::ifstream infile;
    infile.open(filename);
    while (! infile.eof())
    {
        std::string line;
        std::getline(infile, line);

        int temp_cols = 0;
        std::stringstream stream(line);
        while(! stream.eof()){
            stream >> buff[cols*rows+temp_cols++];
        }

        if (temp_cols == 0)
            continue;

        if (cols == 0)
            cols = temp_cols;
        if (temp_cols>=5) {
            rows++;
        }
    }

    infile.close();

    // Populate matrix with numbers.
    Eigen::MatrixXd result(rows,cols);
    for (int i = 0; i < rows; i++)
        for (int j = 0; j < cols; j++)
            result(i,j) = buff[ cols*i+j ];

    return result;
};


template <typename T>
inline T lowpass_filter(T input, double Ts, T outputLast, double dt) {
    double alpha = dt / (Ts + dt);
    return outputLast + (alpha * (input - outputLast));
}

