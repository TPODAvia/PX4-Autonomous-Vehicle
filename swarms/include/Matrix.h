#include <vector>

class Matrix {
public:
    Matrix(int rows, int columns);
    ~Matrix();
    
    void setElement(int row, int column, double value);
    double getElement(int row, int column) const;
    
private:
    int m_rows;
    int m_columns;
    std::vector<std::vector<double>> m_data;
};

Matrix::Matrix(int rows, int columns)
    : m_rows(rows), m_columns(columns), m_data(rows, std::vector<double>(columns)) {
}

Matrix::~Matrix() {
}

void Matrix::setElement(int row, int column, double value) {
    m_data[row][column] = value;
}

double Matrix::getElement(int row, int column) const {
    return m_data[row][column];
}
