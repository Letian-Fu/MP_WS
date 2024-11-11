#ifndef GP_PLANNER_SDF_H
#define GP_PLANNER_SDF_H

#pragma once

#include "headers.h"
#include <fstream>
#include <sstream>
#include <stdexcept>

namespace gp_planner{

class SDFQueryOutOfRange : public std::runtime_error{
public:
    /// constructor
    SDFQueryOutOfRange() : std::runtime_error("Querying SDF out of range") {}
};

class SDF{
public:
    // index and float_index is <row, col, z>
    typedef boost::tuple<size_t, size_t, size_t> index;
    typedef boost::tuple<double, double, double> float_index;
    typedef boost::shared_ptr<SDF> shared_ptr;

private:
    gtsam::Point3 origin_;
    // geometry setting of signed distance field
    size_t field_rows_, field_cols_, field_z_;
    double cell_size_;
    // sdf data
    std::vector<gtsam::Matrix> data_;

public:
    /// constructor
    SDF() {}

    /// constructor with all data
    SDF(const gtsam::Point3& origin, double cell_size,
                        const std::vector<gtsam::Matrix>& data)
    :   origin_(origin),
        field_rows_(data[0].rows()),
        field_cols_(data[0].cols()),
        field_z_(data.size()),
        cell_size_(cell_size),
        data_(data) {}

    /// constructor with no data, insert the data later
    /// used by matlab wrapper
    SDF(const gtsam::Point3& origin, double cell_size,
                        size_t field_rows, size_t field_cols, size_t field_z)
    :   origin_(origin),
        field_rows_(field_rows),
        field_cols_(field_cols),
        field_z_(field_z),
        cell_size_(cell_size),
        data_(std::vector<gtsam::Matrix>(field_z)) {}

    ~SDF() = default;

    /// insert data matrix to each layer of sdf
    /// @param z_idx the z index of 3-D sdf
    /// @param field_layer matrix of each slice of 3-D sdf, Matrix represent the X
    /// (col) & Y (row)
    void initFieldData(size_t z_idx, const gtsam::Matrix& field_layer) {
        if (z_idx >= field_z_)
        throw std::runtime_error(
            "[SignedDistanceField] matrix layer out of index");
        data_[z_idx] = field_layer;
    }

    /// give a point, search for signed distance field and (optional) gradient
    /// @param point query position
    /// @return signed distance
    inline double getSignedDistance(const gtsam::Point3& point) const {
        const float_index pidx = convertPoint3toCell(point);
        return signed_distance(pidx);
    }

    /// give a point, search for signed distance field and (optional) gradient
    /// @param point query position
    /// @param g returned gradient reference
    /// @return signed distance
    inline double getSignedDistance(const gtsam::Point3& point,
                                    gtsam::Vector3& g) const {
        const float_index pidx = convertPoint3toCell(point);
        const gtsam::Vector3 g_idx = gradient(pidx);
        // convert gradient of index to gradient of metric unit
        g = gtsam::Vector3(g_idx(1), g_idx(0), g_idx(2)) / cell_size_;
        return signed_distance(pidx);
    }

    /// convert between point and cell corrdinate
    inline float_index convertPoint3toCell(const gtsam::Point3& point) const {
        // check point range
        if (point.x() < origin_.x() ||
            point.x() > (origin_.x() + (field_cols_ - 1.0) * cell_size_) ||
            point.y() < origin_.y() ||
            point.y() > (origin_.y() + (field_rows_ - 1.0) * cell_size_) ||
            point.z() < origin_.z() ||
            point.z() > (origin_.z() + (field_z_ - 1.0) * cell_size_)) {
        throw SDFQueryOutOfRange();
        }

        const double col = (point.x() - origin_.x()) / cell_size_;
        const double row = (point.y() - origin_.y()) / cell_size_;
        const double z = (point.z() - origin_.z()) / cell_size_;
        return boost::make_tuple(row, col, z);
    }

    inline gtsam::Point3 convertCelltoPoint3(const float_index& cell) const {
        return origin_ + gtsam::Point3(get<1>(cell) * cell_size_,
                                    get<0>(cell) * cell_size_,
                                    get<2>(cell) * cell_size_);
    }

    /// tri-linear interpolation
    inline double signed_distance(const float_index& idx) const {
        const double lr = floor(idx.get<0>()), lc = floor(idx.get<1>()),
                 lz = floor(idx.get<2>());
        const double hr = lr + 1.0, hc = lc + 1.0, hz = lz + 1.0;
        const size_t lri = static_cast<size_t>(lr), lci = static_cast<size_t>(lc),
                    lzi = static_cast<size_t>(lz), hri = static_cast<size_t>(hr),
                    hci = static_cast<size_t>(hc), hzi = static_cast<size_t>(hz);
        return (hr - idx.get<0>()) * (hc - idx.get<1>()) * (hz - idx.get<2>()) *
                signed_distance(lri, lci, lzi) +
            (idx.get<0>() - lr) * (hc - idx.get<1>()) * (hz - idx.get<2>()) *
                signed_distance(hri, lci, lzi) +
            (hr - idx.get<0>()) * (idx.get<1>() - lc) * (hz - idx.get<2>()) *
                signed_distance(lri, hci, lzi) +
            (idx.get<0>() - lr) * (idx.get<1>() - lc) * (hz - idx.get<2>()) *
                signed_distance(hri, hci, lzi) +
            (hr - idx.get<0>()) * (hc - idx.get<1>()) * (idx.get<2>() - lz) *
                signed_distance(lri, lci, hzi) +
            (idx.get<0>() - lr) * (hc - idx.get<1>()) * (idx.get<2>() - lz) *
                signed_distance(hri, lci, hzi) +
            (hr - idx.get<0>()) * (idx.get<1>() - lc) * (idx.get<2>() - lz) *
                signed_distance(lri, hci, hzi) +
            (idx.get<0>() - lr) * (idx.get<1>() - lc) * (idx.get<2>() - lz) *
                signed_distance(hri, hci, hzi);
    }

    /// gradient operator for tri-linear interpolation
    /// gradient regrads to float_index
    /// not differentiable at index point
    inline gtsam::Vector3 gradient(const float_index& idx) const {
        const double lr = floor(idx.get<0>()), lc = floor(idx.get<1>()),
                 lz = floor(idx.get<2>());
        const double hr = lr + 1.0, hc = lc + 1.0, hz = lz + 1.0;
        const size_t lri = static_cast<size_t>(lr), lci = static_cast<size_t>(lc),
                    lzi = static_cast<size_t>(lz), hri = static_cast<size_t>(hr),
                    hci = static_cast<size_t>(hc), hzi = static_cast<size_t>(hz);
        return gtsam::Vector3((hc - idx.get<1>()) * (hz - idx.get<2>()) *
                                    (signed_distance(hri, lci, lzi) -
                                    signed_distance(lri, lci, lzi)) +
                                (idx.get<1>() - lc) * (hz - idx.get<2>()) *
                                    (signed_distance(hri, hci, lzi) -
                                    signed_distance(lri, hci, lzi)) +
                                (hc - idx.get<1>()) * (idx.get<2>() - lz) *
                                    (signed_distance(hri, lci, hzi) -
                                    signed_distance(lri, lci, hzi)) +
                                (idx.get<1>() - lc) * (idx.get<2>() - lz) *
                                    (signed_distance(hri, hci, hzi) -
                                    signed_distance(lri, hci, hzi)),

                            (hr - idx.get<0>()) * (hz - idx.get<2>()) *
                                    (signed_distance(lri, hci, lzi) -
                                    signed_distance(lri, lci, lzi)) +
                                (idx.get<0>() - lr) * (hz - idx.get<2>()) *
                                    (signed_distance(hri, hci, lzi) -
                                    signed_distance(hri, lci, lzi)) +
                                (hr - idx.get<0>()) * (idx.get<2>() - lz) *
                                    (signed_distance(lri, hci, hzi) -
                                    signed_distance(lri, lci, hzi)) +
                                (idx.get<0>() - lr) * (idx.get<2>() - lz) *
                                    (signed_distance(hri, hci, hzi) -
                                    signed_distance(hri, lci, hzi)),

                            (hr - idx.get<0>()) * (hc - idx.get<1>()) *
                                    (signed_distance(lri, lci, hzi) -
                                    signed_distance(lri, lci, lzi)) +
                                (idx.get<0>() - lr) * (hc - idx.get<1>()) *
                                    (signed_distance(hri, lci, hzi) -
                                    signed_distance(hri, lci, lzi)) +
                                (hr - idx.get<0>()) * (idx.get<1>() - lc) *
                                    (signed_distance(lri, hci, hzi) -
                                    signed_distance(lri, hci, lzi)) +
                                (idx.get<0>() - lr) * (idx.get<1>() - lc) *
                                    (signed_distance(hri, hci, hzi) -
                                    signed_distance(hri, hci, lzi)));
    }

    /// access
    inline double signed_distance(size_t r, size_t c, size_t z) const {
        return data_[z](r, c);
    }

    const gtsam::Point3& origin() const { return origin_; }
    size_t x_count() const { return field_cols_; }
    size_t y_count() const { return field_rows_; }
    size_t z_count() const { return field_z_; }
    double cell_size() const { return cell_size_; }
    const std::vector<gtsam::Matrix>& raw_data() const { return data_; }

    /// print
    void print(const std::string& str = "") const {
        std::cout << str;
        std::cout << "field origin:     " << origin_.transpose() << std::endl;
        std::cout << "field resolution: " << cell_size_ << std::endl;
        std::cout << "field size:       " << field_cols_ << " x " << field_rows_
                << " x " << field_z_ << std::endl;
    }

    /// save to file
    void saveSDF(const std::string& filename){
        std::string fext = filename.substr(filename.find_last_of(".") + 1);
        std::ofstream ofs(filename);
        if (!ofs) {
            throw std::runtime_error("Unable to open file for writing: " + filename);
        }
        if(fext == "bin"){
            ofs.open(filename, std::ios::binary); // Ensure binary mode for binary files
            // 写入 origin_
            ofs.write(reinterpret_cast<const char*>(&origin_.x()), sizeof(double));
            ofs.write(reinterpret_cast<const char*>(&origin_.y()), sizeof(double));
            ofs.write(reinterpret_cast<const char*>(&origin_.z()), sizeof(double));

            // 写入 field_rows_, field_cols_, field_z_
            ofs.write(reinterpret_cast<const char*>(&field_rows_), sizeof(size_t));
            ofs.write(reinterpret_cast<const char*>(&field_cols_), sizeof(size_t));
            ofs.write(reinterpret_cast<const char*>(&field_z_), sizeof(size_t));

            // 写入 cell_size_
            ofs.write(reinterpret_cast<const char*>(&cell_size_), sizeof(double));

            // 写入 data_ 的大小
            size_t dataSize = data_.size();
            ofs.write(reinterpret_cast<const char*>(&dataSize), sizeof(size_t));

            // 写入每个 gtsam::Matrix
            for (const auto& matrix : data_) {
                // 写入矩阵的行数和列数
                size_t rows = matrix.rows();
                size_t cols = matrix.cols();
                ofs.write(reinterpret_cast<const char*>(&rows), sizeof(size_t));
                ofs.write(reinterpret_cast<const char*>(&cols), sizeof(size_t));

                // 写入矩阵数据
                for (size_t i = 0; i < rows; ++i) {
                    for (size_t j = 0; j < cols; ++j) {
                        ofs.write(reinterpret_cast<const char*>(&matrix(i, j)), sizeof(double));
                    }
                }
            }
        }
        if (fext == "txt"){
            // 将 origin_ 转换为文本并写入文件
            ofs << "Origin: " << origin_.x() << " " << origin_.y() << " " << origin_.z() << std::endl;

            // 写入 field_rows_, field_cols_, field_z_ 和 cell_size_
            ofs << "FieldRows: " << field_rows_ << std::endl;
            ofs << "FieldCols: " << field_cols_ << std::endl;
            ofs << "FieldZ: " << field_z_ << std::endl;
            ofs << "CellSize: " << cell_size_ << std::endl;

            // 将每个 gtsam::Matrix 转换为文本并写入文件
            for (size_t z = 0; z < data_.size(); ++z) {
                for (size_t i = 0; i < data_[z].rows(); ++i) {
                    for (size_t j = 0; j < data_[z].cols(); ++j) {
                        ofs << data_[z](i, j) << (j < data_[z].cols() - 1 ? " " : "");
                    }
                    ofs << (i < data_[z].rows() - 1 ? "\n" : "");
                }
                ofs << (z < data_.size() - 1 ? "\n\n" : "");
            }
        }
        else {
            throw std::runtime_error("Unsupported file extension: " + fext);
        }
    }

    /// load from file
    bool loadSDF(const std::string& filename){
        std::string fext = filename.substr(filename.find_last_of(".") + 1);
        std::ifstream ifs(filename);
        if (!ifs) {
            throw std::runtime_error("Unable to open file for reading: " + filename);
        }

        if (fext == "bin") {
            ifs.open(filename, std::ios::binary); // Ensure binary mode for binary files
            if (!ifs) {
                throw std::runtime_error("Unable to open file for reading: " + filename);
            }

            // 读取 origin_
            double x, y, z;
            ifs.read(reinterpret_cast<char*>(&x), sizeof(double));
            ifs.read(reinterpret_cast<char*>(&y), sizeof(double));
            ifs.read(reinterpret_cast<char*>(&z), sizeof(double));
            origin_ = gtsam::Point3(x, y, z);

            // 读取 field_rows_, field_cols_, field_z_
            ifs.read(reinterpret_cast<char*>(&field_rows_), sizeof(size_t));
            ifs.read(reinterpret_cast<char*>(&field_cols_), sizeof(size_t));
            ifs.read(reinterpret_cast<char*>(&field_z_), sizeof(size_t));

            // 读取 cell_size_
            ifs.read(reinterpret_cast<char*>(&cell_size_), sizeof(double));

            // 读取 data_ 的大小
            size_t dataSize;
            ifs.read(reinterpret_cast<char*>(&dataSize), sizeof(size_t));
            data_.resize(dataSize);

            // 读取每个 gtsam::Matrix
            for (auto& matrix : data_) {
                // 读取矩阵的行数和列数
                size_t rows, cols;
                ifs.read(reinterpret_cast<char*>(&rows), sizeof(size_t));
                ifs.read(reinterpret_cast<char*>(&cols), sizeof(size_t));
                matrix.resize(rows, cols);

                // 读取矩阵数据
                for (size_t i = 0; i < rows; ++i) {
                    for (size_t j = 0; j < cols; ++j) {
                        ifs.read(reinterpret_cast<char*>(&matrix(i, j)), sizeof(double));
                    }
                }
            }
        } else if (fext == "txt") {
            // 从文件中读取 origin_
            std::string line;
            std::string label;
            std::getline(ifs, line);
            // std::istringstream iss_origin(line.substr(line.find(": ") + 2));
            std::istringstream iss_origin(line);
            iss_origin>>label;
            if (!(iss_origin >> origin_.x() >> origin_.y() >> origin_.z())) {
                throw std::runtime_error("Failed to parse origin");
            }

            // 从文件中读取 field_rows_, field_cols_, field_z_ 和 cell_size_
            std::getline(ifs, line); // Read "FieldRows: n"
            std::istringstream iss_row(line);
            iss_row>>label;
            if (!(iss_row >> field_rows_)) {
                throw std::runtime_error("Failed to parse field_rows");
            }

            std::getline(ifs, line); // Read "FieldCols: n"
            std::istringstream iss_col(line);
            iss_col>>label;
            if (!(iss_col >> field_cols_)) {
                throw std::runtime_error("Failed to parse field_cols");
            }

            std::getline(ifs, line); // Read "FieldZ: n"
            std::istringstream iss_z(line);
            iss_z>>label;
            if (!(iss_z >> field_z_)) {
                throw std::runtime_error("Failed to parse field_z");
            }

            std::getline(ifs, line); // Read "CellSize: n"
            std::istringstream iss_cell(line);
            iss_cell>>label;
            if (!(iss_cell >> cell_size_)) {
                throw std::runtime_error("Failed to parse cell_size");
            }

            // // 从文件中读取每个 gtsam::Matrix
            data_.resize(field_z_);
            for (size_t z = 0; z < field_z_; ++z) {
                gtsam::Matrix& matrix = data_[z];
                matrix.resize(field_rows_, field_cols_);
                for (int i = 0; i < field_rows_; ++i) {
                    std::getline(ifs, line);
                    while(line.empty())    std::getline(ifs, line);
                    std::istringstream iss_row(line);
                    for (int j = 0; j < field_cols_; ++j) {
                        iss_row >> matrix(i, j);
                    }
                }
            }
        } else {
            throw std::runtime_error("Unsupported file extension: " + fext);
        }
        return true;
    }
};
    
}   //namespace gp_planner

#endif //GP_PLANNER_SIGNEDDISTANCEFIELD_H