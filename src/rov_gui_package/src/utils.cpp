#include <rov_gui_package/utils.h>

#include <math.h>
#include <stdexcept>
#include <functional>

#include <QFile>
#include <QTextStream>
#include <QStringList>

namespace {
QVector<QVector3D> base_pars(const QString &path,
                             std::function<void(QTextStream&, QVector<QVector3D>&)> parser)
{
    QVector<QVector3D> vertexses;

    QFile obj_file(path);

    if(!obj_file.exists()) {
        throw std::runtime_error("file not exsist: " + path.toStdString());
    }

    obj_file.open(QIODevice::ReadOnly);
    QTextStream input(&obj_file);
    parser(input, vertexses);

    return vertexses;
}

EulerKrylovAngles to_radians(const EulerKrylovAngles &angles) {
    constexpr float koefficient = M_PI / 180;

    EulerKrylovAngles radian_angles;

    radian_angles.cource = angles.cource * koefficient;
    radian_angles.roll = angles.roll * koefficient;
    radian_angles.pitch = angles.pitch * koefficient;

    return radian_angles;
}

}

QVector<QVector3D> parse_CSV(const QString &path)
{
    auto parser = [](QTextStream& input, QVector<QVector3D>& vertexses) {
        while(!input.atEnd()) {
            QString line = input.readLine();

            if(line.isEmpty() || line[0] == '#') {
                continue;
            }

            QStringList string_list = line.split(',');

            if(string_list.empty()) {
                continue;
            }

            if(!(string_list.size() == 3)) {
                throw std::invalid_argument("parse_CSV: incorrect line value, must be 3");
            }

            vertexses.push_back({string_list[0].toFloat(), string_list[1].toFloat(),
                                 string_list[2].toFloat()});
        }
    };

    return base_pars(path, parser);
}

QVector<QVector3D> parse_OBJ(const QString &path)
{
    auto parser = [](QTextStream& input, QVector<QVector3D>& vertexses) {
        QVector<QVector3D> coords;

        while(!input.atEnd()) {
            QString line = input.readLine();
            QStringList string_list = line.split(' ');

            if(string_list.empty() || string_list[0] == '#' ||
               string_list[0] == "vt" || string_list[0] == "vn" ||
               string_list[0] == "mtllib" || string_list[0] == "o" ||
               string_list[0] == "usemtl" || string_list[0] == "s") {
                continue;
            }

            if(!(string_list.size() == 4)) {
                throw std::invalid_argument("parse_OBJ: incorrect line value, must be 3");
            }

            if(string_list[0] == "v") {
                coords.push_back({string_list[1].toFloat(), string_list[2].toFloat(),
                                  string_list[3].toFloat()});
            } else if(string_list[0] == "f") {
                if(!(string_list.size() == 4)) {
                    throw std::invalid_argument("parse_OBJ: incorrect line value in f"
                                                ", must be 3");
                }

                for(int i = 1; i <= 3; ++i) {
                    QStringList vert = string_list[i].split('/');
                    vertexses.push_back({coords.at(vert[0].toLong() - 1)});
                }
            }
        }
    };

    return base_pars(path, parser);
}

EulerKrylovAngles convert_to_angles(const QQuaternion &quaternion)
{
    return EulerKrylovAngles();
}

QQuaternion convert_to_quaternion(const EulerKrylovAngles &angles)
{
    auto radian_angles = to_radians(angles);

    double L0 = std::cos(radian_angles.roll / 2) * std::cos(radian_angles.pitch / 2) * std::cos(radian_angles.cource / 2) -
            std::sin(radian_angles.roll / 2) * std::sin(radian_angles.pitch / 2) * std::sin(radian_angles.cource / 2);

    double L1 = std::cos(radian_angles.roll / 2) * std::sin(radian_angles.pitch / 2) * std::sin(radian_angles.cource / 2) +
            std::sin(radian_angles.roll / 2) * std::cos(radian_angles.pitch / 2) * std::cos(radian_angles.cource / 2);

    double L2 = std::cos(radian_angles.roll / 2) * std::cos(radian_angles.pitch / 2) * std::sin(radian_angles.cource / 2) +
            std::sin(radian_angles.roll / 2) * std::sin(radian_angles.pitch / 2) * std::cos(radian_angles.cource / 2);

    double L3 = std::cos(radian_angles.roll / 2) * std::sin(radian_angles.pitch / 2) * std::cos(radian_angles.cource / 2) -
            std::sin(radian_angles.roll / 2) * std::cos(radian_angles.pitch / 2) * std::sin(radian_angles.cource / 2);

    return QQuaternion(L0, QVector3D(L1, L2, L3));
}