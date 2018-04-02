// Copyright (c) 2017 Baidu Inc. All Rights Reserved.

// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at

//     http://www.apache.org/licenses/LICENSE-2.0

// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once
#include <cmath>
#include <boost/python.hpp>
#include <boost/tuple/tuple.hpp>

#include "simulator_util.h"

namespace simulator {

class Vec3 {
  public:
    double x;
    double y;
    double z;
    static constexpr double EPS = 1e-3;

    // Note: min() is the minimum positive number
    Vec3() : x(-std::numeric_limits<double>::max()) {
        y = x;
        z = x;
    }

    Vec3(double x_, double y_, double z_) : x(x_), y(y_), z(z_) {}

    Vec3 operator-(const Vec3& l) const {
        return Vec3(x - l.x, y - l.y, z - l.z);
    }

    Vec3 operator+(const Vec3& l) const {
        return Vec3(x + l.x, y + l.y, z + l.z);
    }

    Vec3 operator*(double c) const {
        return Vec3(x * c, y * c, z * c);
    }

    bool operator==(const Vec3& l) const { return square_distance(l) < EPS; }

    double L2_norm() const { return x * x + y * y + z * z; }

    double square_distance(const Vec3& l) const {
        Vec3 diff = (*this) - l;
        return diff.L2_norm();
    }

    bool defined() const {
        return (x > std::numeric_limits<int>::min() +  EPS) &&
               (y > std::numeric_limits<int>::min() +  EPS) &&
               (z > std::numeric_limits<int>::min() +  EPS);
    }

    void random_loc(int w, int h) {
        x = simulator::util::get_rand_ind(w);
        y = simulator::util::get_rand_ind(h);
        z = 0;
    }

    bool in_boundary(int w, int h) const {
        return x >= 0 && x < w && y >= 0 && y < h;
    }

    void scale(double s) {
        x *= s;
        y *= s;
        z *= s;
    }
};

/*
  A world consists of many entities.
  An entity might be an agent, an object, a landmark, etc.
  This is the general data structure for representing a thing in different games
 */
struct Entity {
    Entity() {}

    Entity(boost::python::dict e) {
        const boost::python::tuple& l = boost::python::extract<boost::python::tuple>(e["loc"]);
        type = boost::python::extract<std::string>(e["type"]);
        id = boost::python::extract<std::string>(e["id"]);
        loc = Vec3(boost::python::extract<double>(l[0]),
                   boost::python::extract<double>(l[1]),
                   boost::python::extract<double>(l[2]));
        yaw = boost::python::extract<double>(e["yaw"]);
        scale = boost::python::extract<double>(e["scale"]);
        offset = boost::python::extract<double>(e["offset"]);
        name = boost::python::extract<std::string>(e["name"]);
        asset_path = boost::python::extract<std::string>(e["asset_path"]);
        color = boost::python::extract<std::string>(e["color"]);
        CHECK(offset >= 0 && offset <= 1 - scale);
        auto endswith = [](std::string str, std::string suffix) -> bool {
                            size_t p = str.rfind(suffix);
                            return (p != std::string::npos &&
                                    p == str.length() - suffix.length());
                        };
        if (endswith(asset_path, ".xml")) {
            model_type = "mjcf";
        } else if (endswith(asset_path, ".sdf")) {
            model_type = "sdf";
        } else if (endswith(asset_path, ".urdf")) {
            model_type = "urdf";
        } else {
            LOG(FATAL) << "unknown model file type";
        }
    }

    boost::python::dict to_py_dict() const {
        boost::python::dict d;
        d["type"] = type;
        // python end is unaware of attribute "model_type"
        d["id"] = id;
        d["loc"] = boost::python::make_tuple(loc.x, loc.y, loc.z);
        d["yaw"] = yaw;
        d["scale"] = scale;
        d["offset"] = offset;
        d["name"] = name;
        d["asset_path"] = asset_path;
        d["color"] = color;
        return d;
    }
    std::string type;
    std::string id; // unique identifier of the object instance

    std::string model_type; // type of model conf file [udrf|mjcf]
                            // this is only used for cpp to determine which
                            // loading function to call; python end is unaware
                            // of it
    Vec3 loc;
    double yaw; // the heading orientation of the object
    double scale;  // the scale to be rendered
    double offset; // the offset in a grid
    std::string name; // class name of the object without id
    std::string asset_path;  // the actual asset path on the disk (icon or 3d model)
    std::string color;  // mainly used by python scripts
};

}  // namespace simulator
