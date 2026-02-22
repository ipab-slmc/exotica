//
// Copyright (c) 2019
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//  * Redistributions of source code must retain the above copyright notice,
//    this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the
//    documentation and/or other materials provided with the distribution.
//  * Neither the name of  nor the names of its contributors may be used to
//    endorse or promote products derived from this software without specific
//    prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//

#ifndef EXOTICA_CORE_VISUALIZATION_MESHCAT_TYPES_H_
#define EXOTICA_CORE_VISUALIZATION_MESHCAT_TYPES_H_

#define MSGPACK_USE_DEFINE_MAP

#include <iostream>
#include <msgpack.hpp>

#include <geometric_shapes/shapes.h>

#include <exotica_core/scene.h>
#include <exotica_core/tools.h>

// Backwards compatibility switch 16.04 with meshpack version < 2.0.0
#if (MSGPACK_VERSION_MAJOR >= 2)
typedef msgpack::v1::type::raw_ref msgpack_raw_ref;
typedef msgpack::v1::type::ext msgpack_ext;
#else
typedef msgpack::type::raw_ref msgpack_raw_ref;
typedef std::vector<float> msgpack_ext;
#define MSGPACK_FEATURE_NOT_SUPPORTED
#endif

namespace exotica
{
namespace visualization
{
inline long RGB(double R, double G, double B)
{
    return static_cast<long>(std::min(std::max(R, 0.0), 1.0) * 255) * 65536L + static_cast<long>(std::min(std::max(G, 0.0), 1.0) * 255) * 256L + static_cast<long>(std::min(std::max(B, 0.0), 1.0) * 255);
}

unsigned char random_char()
{
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> dis(0, 255);
    return static_cast<unsigned char>(dis(gen));
}

std::string generate_hex(const unsigned int len)
{
    std::stringstream ss;
    for (auto i = 0; i < len; ++i)
    {
        auto rc = random_char();
        std::stringstream hexstream;
        hexstream << std::hex << int(rc);
        auto hex = hexstream.str();
        ss << (hex.length() < 2 ? '0' + hex : hex);
    }
    return ss.str();
}

std::string generate_uuid()
{
    return generate_hex(4) + "-" + generate_hex(4) + "-" + generate_hex(4) + "-" + generate_hex(4);
}

struct Base
{
    Base() = default;
    Base(const std::string& type_in, const std::string& path_in) : type(type_in), path(path_in){};
    std::string type;
    std::string path;
};

template <typename T>
struct Property : Base
{
    Property() : Base("set_property", ""){};
    Property(const std::string& path_in, const std::string& property_in, const T& value_in) : Base("set_property", path_in), property(property_in), value(value_in){};
    std::string property;
    T value;
    MSGPACK_DEFINE(type, path, property, value);
};

struct MetaData
{
    MetaData() = default;
    MetaData(double version_in, const std::string& type_in) : version(version_in), type(type_in){};
    double version;
    std::string type;
    MSGPACK_DEFINE(version, type);
};

struct Material
{
    Material(long color_in, double opacity_in = 1.0, const std::string& type_in = "MeshPhongMaterial", const std::string& uuid_in = "") : color(color_in), opacity(opacity_in), uuid(uuid_in), type(type_in)
    {
        if (uuid_in == "") uuid = generate_uuid();
    };
    Material()
    {
        uuid = "0000-0000-0000-0000";  // Default material UUID
    };
    std::string uuid = "";
    std::string type = "MeshPhongMaterial";
    long color = 16777215;
    long ambient = 0;
    long emissive = 0;
    long specular = 1118481;
    double shininess = 30.0;
    double opacity = 1.0;
    bool transparent = false;
    bool wireframe = false;
    MSGPACK_DEFINE(uuid, type, color, ambient, emissive, specular, shininess, opacity, transparent, wireframe);
};

struct ObjectData
{
    ObjectData() = default;
    ObjectData(const std::string& type_in, const std::string& uuid_in,
               const std::string& geometry_in, const std::string& material_in) : type(type_in), uuid(uuid_in), geometry(geometry_in), material(material_in){};
    std::string type;
    std::string uuid;
    std::string geometry;
    std::string material;
    std::vector<double> matrix = {1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0};
    MSGPACK_DEFINE(type, uuid, geometry, material, matrix);
};

template <typename T>
struct Object
{
    MetaData metadata = MetaData(4.5, "Object");
    ObjectData object;
    std::vector<Material> materials;
    std::vector<T> geometries;
    MSGPACK_DEFINE(metadata, geometries, materials, object);
};

template <typename T>
struct MeshObject
{
    MetaData metadata = MetaData(4.5, "Object");
    T object;
    std::vector<Material> materials;
    std::vector<T> geometries;
    MSGPACK_DEFINE(metadata, geometries, materials, object);
};

template <typename T>
struct SetObjectType
{
    SetObjectType() = default;
    SetObjectType(const std::string& path_in, const T& object_in) : type("set_object"), path(path_in), object(object_in){};
    std::string type;
    std::string path;
    T object;
    MSGPACK_DEFINE(type, path, object);
};

template <typename T>
SetObjectType<T> SetObject(const std::string& path_in, const T& object_in)
{
    return SetObjectType<T>(path_in, object_in);
};

struct Geometry
{
    Geometry() = default;
    Geometry(const std::string& type_in, const std::string& uuid_in = "") : type(type_in), uuid(uuid_in)
    {
        if (uuid_in == "") uuid = generate_uuid();
    };
    std::string uuid;
    std::string type;
};

struct GeometryBox : Geometry
{
    GeometryBox() : Geometry("BoxGeometry", ""){};
    GeometryBox(double width_in, double height_in, double depth_in, const std::string& uuid_in = "") : Geometry("BoxGeometry", uuid_in), width(width_in), height(height_in), depth(depth_in){};
    double width;
    double height;
    double depth;
    MSGPACK_DEFINE(uuid, type, width, height, depth)
};

struct GeometryCylinder : Geometry
{
    GeometryCylinder() : Geometry("CylinderGeometry", ""){};
    GeometryCylinder(double radius_in, double height_in, int radialSegments_in = 50, const std::string& uuid_in = "") : Geometry("CylinderGeometry", uuid_in), radiusTop(radius_in), radiusBottom(radius_in), height(height_in), radialSegments(radialSegments_in){};
    double height;
    double radiusTop;
    double radiusBottom;
    int radialSegments;
    MSGPACK_DEFINE(uuid, type, height, radiusTop, radiusBottom, radialSegments)
};

struct GeometrySphere : Geometry
{
    GeometrySphere() : Geometry("SphereGeometry", ""){};
    GeometrySphere(double radius_in, int widthSegments_in = 50, int heightSegments_in = 50, const std::string& uuid_in = "") : Geometry("SphereGeometry", uuid_in), radius(radius_in), heightSegments(heightSegments_in), widthSegments(widthSegments_in){};
    double radius;
    int widthSegments;
    int heightSegments;
    MSGPACK_DEFINE(uuid, type, radius, widthSegments, heightSegments)
};

struct GeometryMesh : Geometry
{
    GeometryMesh() : Geometry("_meshfile_geometry", ""){};

    GeometryMesh(const std::string& file_name_in, const std::string& url_in = "", const std::string& format_in = "", const std::string& uuid_in = "") : Geometry("_meshfile_geometry", uuid_in), format(format_in)
    {
        file_name = ParsePath(file_name_in);
        if (format_in == "")
        {
            std::string::size_type pos = file_name.rfind(".");
            if (pos != std::string::npos)
            {
                format = file_name.substr(pos + 1);
            }
        }
        std::transform(format.begin(), format.end(), format.begin(), ::tolower);
        url = ParsePath(url_in);
        std::ifstream file(file_name, std::ios::binary | std::ios::ate);
        std::streamsize size = file.tellg();
        file.seekg(0, std::ios::beg);

        buffer.resize(size);
        file.read(buffer.data(), size);

        data.ptr = buffer.data();
        data.size = buffer.size();
    }

    GeometryMesh(const GeometryMesh& other) : Geometry(other.type, other.uuid)
    {
        buffer = other.buffer;
        url = other.url;
        resources = other.resources;
        matrix = other.matrix;
        format = other.format;
        data.ptr = buffer.data();
        data.size = buffer.size();
    }

    GeometryMesh& operator=(const GeometryMesh& other)
    {
        if (this != &other)
        {
            type = other.type;
            uuid = other.uuid;
            buffer = other.buffer;
            url = other.url;
            resources = other.resources;
            matrix = other.matrix;
            format = other.format;
            data.ptr = buffer.data();
            data.size = buffer.size();
        }
        return *this;
    }

    std::vector<char> buffer;
    std::string file_name;
    std::string format;
    msgpack_raw_ref data;
    std::string url;
    std::map<std::string, std::string> resources;
    std::vector<double> matrix = {1.0, 0.0, 0.0, 0.0,
                                  0.0, 1.0, 0.0, 0.0,
                                  0.0, 0.0, 1.0, 0.0,
                                  0.0, 0.0, 0.0, 1.0};
    MSGPACK_DEFINE(uuid, type, format, resources, url, data, matrix)
};

struct ArrayFloat
{
    ArrayFloat() = default;
    ~ArrayFloat() = default;

    ArrayFloat(double* data_in, unsigned int size)
    {
        data.resize(size);
        for (unsigned int i = 0; i < size; ++i)
            data[i] = static_cast<float>(data_in[i]);
#ifdef MSGPACK_FEATURE_NOT_SUPPORTED
        array = data;
        WARNING("MSGPACK version does not support sending this type of data. Ignoring.");
#else
        array = msgpack_ext(0x17, reinterpret_cast<const char*>(data.data()), sizeof(float) * data.size());
#endif
    }

    ArrayFloat(const ArrayFloat& other)
    {
        itemSize = other.itemSize;
        normalized = other.normalized;
        data = other.data;
#ifdef MSGPACK_FEATURE_NOT_SUPPORTED
        array = data;
#else
        array = msgpack_ext(0x17, reinterpret_cast<const char*>(data.data()), sizeof(float) * data.size());
#endif
    }

    int itemSize = 3;
    std::string type = "Float32Array";
    bool normalized = false;
    std::vector<float> data;
    msgpack_ext array;

    MSGPACK_DEFINE(itemSize, type, normalized, array)
};

struct ArrayInt
{
    ArrayInt() = default;

    ArrayInt(unsigned int* data, unsigned int size)
    {
        array.resize(size);
        for (unsigned int i = 0; i < size; ++i)
            array[i] = data[i];
    }

    int itemSize = 3;
    std::string type = "Uint32Array";
    bool normalized = false;
    std::vector<uint32_t> array;

    MSGPACK_DEFINE(itemSize, type, normalized, array)
};

struct GeometryMeshBufferData
{
    GeometryMeshBufferData() = default;
    GeometryMeshBufferData(shapes::ShapePtr shape_in)
    {
        std::shared_ptr<shapes::Mesh> shape = std::static_pointer_cast<shapes::Mesh>(shape_in);
        attributes.insert(std::make_pair<std::string, ArrayFloat>("position", ArrayFloat(shape->vertices, shape->vertex_count * 3)));
        if (shape->vertex_normals)
            attributes.insert(std::make_pair<std::string, ArrayFloat>("normal", ArrayFloat(shape->vertex_normals, shape->vertex_count * 3)));
        index = ArrayInt(shape->triangles, shape->triangle_count * 3);
    }

    std::map<std::string, ArrayFloat> attributes;
    ArrayInt index;

    MSGPACK_DEFINE(attributes, index)
};

struct GeometryMeshBuffer : Geometry
{
    GeometryMeshBuffer() : Geometry("BufferGeometry", ""){};

    GeometryMeshBuffer(shapes::ShapePtr shape_in, const std::string& uuid_in = "") : Geometry("BufferGeometry", uuid_in)
    {
        data = GeometryMeshBufferData(shape_in);
    };

    GeometryMeshBufferData data;
    std::vector<double> matrix = {1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0};
    MSGPACK_DEFINE(uuid, type, data, matrix)
};

template <typename T>
Object<T> CreateGeometryObject(const T& geometry_in, const Material& material_in = Material(), const std::string& uuid_in = "")
{
    Object<T> ret;
    ret.geometries.push_back(geometry_in);
    ret.materials.push_back(material_in);
    ret.object.type = "Mesh";
    ret.object.uuid = uuid_in == "" ? generate_uuid() : uuid_in;
    ret.object.geometry = geometry_in.uuid;
    ret.object.material = material_in.uuid;
    return ret;
};

template <typename T>
MeshObject<T> CreateMeshObject(const T& geometry_in, const Material& material_in = Material(), const std::string& uuid_in = "")
{
    MeshObject<T> ret;
    ret.object = geometry_in;
    ret.object.type = "_meshfile_object";
    return ret;
};

struct SetTransform
{
    SetTransform() = default;
    SetTransform(const std::string& path_in, const std::vector<double>& matrix_in) : type("set_transform"), path(path_in), matrix(matrix_in){};
    std::string type;
    std::string path;
    std::vector<double> matrix;
    MSGPACK_DEFINE(type, path, matrix);
};

struct Key
{
    Key() = default;
    Key(double time_in, const std::vector<double>& value_in) : time(time_in), value(value_in){};
    double time;
    std::vector<double> value;
    MSGPACK_DEFINE(time, value);
};

struct Track
{
    Track() = default;
    Track(const std::string& name_in, const std::string& type_in) : name(name_in), type(type_in){};
    std::string name;
    std::string type;
    std::vector<Key> keys;
    MSGPACK_DEFINE(name, type, keys);
};

struct Clip
{
    Clip() = default;
    Clip(double fps_in, const std::string& name_in) : fps(fps_in), name(name_in){};
    double fps;
    std::string name;
    std::vector<Track> tracks;
    MSGPACK_DEFINE(fps, name, tracks);
};

struct Animation
{
    Animation() = default;
    Animation(const std::string& path_in) : path(path_in){};
    std::string path;
    Clip clip;
    MSGPACK_DEFINE(path, clip);
};

struct AnimationOption
{
    AnimationOption() = default;
    bool play = true;
    int repetitions = 1;
    MSGPACK_DEFINE(play, repetitions);
};

struct SetAnimation
{
    SetAnimation() = default;
    std::string type = "set_animation";
    std::string path = "";
    std::vector<Animation> animations;
    AnimationOption options;
    MSGPACK_DEFINE(type, path, animations, options);
};

struct Delete
{
    Delete() = default;
    Delete(const std::string& path_in) : path(path_in){};
    std::string type = "delete";
    std::string path = "";
    MSGPACK_DEFINE(type, path);
};
}  // namespace visualization
}  // namespace exotica
#endif  // EXOTICA_CORE_VISUALIZATION_MESHCAT_TYPES_H_
