#!/usr/bin/env python
from __future__ import print_function
import sys
import os
import re

def to_camel_cased(name):
    r = re.compile(r'([_\/:][a-z]|[_\/:][0-9])')
    ret = ''
    first = True
    tokens = r.split(name, 1)
    while True:
        if len(tokens) > 1:
            if tokens[1][0] == '_':
                if first:
                    ret += tokens[0][0].upper() + tokens[0][1:].lower() + tokens[1][1].upper()
                else:
                    ret += tokens[0].lower() + tokens[1][1].upper()
            else:
                if first:
                    ret += tokens[0][0].upper() + tokens[0][1:].lower() + tokens[1][0] + tokens[1][1].upper()
                else:
                    ret += tokens[0].lower() + tokens[1][0] + tokens[1][1].upper()
            tokens = r.split(tokens[2], 1)
        else:
            if first:
                if len(tokens) > 1:
                    ret += tokens[0][0].upper() + tokens[0][1:].lower() + tokens[1][0]
                else:
                    ret += tokens[0][0].upper() + tokens[0][1:].lower()
            else:
                ret += tokens[0].lower()
            return ret
        first = False

def to_underscores(name, num_pass = 0):
    r = re.compile(r'([A-Za-z][0-9]|[0-9][A-z]|[a-z][A-Z]|[A-Z][A-Z][a-z]|[_\/:][A-z])')
    ret = ''
    tokens = r.split(name, 1)
    while True:
        if len(tokens) > 1:
            if tokens[1][0] == '/' or tokens[1][0] == ':' or tokens[1][0] == '_' or tokens[1][1] == '_':
                ret += tokens[0].lower() + tokens[1][0].lower() + tokens[1][1:].lower()
            else:
                ret += tokens[0].lower() + tokens[1][0].lower() + '_' + tokens[1][1:].lower()
            tokens = r.split(tokens[2], 1)
        else:
            ret += tokens[0].lower()
            if num_pass < 1:
                return to_underscores(ret, num_pass + 1)
            else:
                return ret

def eprint(*args, **kwargs):
    print(*args, file = sys.stderr, **kwargs)

def eprint(msg):
    sys.stderr.write(msg+'\n')
    sys.exit(2)

def constructor_argument_list(data):
    ret = ""
    for d in data:
        if 'Required' in d:
            ret += " " + d['Type'] + " _" + (d['Name']) + default_argument_value(d) + ","
    return ret[0:-1]

def constructor_list(data):
    ret=""
    for d in data:
        if 'Required' in d:
            ret += ",\n        " + (d['Name']) + "(_" + (d['Name']) + ") "
    return ret

def default_value(data):
    if data['Value'] is None:
        return ""
    elif data['Value']=='{}':
        return ""
    else:
        return data['Value']

def default_argument_value(data):
    if data['Value'] == None:
        return ""
    else:
        return " = "+data['Value']

def is_required(data):
    if data['Required']:
        return "true"
    else:
        return "false"

def default_constructor_list(data):
    ret = ""
    for d in data:
        if 'Required' in d and not d['Required']:
            ret += ",\n        " + (d['Name']) + "("+default_value(d) + ") "
    return ret

def needs_default_constructor(data):
    for d in data:
        if d['Required']:
            return True
    return False

def declaration(data):
    if 'Required' in data:
      return "    " + data['Type'] + " " + (data['Name']) + ";\n"
    else:
      return ""

def parser(type_in):
    parser = ""
    if type_in == 'std::string':
        return "boost::any_cast<" + type_in + ">(prop.Get())"
    elif type_in == 'exotica::Initializer' or type_in == 'Initializer':
        return "prop.IsInitializerVectorType()?boost::any_cast<std::vector<exotica::Initializer>>(prop.Get()).at(0):boost::any_cast<exotica::Initializer>(prop.Get())"
    elif type_in == 'std::vector<Initializer>' or type_in == 'std::vector<exotica::Initializer>':
        return "boost::any_cast<std::vector<exotica::Initializer>>(prop.Get())"
    elif type_in == 'Eigen::VectorXd':
        parser = "ParseVector<double,Eigen::Dynamic>"
    elif type_in == 'Eigen::Vector4d':
        parser = "ParseVector<double,4>"
    elif type_in == 'Eigen::Vector3d':
        parser = "ParseVector<double,3>"
    elif type_in == 'Eigen::Vector2d':
        parser = 'ParseVector<double,2>'
    elif type_in == 'Eigen::VectorXi':
        parser = "ParseVector<int,Eigen::Dynamic>"
    elif type_in == 'bool':
        parser = "ParseBool"
    elif type_in == 'double':
        parser = "ParseDouble"
    elif type_in == 'int':
        parser = "ParseInt"
    elif type_in == 'std::vector<std::string>':
        parser = "ParseList"
    elif type_in == 'std::vector<int>':
        parser = "ParseIntList"
    elif type_in == 'std::vector<bool>':
        parser = "ParseBoolList"
    else:
        eprint("Unknown data type '" + type_in + "'")
        sys.exit(2)

    return "prop.IsStringType()?" + parser + "(boost::any_cast<std::string>(prop.Get())):boost::any_cast<" + type_in + ">(prop.Get())"


def copy(data):
    if 'Required' in data:
      return "        ret.properties_.emplace(\""+data['Name']+"\", Property(\""+data['Name']+"\", "+is_required(data)+", boost::any("+(data['Name'])+")));\n"
    else:
      return ""

def add(data):
    if 'Required' in data:
      return "        if (other.HasProperty(\"" + data['Name'] + "\")) {const Property& prop=other.properties_.at(\"" + data['Name'] + "\"); if(prop.IsSet()) " + (data['Name']) + " = " + parser(data['Type']) + ";}\n"
    else:
      return ""

def check(data, name):
    if 'Required' in data and data['Required']:
      return "        if(!other.HasProperty(\"" + data['Name'] + "\") || !other.properties_.at(\"" + data['Name'] + "\").IsSet()) ThrowPretty(\"Initializer " + name + " requires property " + data['Name'] + " to be set!\");\n"
    else:
      return ""



def construct(namespace, class_name_orig, data, include):
    class_name = class_name_orig + "Initializer"
    ret = """// This file was automatically generated. Do not edit this file!
#ifndef INITIALIZER_""" + namespace.upper() + "_" + to_underscores(class_name).upper() + """_H
#define INITIALIZER_""" + namespace.upper() + "_" + to_underscores(class_name).upper() + """_H

#include <exotica_core/property.h>

namespace exotica
{
inline std::vector<Initializer> Get"""+to_camel_cased(namespace)+"""Initializers();
}

namespace exotica
{

class """ + class_name + " : public InitializerBase"+"""
{
public:
    static std::string GetContainerName() {return """+"\"exotica/"+class_name_orig+"\""+ """ ;}

    """
    if needs_default_constructor(data):
        ret += class_name + "() : InitializerBase()" + default_constructor_list(data) + """
    {
    }

    """
    ret += class_name + "(" + constructor_argument_list(data) + ") : InitializerBase()" + constructor_list(data) + """
    {
    }

    """ + class_name + """(const Initializer& other) : """ + class_name + """()
    {
"""
    for d in data:
        ret += add(d)
    ret += """    }

    virtual Initializer GetTemplate() const
    {
        return (Initializer)""" + class_name + """();
    }

    virtual std::vector<Initializer> GetAllTemplates() const
    {
        return Get""" + to_camel_cased(namespace) + """Initializers();
    }

    virtual void Check(const Initializer& other) const
    {
"""
    for d in data:
        ret += check(d,class_name)
    ret += """    }

    operator Initializer()
    {
        Initializer ret(GetContainerName());
"""
    for d in data:
        ret += copy(d)
    ret += """        return ret;
    }

"""
    for d in data:
        ret += declaration(d)
    ret += "};" + """

}

#include<""" + namespace + "/" + namespace + """_initializers_numerator.h>

"""
    for i in include:
        ret += "#include <"+i+".h>\n"
    ret += """
#endif"""
    return ret

def parse_line(line, line_number, function_name):
    # ignore lines with comments, otherwise comments including ';' will not be recognised
    if line.startswith("//"):
        return None

    last = line.find(";")
    if last >= 0:
        line = line[0:last].strip()
    else:
        last = line.find("//")
        if last >= 0:
            line = line[0:last].strip()
        else:
            line=line.strip()

    if len(line) == 0:
        return None

    if line.startswith('include'):
        return {'Include' : line[7:].strip().strip(">").strip("<").strip('"'), 'Code' : line.strip()}
    if line.startswith("extend"):
        return {'Extends' : line[6:].strip().strip(">").strip("<").strip('"'), 'Code' : line.strip()}
    if line.startswith("class"):
        return {'ClassName' : line[5:].strip().strip(">").strip("<").strip('"'), 'Code' : line.strip()}

    if last == -1:
        eprint("Can't find ';' in '" + function_name + "', on line " + str(line_number))
        sys.exit(2)

    required = True
    if line.startswith("Required"):
        required = True
    elif line.startswith("Optional"):
        required = False
    else:
        eprint("Can't parse 'Required/Optional' tag in '" + function_name + "', on line " + str(line_number))
        sys.exit(2)

    value = None
    field_type = ""
    name = ""
    if not required:
        eq = line.find("=")
        has_default_arg = not (eq == -1)
        if has_default_arg:
            value = line[eq + 1:last]
        else:
            # we need this to get the parameter name
            eq = last
        name_start = line[0:eq].strip().rfind(" ")
        name = line[name_start:eq].strip()
        field_type = line[9:name_start].strip()
        if not has_default_arg:
            eprint("Optional parameter '" + name + "' requires a default argument!")
    else:
        name_start = line[0:last].strip().rfind(" ")
        name = line[name_start:last].strip()
        field_type = line[9:name_start].strip()

    return {'Required' : required, 'Type' : field_type, 'Name' : name, 'Value' : value}

def parse_file(file_name):
    with open(file_name) as f:
        lines = f.readlines()
    data = []
    include = []
    extends = []
    names = []
    i = 0
    optionalOnly = False
    for l in lines:
        i = i + 1
        d = parse_line(l, i, file_name)
        if d != None:
            if 'Required' in d:
                if d['Required'] == False:
                    optionalOnly = True
                else:
                    if optionalOnly:
                        eprint("Required properties_ have to come before Optional ones, in '" + file_name + "', on line " + str(i))
                        sys.exit(2)
                data.append(d)
            if 'Include' in d:
                include.append(d['Include'])
            if 'Extends' in d:
                extends.append(d['Extends'])
            if 'ClassName' in d:
                names.append(d['ClassName'])
    if len(names) != 1:
        eprint("Could not parse initializer class name in '" + file_name + "'!")
        eprint(names)
        sys.exit(2)
    return {'Data' : data, 'Include' : include, 'Extends' : extends, 'ClassName' : names[0]}

def contains_data(type_name, name, list_in):
    for d in list_in:
        if d['Type'] == type_name and d['Name'] == name:
            return d['Class']
    return False

def contains_include(name, list_in):
    for d in list_in:
        if d == name:
            return True
    return False

def contains_extends(name, list_in):
    for d in list_in:
        if d == name:
            return True
    return False

def collect_extensions(input_files, search_dirs, content):
    file_content = parse_file(input_files)
    class_name = file_content['ClassName']
    if 'Extends' in file_content:
        for e in file_content['Extends']:
            if not contains_extends(e, content['Extends']):
                file_name = None
                ext = e.split('/')
                for d in search_dirs:
                    ff = d + '/share/' + ext[0] + '/init/' + ext[1] + '.in'
                    if os.path.isfile(ff):
                        file_name = ff
                        break
                if not file_name:
                    eprint("Cannot find extension '" + e + "'!")
                content['Extends'].append(e)
                content = collect_extensions(file_name, search_dirs, content)

    if 'Data' in file_content:
      for d in file_content['Data']:
          cls = contains_data(d['Type'], d['Name'], content['Data'])
          if cls:
              for e in content['Data']:
                if e['Name'] == d['Name']:
                  e['Value'] = d['Value']
          else:
              d['Class'] = class_name
              content['Data'].append(d)
    if 'Include' in file_content:
        for i in file_content['Include']:
            if not contains_include(i, content['Include']):
                content['Include'].append(i)

    content['ClassName'] = class_name
    return content

def sort_data(data):
    a = []
    b = []
    for d in data:
        if d['Required']:
            a.append(d)
        else:
            b.append(d)
    return a + b

def generate(input_files, output_files, namespace, search_dirs, devel_dir):
    print("Generating " + output_files)
    content = collect_extensions(input_files, search_dirs, {'Data' : [], 'Include' : [], 'Extends' : []})
    txt = construct(namespace, content['ClassName'], sort_data(content['Data']), content['Include'])
    path = os.path.dirname(output_files)
    if not os.path.exists(path):
        os.makedirs(path)
    with open(output_files, "w") as f:
        f.write(txt)
    return content['ClassName']

def create_class_init_header(class_inits, file_name):
    ret = """// This file was automatically generated. Do not edit this file!
#ifndef INITIALIZE_PROJECT_HEADER_""" + namespace.upper() + """_H_$
#define INITIALIZE_PROJECT_HEADER_""" + namespace.upper() + """_H_$

#include <exotica_core/property.h>
"""
    for init in class_inits:
        ret += '#include <' + namespace + '/' + init[0] + '_initializer.h>\n'
    ret += """

namespace exotica
{

inline std::vector<Initializer> Get""" + to_camel_cased(namespace) + """Initializers()
{
    std::vector<Initializer> ret;
"""
    for init in class_inits:
        ret += '    ret.push_back(' + init[1] + 'Initializer().GetTemplate());\n'
    ret += """   return ret;
}

}

#endif
"""
    path = os.path.dirname(file_name)
    if not os.path.exists(path):
        os.makedirs(path)
    with open(file_name, "w") as f:
        f.write(ret)

if __name__ == "__main__":
    if len(sys.argv) > 5:
        offset = 5
        n = int((len(sys.argv) - offset) / 2)
        namespace = sys.argv[1]
        search_dirs = sys.argv[2].split(':')
        devel_dir = sys.argv[3]
        class_inits_header_file = sys.argv[4]
        if not os.path.exists(devel_dir + '/init'):
            os.makedirs(devel_dir + '/init')

        for i in range(0, n):
            input_files = sys.argv[offset + i]
            class_name = os.path.basename(sys.argv[offset + i][0:-3])
            with open(input_files, "r") as fi:
                with open(devel_dir + '/init/' + class_name + '.in', "w") as f:
                    f.write(fi.read())

        class_inits = []
        for i in range(0, n):
            input_files = sys.argv[offset + i]
            output_files = sys.argv[offset + n + i]
            class_file_name = os.path.basename(sys.argv[offset + i][0:-3])
            class_inits.append((class_file_name, generate(input_files, output_files, namespace, search_dirs, devel_dir)))

        create_class_init_header(class_inits, class_inits_header_file)
    else:
      eprint("Initializer generation failure: invalid arguments!")
      sys.exit(1)
