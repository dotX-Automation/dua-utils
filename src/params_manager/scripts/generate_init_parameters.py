import argparse
import yaml
import sys

##################################################################
# cpp
##################################################################

defaults = {
    'bool': '{{default_value}}',
    'double': '{{default_value}}, {{min_value}}, {{max_value}}, {{step}}',
    'integer': '{{default_value}}, {{min_value}}, {{max_value}}, {{step}}',
    'string': '"{{default_value}}"',
    'bool_array': '{{{default_value}}}',
    'double_array': '{{{default_value}}}, {{min_value}}, {{max_value}}, {{step}}',
    'integer_array': '{{{default_value}}}, {{min_value}}, {{max_value}}, {{step}}',
    'string_array': '{"{{default_value}}"}'
}

params_decl = """\
  // {{description}}
  {{manager_name}}->declare_{{type}}_parameter(
    "{{param_name}}",
    {{param_values}},
    "{{description}}",
    "{{constraints}}",
    {{read_only}},
    {{validator}});

"""

##################################################################

cpp_code = """\
#include <{{header_include_path}}>
{{namespace1}}
/**
 * @brief Routine to initialize node parameters.
 */
void {{node_class_name}}::init_parameters()
{
{{params_declarations}}
}
{{namespace2}}"""

##################################################################
# Script
##################################################################

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('config_file', type=str, help='Configuration file absolute path', default='')
    if len(sys.argv) != 2:
        parser.print_usage()
        exit(-1)
    args = parser.parse_args()
    yaml_file = args.config_file

    # Open yaml file and read data
    try:
        with open(yaml_file) as f:
            yaml_data = yaml.load(f)
    except:
        print('Failed to parse yaml file. Terminating.')
        exit(-1)

    # Get general configuration
    header_include_path = yaml_data.get('header_include_path')
    if header_include_path == None:
        print('Invalid header_include_path')
        exit(-1)

    manager_name = yaml_data.get('manager_name', 'pmanager_')

    namespace = yaml_data.get('namespace', '')

    node_class_name = yaml_data.get('node_class_name')
    if node_class_name == None:
        print('Invalid node_class_name')
        exit(-1)

    output_dir = yaml_data.get('output_dir')
    if output_dir == None:
        print('Invalid output_dir')
        exit(-1)

    # Get ordered params dictionary
    params_dict = dict(sorted(yaml_data['params'].items()))

    params_decls_cpp = ''
    
    for (param_name, values) in params_dict.items():
        try:
            # Parameters declarations
            if values['type'] in ['integer', 'double', 'integer_array', 'double_array']:
                param_values = defaults[values['type']].replace('{{default_value}}', str(values['default_value']))\
                                                    .replace('{{min_value}}', str(values['min_value']))\
                                                    .replace('{{max_value}}', str(values['max_value']))\
                                                    .replace('{{step}}', str(values['step']))
            else:
                param_values = defaults[values['type']].replace(
                    '{{default_value}}', str(values['default_value']).lower())

            params_decls_cpp += params_decl.replace('{{description}}', values['description'])\
                                        .replace('{{type}}', values['type'])\
                                        .replace('{{param_name}}', param_name)\
                                        .replace('{{param_values}}', param_values)\
                                        .replace('{{description}}', values['description'])\
                                        .replace('{{constraints}}', values['constraints'])\
                                        .replace('{{manager_name}}', manager_name)\
                                        .replace('{{validator}}', values.get('validator', 'nullptr'))\
                                        .replace('{{read_only}}', str(values['read_only']).lower())
        except:
            print(f'Invalid configuration for parameter: {param_name}')
            exit(-1)

    cpp_code = cpp_code.replace('{{params_declarations}}', params_decls_cpp[:-2])
    cpp_code = cpp_code.replace('{{node_class_name}}', node_class_name)
    cpp_code = cpp_code.replace('{{header_include_path}}', header_include_path)
    if namespace != '':
        namespace1 = '\nnamespace '+namespace+'\n{\n'
        namespace2 = '\n} // namespace '+namespace+'\n'
        cpp_code = cpp_code.replace('{{namespace1}}', namespace1)
        cpp_code = cpp_code.replace('{{namespace2}}', namespace2)
    else:
        cpp_code = cpp_code.replace('{{namespace1}}', namespace)
        cpp_code = cpp_code.replace('{{namespace2}}', namespace)

    # Create cpp file
    with open(f'{output_dir}/init_parameters.cpp', 'w') as f:
        f.write(cpp_code)
