import json

def generate_func_skeleton():
    with open('./Opcodes.json', 'r') as file:
        instructions = json.load(file)
    # print(instructions['unprefixed'])

    mnemonics = []
    for instruction in instructions['unprefixed']:
        mnemonic = instructions['unprefixed'][instruction]['mnemonic']
        if mnemonic.startswith('ILLEGAL'):
            mnemonic = "XXX"
        elif mnemonic.startswith('PREFIX'):
            mnemonic = "PREF"
        while len(mnemonic) < 4:
            mnemonic += ' '
        mnemonics.append(mnemonic)
    
    mnemonics_prefixed = []
    for instruction in instructions['cbprefixed']:
        mnemonic = instructions['cbprefixed'][instruction]['mnemonic']
        if mnemonic.startswith('ILLEGAL'):
            mnemonic = "XXX"
        if mnemonic not in mnemonics:
            while len(mnemonic) < 4:
                mnemonic += ' '
            mnemonics_prefixed.append(mnemonic)

    mnemonics = sorted(list(set(mnemonics)))
    print(mnemonics)
    mnemonics_prefixed = sorted(list(set(mnemonics_prefixed)))
    print(mnemonics_prefixed)

    with open('./instructions.cpp', 'w') as file:
        file.write('#include "sm83.hpp"\n#include <cstdint>\n\n// Unprefixed instructions\n')
        for mnemonic in mnemonics:
            file.write(f'SM83::instr_ret_info SM83::{mnemonic}() {{}}\n')
        file.write('\n// Prefixed instructions\n')
        for mnemonic in mnemonics_prefixed:
            file.write(f'SM83::instr_ret_info SM83::{mnemonic}() {{}}\n')
    
    with open('./instructions.hpp', 'w') as file:
        file.write('// Unprefixed instructions\n')
        counter = 0
        for mnemonic in mnemonics:
            file.write(f'SM83::instr_ret_info {mnemonic}();')
            if counter % 4 == 3:
                file.write('\n')
            else:
                file.write(' ')
            counter += 1
        file.write('\n// Prefixed instructions\n')
        counter = 0
        for mnemonic in mnemonics_prefixed:
            file.write(f'SM83::instr_ret_info {mnemonic}();')
            if counter % 4 == 3:
                file.write('\n')
            else:
                file.write(' ')
            counter += 1
        
def generate_init_arrays():
    with open('./Opcodes.json', 'r') as file:
        instructions = json.load(file)
    counter = 0
    out = """#include "sm83.hpp"\nvoid SM83::fill_instruction_array() {
    instructions = {\n"""
    for instruction in instructions['unprefixed']:
        mnemonic = instructions["unprefixed"][instruction]["mnemonic"]
        if mnemonic.startswith('ILLEGAL'):
            mnemonic = "XXX"
        out += f'{{"{mnemonic}", &SM83::{mnemonic}}}, '
        if counter % 4 == 3:
            out += '\n'
        else:
            out += ' '
        counter += 1
    
    out += """};
    prefix_instructions = {\n"""
    
    
    for instruction in instructions['cbprefixed']:
        mnemonic = instructions["cbprefixed"][instruction]["mnemonic"]
        if mnemonic.startswith('ILLEGAL'):
            mnemonic = "XXX"
        out += f'{{"{mnemonic}", &SM83::{mnemonic}}}, '
        if counter % 4 == 3:
            out += '\n'
        else:
            out += ' '
        counter += 1
    out = out[:-2] + '\n'
    out = out.replace('ILLEGAL', 'XXX')
    out = out.replace('PREFIX', 'PREF')
    out += '};\n}'
    
    with open('../SM83/initialize_arrays.cpp', 'w') as file:
        file.write(out)

if __name__ == "__main__":
    generate_init_arrays()