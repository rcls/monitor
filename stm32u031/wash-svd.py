#!/usr/bin/python3

import xml.etree.ElementTree as ET
import os, shutil, subprocess, sys

SVD2RUST='/home/mirror/svd2rust/target/debug/svd2rust'

alternates_remove = {
    'ADC_CHSELR_ALTERNATE',
    'LPUART_CR1_ALTERNATE',
    'LPUART_ISR_ALTERNATE',
    'USART_CR1_ALTERNATE',
    'USART_ISR_ALTERNATE',
    'TIM2_CCMR1_ALTERNATE1',
    'TIM3_CCMR1_ALTERNATE1',
    'TIM2_CCMR2_ALTERNATE1',
    'TIM3_CCMR2_ALTERNATE1',
    'TIM2_CNT_ALTERNATE1',
    'TIM3_CNT_ALTERNATE1',
    'TIM15_CCMR1_ALTERNATE1',
    'TIM16_CCMR1_ALTERNATE1',
}
alternates_keep = {}

def deprefix(svd):
    for registers in svd.findall('.//registers'):
        for register in registers.findall('register'):
            name = register.find('name').text
            if not 'ALTERNATE' in name:
                continue
            if not name in alternates_remove:
                assert name in alternates_keep, f'??? {name}'
                continue
            assert not name in alternates_keep, f'!!! {name}'
            registers.remove(register)

    for peripheral in svd.findall('.//peripheral'):
        print(peripheral.find('name').text)
        names = []
        for register in peripheral.findall('registers/register'):
            names.append(register.find('name'))
            names.append(register.find('displayName'))
        common_prefix = None
        for name in names:
            n = name.text
            if common_prefix is None:
                common_prefix = n
            else:
                while not n.startswith(common_prefix):
                    common_prefix = common_prefix[:-1]
        print(f' {common_prefix}')
        if common_prefix is None or not '_' in common_prefix:
            continue
        common_prefix == common_prefix.rsplit('_', 1)[0] + '_'
        for name in names:
            name.text = name.text.removeprefix(common_prefix)

def register_array(peripheral, first, pattern, items, increment = None):
    assert first in items
    registers = peripheral.find('registers')
    assert registers is not None
    prototype = registers.find(f"register[name='{first}']")
    print(registers.find('register'))
    assert prototype is not None
    prototype.find('name').text = pattern
    assert prototype.find('dim') == None
    assert prototype.find('dimIncrement') == None
    if increment is None:
        increment = int(prototype.find('size').text, 0) // 8
    dim = len(items)
    ET.SubElement(prototype, 'dim').text = f'{dim}'
    ET.SubElement(prototype, 'dimIncrement').text = f'{increment}'
    children = registers.findall('register')
    assert type(children) == list
    for r in children:
        name = r.find('name')
        if name != first and name in items:
            registers.remove(r)

scriptdir = os.path.dirname(sys.argv[0])
if scriptdir != '':
    os.chdir(scriptdir)

svd = ET.parse('STM32U031.svd')

deprefix(svd)

#dmamux = svd.find(".//peripheral[name='DMAMUX']")
#register_array(
#    dmamux, 'C0CR', 'CCR[%s]', [f'C{i}CR' for i in range(12)]);
#register_array(
#    dmamux, 'RG0CR', 'RGCR[%s]', [f'RG{i}CR' for i in range(4)]);

svd.write('washed.svd')

assert os.path.exists('wash-svd.py')

shutil.rmtree('raw', ignore_errors=True)
shutil.rmtree('src', ignore_errors=True)
os.mkdir('raw')
os.mkdir('src')

subprocess.run([SVD2RUST, '--ident-formats-theme', 'legacy',
                '-f', 'register_accessor:::',
                '-f', 'field_accessor:::',
                '-f', 'enum_value:::',
                '-f', 'enum_value_accessor:::',
                '-o', 'raw', '-i', 'washed.svd'],
               check=True)
subprocess.run(['form', '-i', 'raw/lib.rs', '-o', 'src'])
subprocess.run(
    ['rustfmt', '--edition', '2021', '--emit', 'files', 'src/lib.rs'])
