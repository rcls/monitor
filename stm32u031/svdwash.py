#!/usr/bin/python3

import xml.etree.ElementTree as ET

def deprefix(svd, alternates_remove, alternates_keep):
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
        if name != first and name.text in items:
            registers.remove(r)

def register_derivatives(peripheral, prototype: str, derived: [str]):
    registers = peripheral.find('registers')
    assert registers is not None
    proto = registers.find(f"register[name='{prototype}']")
    assert proto is not None
    for d in derived:
        reg = registers.find(f"register[name='{d}']")
        assert reg != proto, f'{prototype} {d} {proto} {reg}'
        reg.set('derivedFrom', prototype)
        reg.remove(reg.find('fields'))

def peripheral_derivatives(svd, prototype: str, derived: [str]):
    peripherals = svd.find('.//peripherals')
    proto = peripherals.find(f"peripheral[name='{prototype}']")
    assert proto is not None
    for d in derived:
        periph = peripherals.find(f"peripheral[name='{d}']")
        assert periph != proto
        periph.set('derivedFrom', prototype)
        periph.remove(periph.find('registers'))

def num_field(n, f):
    return int(n.find(f).text, 0)
def addressoffset(n):
    return num_field(n, 'addressOffset')

def clusterfy(peripheral, name: str, fields: [str], replaced: [[str]]):
    registers = peripheral.find('registers')
    assert registers is not None

    reg_by_name = {}
    for r in registers.findall('register'):
        reg_by_name[r.find('name').text] = r
    #print(reg_by_name)

    for r in replaced:
        assert len(r) == len(fields)
    assert len(replaced) >= 2, "We don't handle singletons..."

    rep_regs = [list(map(reg_by_name.get, rr)) for rr in replaced]
    #print(rep_regs)

    # Check that the register have regular strides.
    base = addressoffset(rep_regs[0][0])
    stride = addressoffset(rep_regs[1][0]) - base
    # Not sure if this is essential.
    for r, s in zip(rep_regs[0][:-1], rep_regs[0][1:]):
        print(r, s)
        assert addressoffset(r) <= addressoffset(s)
    # Check that the addresses all match for clusterification.
    for rr, ss in zip(rep_regs[:-1], rep_regs[1:]):
        for r, s in zip(rr, ss):
            assert addressoffset(s) - addressoffset(r) == stride
            assert num_field(r, 'size') == num_field(s, 'size')
    # Now build the cluster.
    # Ugh...
    index = None
    for n, e in enumerate(registers):
        if e == rep_regs[0][0]:
            index = n
            break
    else:
        assert None, 'Bugger'
    cluster = ET.Element('cluster')
    registers.insert(index, cluster)
    for rr in rep_regs:
        for r in rr:
            registers.remove(r)
    ET.SubElement(cluster, 'dim').text = str(len(rep_regs))
    ET.SubElement(cluster, 'dimIncrement').text = str(stride)
    ET.SubElement(cluster, 'name').text = name
    ET.SubElement(cluster, 'description').text = f'Cluster for {name}'
    ET.SubElement(cluster, 'addressOffset').text = f'{base:#x}'
    for r, n in zip(rep_regs[0], fields):
        cluster.append(r)
        r.find('name').text = n
        r.find('displayName').text = n
        r.find('addressOffset').text = hex(addressoffset(r) - base)
