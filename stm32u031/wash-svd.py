#!/usr/bin/python3

import xml.etree.ElementTree as ET
import os, shutil, subprocess, sys

from svdwash import (clusterfy, deprefix, peripheral_derivatives,
                     register_array, register_derivatives)

SVD2RUST='/home/mirror/svd2rust/target/debug/svd2rust'

scriptdir = os.path.dirname(sys.argv[0])
if scriptdir != '':
    os.chdir(scriptdir)

svd = ET.parse('STM32U031.svd')

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

deprefix(svd, alternates_remove, alternates_keep)

dma = svd.find(".//peripheral[name='DMA1']")
clusterfy(dma, 'CH[%s]', ['CR', 'NDTR', 'PAR', 'MAR'],
          [f'CCR{i} CNDTR{i} CPAR{i} CMAR{i}'.split() for i in range(1, 8)])

dmamux = svd.find(".//peripheral[name='DMAMUX']")
register_array(
    dmamux, 'C0CR', 'CCR[%s]', [f'C{i}CR' for i in range(12)]);

tamp = svd.find(".//peripheral[name='TAMP']")
register_array(
    tamp, 'BKP0R', 'BKPR[%s]', [f'BKP{i}R' for i in range(9)])

pwr = svd.find(".//peripheral[name='PWR']")
register_derivatives(pwr, 'PUCRA', ['PUCRB', 'PUCRC'])
register_derivatives(pwr, 'PDCRA', ['PDCRB', 'PDCRC'])

peripheral_derivatives(svd, 'GPIOA', ['GPIOB', 'GPIOC'])

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
                '-f', 'cluster_accessor:::',
                #'-f', 'peripheral_mod:::',
                '-o', 'raw', '-i', 'washed.svd'],
               check=True)
subprocess.run(['form', '-i', 'raw/lib.rs', '-o', 'src'])
subprocess.run(
    ['rustfmt', '--edition', '2021', '--emit', 'files', 'src/lib.rs'])
